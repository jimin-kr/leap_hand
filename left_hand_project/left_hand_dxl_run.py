import argparse
import json
import time
from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    COMM_SUCCESS,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
    DXL_HIWORD,
)


# Shared calibration (open = 2048 for all motors)
MAP_FILE = (
    Path(__file__).resolve().parent.parent
    / "leap_teleop" / "config" / "left_hand_motor_map.json"
)

PROTOCOL_VERSION = 2.0

# XC330-M288 / Protocol 2.0 control table
ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Position PID / Goal Current (RAM, 2 bytes each) - 매 실행마다 써줘야 함
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102

# 공식 LEAP Hand 기본값 (진동 억제용 D gain 포함)
DEFAULT_KP = 600
DEFAULT_KI = 0
DEFAULT_KD = 200
DEFAULT_GOAL_CURRENT = 350  # mA

# 스캔한 open 값이 map의 open 값과 이 이상 차이 나면 경고 (손이 펴진 상태가 아닐 수 있음)
OPEN_MISMATCH_WARN_TICK = 341  # ~30 deg

LEN_GOAL_POSITION = 4

TORQUE_OFF = 0
TORQUE_ON = 1

FINGER_IDS = {
    "thumb": [15, 14, 13, 12],
    "index": [3, 2, 1, 0],
    "middle": [7, 6, 5, 4],
    "ring": [11, 10, 9, 8],
    "all": list(range(16)),
}


def shortest_angle_delta(open_deg, close_deg):
    """
    360도 경계에서 가장 짧은 방향으로 이동하게 만드는 함수.
    예: 360.53 -> 32.87 은 -327.66도가 아니라 +32.34도로 처리.
    """
    delta = close_deg - open_deg

    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    return delta


def deg_to_tick(deg):
    """
    0~360 deg를 Dynamixel position tick 0~4095로 변환.
    """
    deg = deg % 360.0
    # 4096 ticks/rev, same as left_hand_finger_ratio_node_safe.py (180 deg -> 2048)
    return int(round(deg / 360.0 * 4096.0)) % 4096


def wrap_tick_delta(delta):
    """tick 차이를 [-2048, 2048] 범위의 최단 방향으로 정리."""
    while delta > 2048:
        delta -= 4096
    while delta < -2048:
        delta += 4096
    return delta


def map_open_tick(data, key):
    if "open_pose_tick" in data:
        return int(data["open_pose_tick"][key])
    return deg_to_tick(float(data["open_pose_deg"][key]))


def map_delta_tick(data, key):
    """motor map 기준 open -> close 이동량(tick). 절대값이 아니라 차이량만 사용."""
    if "open_pose_tick" in data and "close_pose_tick" in data:
        return wrap_tick_delta(int(data["close_pose_tick"][key]) - int(data["open_pose_tick"][key]))

    delta_deg = shortest_angle_delta(
        float(data["open_pose_deg"][key]), float(data["close_pose_deg"][key])
    )
    return int(round(delta_deg / 360.0 * 4096.0))


def make_target_pose(data, ratio, selected_ids, base_ticks=None):
    """
    target = base_open_tick + (map close - map open) * ratio
    base_ticks 가 있으면 (시작 시 스캔한 실제 엔코더 값) 그걸 open 기준으로 사용.
    없으면 map 의 open_pose_tick 사용.
    """
    targets = {}

    for motor_id in selected_ids:
        key = str(motor_id)

        if base_ticks is not None:
            open_tick = int(base_ticks[motor_id])
        else:
            open_tick = map_open_tick(data, key)

        delta_tick = map_delta_tick(data, key)
        target_tick = int(round(open_tick + delta_tick * ratio)) % 4096

        targets[motor_id] = {
            "open_tick": open_tick,
            "delta_tick": delta_tick,
            "deg": round(target_tick / 4096.0 * 360.0, 2),
            "tick": target_tick,
        }

    return targets


def read_present_ticks(packet_handler, port_handler, ids):
    ticks = {}

    for dxl_id in ids:
        value, result, error = packet_handler.read4ByteTxRx(
            port_handler, dxl_id, ADDR_PRESENT_POSITION
        )
        if not check_comm(packet_handler, dxl_id, result, error, "present position"):
            return None
        ticks[dxl_id] = int(value) % 4096

    return ticks


def parse_ids(args, data=None):
    """
    --ids 0,1,2,3 이 있으면 그걸 우선 사용.
    없으면 motor_map의 ids 또는 FINGER_IDS 기준 사용.
    """
    if args.ids:
        return [int(x.strip()) for x in args.ids.split(",") if x.strip() != ""]

    if data and "ids" in data and args.finger in data["ids"]:
        return data["ids"][args.finger]

    return FINGER_IDS[args.finger]


def check_comm(packet_handler, dxl_id, result, error, action):
    if result != COMM_SUCCESS:
        print(f"[ERROR] ID {dxl_id} {action}: {packet_handler.getTxRxResult(result)}")
        return False

    if error != 0:
        print(f"[ERROR] ID {dxl_id} {action}: {packet_handler.getRxPacketError(error)}")
        return False

    return True


def set_torque(packet_handler, port_handler, ids, enable):
    for dxl_id in ids:
        result, error = packet_handler.write1ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_ON if enable else TORQUE_OFF,
        )

        ok = check_comm(
            packet_handler,
            dxl_id,
            result,
            error,
            "torque on" if enable else "torque off",
        )

        if not ok:
            return False

    return True


def set_gains(packet_handler, port_handler, ids, kp, ki, kd, goal_current):
    items = [
        (ADDR_POSITION_P_GAIN, kp, "P gain"),
        (ADDR_POSITION_I_GAIN, ki, "I gain"),
        (ADDR_POSITION_D_GAIN, kd, "D gain"),
        (ADDR_GOAL_CURRENT, goal_current, "goal current"),
    ]

    for dxl_id in ids:
        for addr, value, label in items:
            result, error = packet_handler.write2ByteTxRx(port_handler, dxl_id, addr, int(value))
            if not check_comm(packet_handler, dxl_id, result, error, label):
                return False

    return True


def set_profile_velocity(packet_handler, port_handler, ids, velocity):
    for dxl_id in ids:
        result, error = packet_handler.write4ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_PROFILE_VELOCITY,
            velocity,
        )

        ok = check_comm(
            packet_handler,
            dxl_id,
            result,
            error,
            "profile velocity",
        )

        if not ok:
            return False

    return True


def set_profile_acceleration(packet_handler, port_handler, ids, acceleration):
    for dxl_id in ids:
        result, error = packet_handler.write4ByteTxRx(
            port_handler,
            dxl_id,
            ADDR_PROFILE_ACCELERATION,
            acceleration,
        )

        ok = check_comm(
            packet_handler,
            dxl_id,
            result,
            error,
            "profile acceleration",
        )

        if not ok:
            return False

    return True


def sync_write_goal_position(packet_handler, port_handler, targets):
    group = GroupSyncWrite(
        port_handler,
        packet_handler,
        ADDR_GOAL_POSITION,
        LEN_GOAL_POSITION,
    )

    for dxl_id, target in targets.items():
        tick = int(target["tick"])

        param_goal_position = [
            DXL_LOBYTE(DXL_LOWORD(tick)),
            DXL_HIBYTE(DXL_LOWORD(tick)),
            DXL_LOBYTE(DXL_HIWORD(tick)),
            DXL_HIBYTE(DXL_HIWORD(tick)),
        ]

        if not group.addParam(dxl_id, param_goal_position):
            print(f"[ERROR] ID {dxl_id} groupSyncWrite addParam failed")
            group.clearParam()
            return False

    result = group.txPacket()

    if result != COMM_SUCCESS:
        print(f"[ERROR] groupSyncWrite txPacket: {packet_handler.getTxRxResult(result)}")
        group.clearParam()
        return False

    group.clearParam()
    return True


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--ratio", type=float, default=0.0)
    parser.add_argument("--finger", choices=list(FINGER_IDS.keys()), default="thumb")
    parser.add_argument("--ids", default=None, help="예: 0 또는 0,1,2,3")
    parser.add_argument("--profile-velocity", type=int, default=30)
    parser.add_argument("--profile-acceleration", type=int, default=10, help="가감속 프로파일 (진동/급발진 방지)")
    parser.add_argument("--kp", type=int, default=DEFAULT_KP, help="Position P gain")
    parser.add_argument("--ki", type=int, default=DEFAULT_KI, help="Position I gain")
    parser.add_argument("--kd", type=int, default=DEFAULT_KD, help="Position D gain (진동 억제)")
    parser.add_argument("--goal-current", type=int, default=DEFAULT_GOAL_CURRENT, help="최대 전류 [mA]")
    parser.add_argument("--hold", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--keep-torque", action="store_true")
    parser.add_argument(
        "--use-map-open",
        action="store_true",
        help="시작 시 엔코더 스캔 대신 motor map 의 open_pose_tick 을 open 기준으로 사용",
    )
    parser.add_argument(
        "--no-return",
        action="store_true",
        help="hold 후 스캔한 open 위치로 복귀하지 않음",
    )

    args = parser.parse_args()

    if not MAP_FILE.exists():
        raise FileNotFoundError(f"motor map 파일 없음: {MAP_FILE}")

    with open(MAP_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    baudrate = args.baudrate if args.baudrate is not None else int(data["baudrate"])
    ratio = max(0.0, min(1.0, args.ratio))
    selected_ids = parse_ids(args, data)

    print()
    print("LEFT HAND DXL RUN")
    print("=================")
    print(f"port      : {args.port}")
    print(f"baudrate  : {baudrate}")
    print(f"ratio     : {ratio}")
    print(f"ids       : {selected_ids}")
    print(f"execute   : {args.execute}")
    print(f"open base : {'motor map' if args.use_map_open else 'encoder scan (now)'}")
    print()

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(PROTOCOL_VERSION)
    port_ok = port_handler.openPort() and port_handler.setBaudRate(baudrate)

    if not port_ok:
        print(f"[ERROR] 포트 열기/baudrate 설정 실패: {args.port} @ {baudrate}")
        port_handler.closePort()
        if not args.execute:
            print("[DRY RUN] 스캔 불가 -> motor map open 기준으로 계산만 표시")
            targets = make_target_pose(data, ratio, selected_ids)
            for dxl_id in selected_ids:
                t = targets[dxl_id]
                print(f"ID {dxl_id:>2}: open {t['open_tick']:>4} + {t['delta_tick']:>5} x {ratio} -> tick {t['tick']:>4} ({t['deg']} deg)")
        return

    base_ticks = None
    moved = False

    try:
        # 1) 시작 시 엔코더 스캔 (torque 는 건드리지 않음 = 읽기만)
        if not args.use_map_open:
            print("[STEP] scan present position (open baseline)")
            base_ticks = read_present_ticks(packet_handler, port_handler, selected_ids)
            if base_ticks is None:
                print("[ERROR] 엔코더 스캔 실패 -> 중단")
                return

            for dxl_id in selected_ids:
                diff = wrap_tick_delta(base_ticks[dxl_id] - map_open_tick(data, str(dxl_id)))
                warn = "  <-- [WARN] map open 과 차이 큼. 손가락 펴져 있는지 확인!" if abs(diff) > OPEN_MISMATCH_WARN_TICK else ""
                print(f"  ID {dxl_id:>2}: scanned {base_ticks[dxl_id]:>4} (map open diff {diff:+5d}){warn}")
            print()

        targets = make_target_pose(data, ratio, selected_ids, base_ticks)

        for dxl_id in selected_ids:
            t = targets[dxl_id]
            print(f"ID {dxl_id:>2}: open {t['open_tick']:>4} + {t['delta_tick']:>5} x {ratio} -> tick {t['tick']:>4} ({t['deg']} deg)")

        if not args.execute:
            print()
            print("[DRY RUN] 실제 모터에는 명령 안 보냄.")
            print("실제로 움직이려면 명령 뒤에 --execute 붙여.")
            return

        print()
        print("[STEP] torque off")
        if not set_torque(packet_handler, port_handler, selected_ids, False):
            return

        print("[STEP] set profile velocity & acceleration")
        if not set_profile_velocity(packet_handler, port_handler, selected_ids, args.profile_velocity):
            return
        if not set_profile_acceleration(packet_handler, port_handler, selected_ids, args.profile_acceleration):
            return

        print(f"[STEP] set gains P={args.kp} I={args.ki} D={args.kd}, goal current={args.goal_current}mA")
        if not set_gains(packet_handler, port_handler, selected_ids, args.kp, args.ki, args.kd, args.goal_current):
            return

        print("[STEP] torque on")
        moved = True
        if not set_torque(packet_handler, port_handler, selected_ids, True):
            return

        print("[STEP] send goal position")
        if not sync_write_goal_position(packet_handler, port_handler, targets):
            return

        print(f"[STEP] hold {args.hold} sec")
        time.sleep(args.hold)

        # 2) 스캔한 open 위치로 복귀 -> 다음 실행 때 스캔값이 누적되지 않도록
        if not args.no_return and not args.keep_torque:
            open_targets = make_target_pose(data, 0.0, selected_ids, base_ticks)
            print("[STEP] return to open baseline")
            sync_write_goal_position(packet_handler, port_handler, open_targets)
            time.sleep(args.hold)

    finally:
        # Ctrl+C 포함 어떤 경우에도 즉시 torque off (keep-torque 제외)
        if moved and not args.keep_torque:
            print("[STEP] torque off")
            set_torque(packet_handler, port_handler, selected_ids, False)

        port_handler.closePort()
        print("[DONE]")


if __name__ == "__main__":
    main()