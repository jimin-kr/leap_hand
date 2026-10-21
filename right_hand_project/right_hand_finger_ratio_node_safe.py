import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

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


PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Position PID / Goal Current (RAM, 2 bytes each)
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102

LEN_GOAL_POSITION = 4

TORQUE_OFF = 0
TORQUE_ON = 1

FINGER_TOPIC = "/right_hand/finger_ratios"
ARM_TOPIC = "/right_hand/arm"
FALLBACK_FINGER_TOPIC = "/left_hand/finger_ratios"
FALLBACK_ARM_TOPIC = "/left_hand/arm"

FINGER_NAMES = ["thumb", "index", "middle", "ring"]

# Shared calibration (open = 2048 for all motors), same file the RViz node uses.
DEFAULT_MOTOR_MAP = (
    Path(__file__).resolve().parent.parent
    / "leap_teleop" / "config" / "right_hand_motor_map.json"
)


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    while delta > 180.0:
        delta -= 360.0

    while delta < -180.0:
        delta += 360.0

    return delta


def deg_to_tick(deg):
    deg = deg % 360.0
    return int(round(deg / 360.0 * 4096.0)) % 4096


def load_motor_map(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_id_list(motor_map):
    ids = []

    for finger in FINGER_NAMES:
        ids.extend(motor_map["ids"][finger])

    return ids


def wrap_tick_delta(delta):
    while delta > 2048:
        delta -= 4096
    while delta < -2048:
        delta += 4096
    return delta


def map_open_tick(motor_map, key):
    if "open_pose_tick" in motor_map:
        return int(motor_map["open_pose_tick"][key])
    return deg_to_tick(float(motor_map["open_pose_deg"][key]))


def map_delta_tick(motor_map, key):
    """motor map 기준 open -> close 이동량(tick). 절대값이 아니라 차이량만 사용."""
    if "open_pose_tick" in motor_map and "close_pose_tick" in motor_map:
        return wrap_tick_delta(
            int(motor_map["close_pose_tick"][key]) - int(motor_map["open_pose_tick"][key])
        )
    delta_deg = shortest_angle_delta(
        float(motor_map["open_pose_deg"][key]), float(motor_map["close_pose_deg"][key])
    )
    return int(round(delta_deg / 360.0 * 4096.0))


def make_target_ticks(motor_map, ratios, base_ticks):
    """target = base_open_tick(시작 시 스캔) + map delta * ratio"""
    target = {}

    for finger in FINGER_NAMES:
        ratio = clamp(ratios[finger])

        for dxl_id in motor_map["ids"][finger]:
            dxl_id = int(dxl_id)
            key = str(dxl_id)
            delta = map_delta_tick(motor_map, key)
            target[dxl_id] = int(round(base_ticks[dxl_id] + delta * ratio)) % 4096

    return target


class LeftHandSafeFingerRatioNode(Node):
    def __init__(self, args):
        super().__init__("left_hand_finger_ratio_node_safe")

        self.args = args
        self.motor_map = load_motor_map(Path(args.motor_map))

        self.port_name = args.port
        self.baudrate = args.baudrate if args.baudrate is not None else int(self.motor_map.get("baudrate", 4000000))
        self.profile_velocity = args.profile_velocity
        self.profile_acceleration = args.profile_acceleration
        self.thumb_profile_velocity = args.thumb_profile_velocity
        self.thumb_profile_acceleration = args.thumb_profile_acceleration
        self.thumb_bend_velocity = args.thumb_bend_velocity
        self.thumb_bend_acceleration = args.thumb_bend_acceleration

        self.dxl_ids = build_id_list(self.motor_map)
        self.thumb_ids = set(self.motor_map.get("ids", {}).get("thumb", [15, 14, 13, 12]))
        self.thumb_bend_ids = {14, 15}

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.armed = False

        self.last_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        self.open_hardware()

        self.finger_sub = self.create_subscription(
            Float32MultiArray,
            FINGER_TOPIC,
            self.finger_callback,
            10,
        )
        self.fallback_finger_sub = self.create_subscription(
            Float32MultiArray,
            FALLBACK_FINGER_TOPIC,
            self.finger_callback,
            10,
        )

        self.arm_sub = self.create_subscription(
            Bool,
            ARM_TOPIC,
            self.arm_callback,
            10,
        )
        self.fallback_arm_sub = self.create_subscription(
            Bool,
            FALLBACK_ARM_TOPIC,
            self.arm_callback,
            10,
        )

        self.get_logger().info("Right Hand SAFE Finger Ratio Node started")
        self.get_logger().info(f"Subscribed: {FINGER_TOPIC} (fallback: {FALLBACK_FINGER_TOPIC})")
        self.get_logger().info(f"Subscribed: {ARM_TOPIC} (fallback: {FALLBACK_ARM_TOPIC})")
        self.get_logger().info("Initial state: DISARMED")
        self.get_logger().info("Arm command:")
        self.get_logger().info(
            'ros2 topic pub --once /right_hand/arm std_msgs/msg/Bool "{data: true}"'
        )
        self.get_logger().info("Disarm command:")
        self.get_logger().info(
            'ros2 topic pub --once /right_hand/arm std_msgs/msg/Bool "{data: false}"'
        )

    def open_hardware(self):
        if not self.port_handler.openPort():
            raise RuntimeError(f"port open failed: {self.port_name}")

        self.get_logger().info(f"port open ok: {self.port_name}")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"baudrate set failed: {self.baudrate}")

        self.get_logger().info(f"baudrate set ok: {self.baudrate}")

        self.torque_off_all()
        self.set_profile_velocity_all()
        self.set_profile_acceleration_all()
        self.set_gains_all()
        self.capture_open_baseline()

        self.get_logger().info("hardware ready, torque OFF, DISARMED")

    def set_gains_all(self):
        items = [
            (ADDR_POSITION_P_GAIN, self.args.kp, "P gain"),
            (ADDR_POSITION_I_GAIN, self.args.ki, "I gain"),
            (ADDR_POSITION_D_GAIN, self.args.kd, "D gain"),
            (ADDR_GOAL_CURRENT, self.args.goal_current, "goal current"),
        ]

        for dxl_id in self.dxl_ids:
            for addr, value, label in items:
                result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, dxl_id, addr, int(value)
                )
                if result != COMM_SUCCESS:
                    raise RuntimeError(
                        f"ID {dxl_id} {label}: {self.packet_handler.getTxRxResult(result)}"
                    )
                if error != 0:
                    raise RuntimeError(
                        f"ID {dxl_id} {label}: {self.packet_handler.getRxPacketError(error)}"
                    )

        self.get_logger().info(
            f"gains P={self.args.kp} I={self.args.ki} D={self.args.kd}, "
            f"goal current={self.args.goal_current}mA"
        )

    def capture_open_baseline(self):
        """시작 시(torque OFF) 현재 엔코더 값을 open 기준으로 저장. 노드 실행 시 손을 편 상태로 두어야 함."""
        if self.args.use_map_open:
            self.base_ticks = {
                int(i): map_open_tick(self.motor_map, str(i)) for i in self.dxl_ids
            }
            self.get_logger().info("open baseline: motor map open_pose_tick")
            return

        self.base_ticks = self.read_present_positions()
        self.get_logger().info("open baseline: encoder scan at startup")

        for dxl_id in sorted(self.base_ticks):
            diff = wrap_tick_delta(
                self.base_ticks[dxl_id] - map_open_tick(self.motor_map, str(dxl_id))
            )
            line = f"  ID {dxl_id:>2}: scanned {self.base_ticks[dxl_id]:>4} (map open diff {diff:+5d})"
            if abs(diff) > 341:  # ~30 deg
                self.get_logger().warn(line + "  <-- map open 과 차이 큼. 손가락 펴져 있는지 확인!")
            else:
                self.get_logger().info(line)

    def write1(self, dxl_id, address, value, label):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            value,
        )

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getTxRxResult(result)}"
            )

        if error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getRxPacketError(error)}"
            )

    def write4(self, dxl_id, address, value, label):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
            int(value),
        )

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getTxRxResult(result)}"
            )

        if error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getRxPacketError(error)}"
            )

    def read4(self, dxl_id, address, label):
        value, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler,
            dxl_id,
            address,
        )

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getTxRxResult(result)}"
            )

        if error != 0:
            raise RuntimeError(
                f"ID {dxl_id} {label}: {self.packet_handler.getRxPacketError(error)}"
            )

        return int(value)

    def torque_on_all(self):
        for dxl_id in self.dxl_ids:
            self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON, "torque on")

        self.get_logger().info("torque ON all")

    def torque_off_all(self):
        for dxl_id in self.dxl_ids:
            try:
                self.write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_OFF, "torque off")
            except Exception as e:
                self.get_logger().warn(f"ID {dxl_id} torque off warning (proceeding to others): {e}")

        self.get_logger().info("torque OFF all completed")

    def set_profile_velocity_all(self):
        for dxl_id in self.dxl_ids:
            if dxl_id in self.thumb_bend_ids:
                vel = self.thumb_bend_velocity
            elif dxl_id in self.thumb_ids:
                vel = self.thumb_profile_velocity
            else:
                vel = self.profile_velocity

            self.write4(
                dxl_id,
                ADDR_PROFILE_VELOCITY,
                vel,
                "profile velocity",
            )

        self.get_logger().info(
            f"profile velocity: fingers={self.profile_velocity}, "
            f"thumb_rot(12,13)={self.thumb_profile_velocity}, "
            f"thumb_bend(14,15)={self.thumb_bend_velocity}"
        )

    def set_profile_acceleration_all(self):
        for dxl_id in self.dxl_ids:
            if dxl_id in self.thumb_bend_ids:
                acc = self.thumb_bend_acceleration
            elif dxl_id in self.thumb_ids:
                acc = self.thumb_profile_acceleration
            else:
                acc = self.profile_acceleration

            self.write4(
                dxl_id,
                ADDR_PROFILE_ACCELERATION,
                acc,
                "profile acceleration",
            )

        self.get_logger().info(
            f"profile acceleration: fingers={self.profile_acceleration}, "
            f"thumb_rot(12,13)={self.thumb_profile_acceleration}, "
            f"thumb_bend(14,15)={self.thumb_bend_acceleration}"
        )

    def read_present_positions(self):
        positions = {}

        for dxl_id in self.dxl_ids:
            tick = self.read4(dxl_id, ADDR_PRESENT_POSITION, "present position")
            positions[int(dxl_id)] = int(tick) % 4096

        return positions

    def sync_write_goal_ticks(self, goal_ticks):
        group = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        for dxl_id, tick in goal_ticks.items():
            tick = int(tick) % 4096

            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(tick)),
                DXL_HIBYTE(DXL_LOWORD(tick)),
                DXL_LOBYTE(DXL_HIWORD(tick)),
                DXL_HIBYTE(DXL_HIWORD(tick)),
            ]

            ok = group.addParam(int(dxl_id), param_goal_position)

            if not ok:
                raise RuntimeError(f"ID {dxl_id} groupSyncWrite addParam failed")

        result = group.txPacket()

        if result != COMM_SUCCESS:
            raise RuntimeError(
                f"groupSyncWrite failed: {self.packet_handler.getTxRxResult(result)}"
            )

        group.clearParam()

    def sync_write_goal_deg(self, goal_deg):
        goal_ticks = {}

        for dxl_id, deg in goal_deg.items():
            goal_ticks[int(dxl_id)] = deg_to_tick(float(deg))

        self.sync_write_goal_ticks(goal_ticks)

    def arm(self):
        if self.armed:
            self.get_logger().info("already ARMED")
            return

        self.get_logger().warn("ARM requested")

        current_ticks = self.read_present_positions()

        self.torque_on_all()

        time.sleep(0.05)

        self.sync_write_goal_ticks(current_ticks)

        self.armed = True

        self.get_logger().warn("ARMED: torque ON, current pose hold")

    def disarm(self):
        if not self.armed:
            self.get_logger().info("already DISARMED")
            return

        self.get_logger().warn("DISARM requested")

        self.torque_off_all()

        self.armed = False

        self.get_logger().warn("DISARMED: torque OFF")

    def arm_callback(self, msg):
        if msg.data:
            try:
                self.arm()
            except Exception as e:
                self.get_logger().error(f"ARM failed: {e}")
                self.armed = False

                try:
                    self.torque_off_all()
                except Exception:
                    pass
        else:
            try:
                self.disarm()
            except Exception as e:
                self.get_logger().error(f"DISARM failed: {e}")

    def finger_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(
                "finger ratio message must have 4 values: [thumb, index, middle, ring]"
            )
            return

        ratios = {
            "thumb": clamp(msg.data[0]),
            "index": clamp(msg.data[1]),
            "middle": clamp(msg.data[2]),
            "ring": clamp(msg.data[3]),
        }

        self.last_ratios = ratios

        if not self.armed:
            self.get_logger().info(
                "finger ratio received but node is DISARMED. command ignored."
            )
            return

        try:
            target_ticks = make_target_ticks(self.motor_map, ratios, self.base_ticks)
            self.sync_write_goal_ticks(target_ticks)

            self.get_logger().info(
                f"sent ratios: "
                f"thumb={ratios['thumb']:.2f}, "
                f"index={ratios['index']:.2f}, "
                f"middle={ratios['middle']:.2f}, "
                f"ring={ratios['ring']:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"failed to send finger ratios: {e}")

    def cleanup(self):
        self.get_logger().warn("cleanup: torque OFF and close port")

        try:
            self.torque_off_all()
        except Exception as e:
            self.get_logger().error(f"cleanup torque off failed: {e}")

        try:
            self.port_handler.closePort()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--profile-velocity", type=int, default=10, help="기타 손가락 속도 (기본: 10)")
    parser.add_argument("--profile-acceleration", type=int, default=10, help="기타 손가락 가감속 (기본: 10)")
    parser.add_argument("--thumb-profile-velocity", type=int, default=15, help="엄지 회전(12, 13번) 속도 (기본: 15)")
    parser.add_argument("--thumb-profile-acceleration", type=int, default=15, help="엄지 회전(12, 13번) 가감속 (기본: 15)")
    parser.add_argument("--thumb-bend-velocity", type=int, default=7, help="엄지 굽힘(14, 15번) 속도 (기본: 7)")
    parser.add_argument("--thumb-bend-acceleration", type=int, default=7, help="엄지 굽힘(14, 15번) 가감속 (기본: 7)")
    parser.add_argument("--kp", type=int, default=600, help="Position P gain")
    parser.add_argument("--ki", type=int, default=0, help="Position I gain")
    parser.add_argument("--kd", type=int, default=200, help="Position D gain (진동 억제)")
    parser.add_argument("--goal-current", type=int, default=350, help="최대 전류 [mA]")
    parser.add_argument("--motor-map", type=str, default=str(DEFAULT_MOTOR_MAP))
    parser.add_argument(
        "--use-map-open",
        action="store_true",
        help="시작 시 엔코더 스캔 대신 motor map 의 open_pose_tick 을 open 기준으로 사용",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = None

    try:
        node = LeftHandSafeFingerRatioNode(args)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
