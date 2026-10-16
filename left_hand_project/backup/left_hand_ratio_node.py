import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

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


MAP_FILE = Path("left_hand_motor_map.json")

PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

LEN_GOAL_POSITION = 4

TORQUE_OFF = 0
TORQUE_ON = 1

FINGER_IDS = {
    "thumb": [0, 1, 2, 3],
    "index": [4, 5, 6, 7],
    "middle": [8, 9, 10, 11],
    "ring": [12, 13, 14, 15],
    "all": list(range(16)),
}


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    return delta


def deg_to_tick(deg):
    deg = deg % 360.0
    return int(round(deg / 360.0 * 4095))


def make_target_pose(data, ratio, selected_ids):
    open_pose = data["open_pose_deg"]
    close_pose = data["close_pose_deg"]

    targets = {}

    for dxl_id in selected_ids:
        key = str(dxl_id)

        open_deg = float(open_pose[key])
        close_deg = float(close_pose[key])

        delta = shortest_angle_delta(open_deg, close_deg)
        target_deg = (open_deg + delta * ratio) % 360.0
        target_tick = deg_to_tick(target_deg)

        targets[dxl_id] = {
            "deg": round(target_deg, 2),
            "tick": target_tick,
        }

    return targets


def parse_ids(ids_text, finger):
    if ids_text:
        return [int(x.strip()) for x in ids_text.split(",") if x.strip() != ""]

    return FINGER_IDS[finger]


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


class LeftHandRatioNode(Node):
    def __init__(self, args):
        super().__init__("left_hand_ratio_node")

        if not MAP_FILE.exists():
            raise FileNotFoundError("left_hand_motor_map.json 파일이 같은 폴더에 없음")

        with open(MAP_FILE, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self.port_name = args.port
        self.baudrate = args.baudrate if args.baudrate is not None else int(self.data["baudrate"])
        self.profile_velocity = args.profile_velocity
        self.selected_ids = parse_ids(args.ids, args.finger)

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.is_ready = False
        self.last_ratio = None

        self.get_logger().info("LEAP Left Hand Ratio Node starting")
        self.get_logger().info(f"port: {self.port_name}")
        self.get_logger().info(f"baudrate: {self.baudrate}")
        self.get_logger().info(f"ids: {self.selected_ids}")
        self.get_logger().info(f"profile velocity: {self.profile_velocity}")

        self.open_hardware()

        self.subscription = self.create_subscription(
            Float32,
            "/left_hand/close_ratio",
            self.ratio_callback,
            10,
        )

        self.get_logger().info("Subscribed: /left_hand/close_ratio")
        self.get_logger().info("Send Float32 ratio: 0.0=open, 1.0=fist")

    def open_hardware(self):
        if not self.port_handler.openPort():
            raise RuntimeError(f"포트 열기 실패: {self.port_name}")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"baudrate 설정 실패: {self.baudrate}")

        self.get_logger().info("port open ok")

        self.get_logger().info("torque off")
        if not set_torque(self.packet_handler, self.port_handler, self.selected_ids, False):
            raise RuntimeError("torque off 실패")

        self.get_logger().info("set profile velocity")
        if not set_profile_velocity(
            self.packet_handler,
            self.port_handler,
            self.selected_ids,
            self.profile_velocity,
        ):
            raise RuntimeError("profile velocity 설정 실패")

        self.get_logger().info("torque on")
        if not set_torque(self.packet_handler, self.port_handler, self.selected_ids, True):
            raise RuntimeError("torque on 실패")

        self.is_ready = True

        self.get_logger().info("move to open pose")
        self.send_ratio(0.0)

    def send_ratio(self, ratio):
        ratio = max(0.0, min(1.0, float(ratio)))

        targets = make_target_pose(self.data, ratio, self.selected_ids)

        ok = sync_write_goal_position(
            self.packet_handler,
            self.port_handler,
            targets,
        )

        if not ok:
            self.get_logger().error(f"failed to send ratio {ratio}")
            return False

        self.last_ratio = ratio
        self.get_logger().info(f"sent ratio = {ratio:.3f}")
        return True

    def ratio_callback(self, msg):
        ratio = float(msg.data)

        if ratio < 0.0 or ratio > 1.0:
            self.get_logger().warn(f"input ratio {ratio} out of range, clamped to 0.0~1.0")

        ratio = max(0.0, min(1.0, ratio))
        self.send_ratio(ratio)

    def cleanup(self):
        if not self.is_ready:
            return

        self.get_logger().info("cleanup: move to open pose")
        self.send_ratio(0.0)
        time.sleep(0.5)

        self.get_logger().info("cleanup: torque off")
        set_torque(self.packet_handler, self.port_handler, self.selected_ids, False)

        self.port_handler.closePort()
        self.get_logger().info("cleanup done")
        self.is_ready = False


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=None)
    parser.add_argument("--finger", choices=list(FINGER_IDS.keys()), default="all")
    parser.add_argument("--ids", default=None, help="예: 0 또는 0,1,2,3")
    parser.add_argument("--profile-velocity", type=int, default=30)

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = None

    try:
        node = LeftHandRatioNode(args)
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[CTRL+C] shutdown requested")

    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()