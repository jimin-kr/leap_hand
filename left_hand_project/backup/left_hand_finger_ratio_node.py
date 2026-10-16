import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

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
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116

LEN_GOAL_POSITION = 4

TORQUE_OFF = 0
TORQUE_ON = 1

MAP_FILE = Path("left_hand_motor_map.json")

FINGER_IDS = {
    "thumb": [0, 1, 2, 3],
    "index": [4, 5, 6, 7],
    "middle": [8, 9, 10, 11],
    "ring": [12, 13, 14, 15],
}

FINGER_ORDER = ["thumb", "index", "middle", "ring"]


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0

    return delta


def deg_to_tick(deg):
    deg = deg % 360.0
    tick = int(round(deg / 360.0 * 4096.0))

    if tick < 0:
        tick = 0
    if tick > 4095:
        tick = 4095

    return tick


def clamp_ratio(value):
    value = float(value)

    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0

    return value


def make_target_pose(open_pose, close_pose, finger_ratios):
    """
    finger_ratios example:
    {
        "thumb": 0.0,
        "index": 1.0,
        "middle": 0.5,
        "ring": 0.0,
    }
    """

    target_pose = {}

    for finger_name, ids in FINGER_IDS.items():
        ratio = finger_ratios[finger_name]

        for dxl_id in ids:
            key = str(dxl_id)

            open_deg = float(open_pose[key])
            close_deg = float(close_pose[key])

            delta_deg = shortest_angle_delta(open_deg, close_deg)
            target_deg = open_deg + delta_deg * ratio

            target_pose[dxl_id] = target_deg

    return target_pose


class LeftHandFingerRatioNode(Node):
    def __init__(self, args):
        super().__init__("left_hand_finger_ratio_node")

        self.port_name = args.port
        self.baudrate = args.baudrate
        self.profile_velocity = args.profile_velocity

        if not MAP_FILE.exists():
            raise FileNotFoundError("left_hand_motor_map.json 파일이 같은 폴더에 없음")

        with open(MAP_FILE, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self.open_pose = self.data["open_pose_deg"]
        self.close_pose = self.data["close_pose_deg"]

        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.current_finger_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        self.get_logger().info("LEAP Left Hand Finger Ratio Node starting")
        self.get_logger().info(f"port: {self.port_name}")
        self.get_logger().info(f"baudrate: {self.baudrate}")
        self.get_logger().info(f"profile velocity: {self.profile_velocity}")
        self.get_logger().info("topic: /left_hand/finger_ratios")
        self.get_logger().info("data order: [thumb, index, middle, ring]")

        self.open_hardware()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/left_hand/finger_ratios",
            self.finger_ratio_callback,
            10,
        )

        self.get_logger().info("Subscribed: /left_hand/finger_ratios")
        self.get_logger().info("Example:")
        self.get_logger().info(
            'ros2 topic pub --once /left_hand/finger_ratios std_msgs/msg/Float32MultiArray "{data: [0.0, 1.0, 0.0, 0.0]}"'
        )

    def check_comm_result(self, dxl_id, comm_result, dxl_error, action):
        if comm_result != COMM_SUCCESS:
            print(
                f"[ERROR] ID {dxl_id} {action}: "
                f"{self.packet_handler.getTxRxResult(comm_result)}"
            )
            return False

        if dxl_error != 0:
            print(
                f"[ERROR] ID {dxl_id} {action}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )
            return False

        return True

    def set_torque(self, dxl_id, enable):
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            enable,
        )

        return self.check_comm_result(
            dxl_id,
            comm_result,
            dxl_error,
            "torque on" if enable else "torque off",
        )

    def set_profile_velocity(self, dxl_id, velocity):
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_PROFILE_VELOCITY,
            int(velocity),
        )

        return self.check_comm_result(
            dxl_id,
            comm_result,
            dxl_error,
            "set profile velocity",
        )

    def sync_write_goal_position(self, target_pose):
        group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        for dxl_id, target_deg in target_pose.items():
            goal_tick = deg_to_tick(target_deg)

            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(goal_tick)),
                DXL_HIBYTE(DXL_LOWORD(goal_tick)),
                DXL_LOBYTE(DXL_HIWORD(goal_tick)),
                DXL_HIBYTE(DXL_HIWORD(goal_tick)),
            ]

            add_ok = group_sync_write.addParam(dxl_id, param_goal_position)

            if not add_ok:
                print(f"[ERROR] ID {dxl_id} groupSyncWrite addParam failed")
                group_sync_write.clearParam()
                return False

        comm_result = group_sync_write.txPacket()

        if comm_result != COMM_SUCCESS:
            print(
                "[ERROR] groupSyncWrite txPacket:",
                self.packet_handler.getTxRxResult(comm_result),
            )
            group_sync_write.clearParam()
            return False

        group_sync_write.clearParam()
        return True

    def open_hardware(self):
        if not self.port_handler.openPort():
            raise RuntimeError(f"port open 실패: {self.port_name}")

        self.get_logger().info("port open ok")

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"baudrate 설정 실패: {self.baudrate}")

        self.get_logger().info("baudrate set ok")

        all_ids = []
        for ids in FINGER_IDS.values():
            all_ids.extend(ids)

        self.get_logger().info("torque off")

        for dxl_id in all_ids:
            if not self.set_torque(dxl_id, TORQUE_OFF):
                raise RuntimeError(f"ID {dxl_id} torque off 실패")

        self.get_logger().info("set profile velocity")

        for dxl_id in all_ids:
            if not self.set_profile_velocity(dxl_id, self.profile_velocity):
                raise RuntimeError(f"ID {dxl_id} profile velocity 설정 실패")

        self.get_logger().info("torque on")

        for dxl_id in all_ids:
            if not self.set_torque(dxl_id, TORQUE_ON):
                raise RuntimeError(f"ID {dxl_id} torque on 실패")

        self.get_logger().info("move to open pose")

        open_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        target_pose = make_target_pose(
            self.open_pose,
            self.close_pose,
            open_ratios,
        )

        if not self.sync_write_goal_position(target_pose):
            raise RuntimeError("open pose 이동 실패")

        time.sleep(1.0)

    def finger_ratio_callback(self, msg):
        data = list(msg.data)

        if len(data) != 4:
            self.get_logger().error(
                f"finger_ratios length must be 4, got {len(data)}"
            )
            self.get_logger().error("expected: [thumb, index, middle, ring]")
            return

        finger_ratios = {
            "thumb": clamp_ratio(data[0]),
            "index": clamp_ratio(data[1]),
            "middle": clamp_ratio(data[2]),
            "ring": clamp_ratio(data[3]),
        }

        self.current_finger_ratios = finger_ratios

        self.get_logger().info(
            "finger ratios: "
            f"thumb={finger_ratios['thumb']:.2f}, "
            f"index={finger_ratios['index']:.2f}, "
            f"middle={finger_ratios['middle']:.2f}, "
            f"ring={finger_ratios['ring']:.2f}"
        )

        target_pose = make_target_pose(
            self.open_pose,
            self.close_pose,
            finger_ratios,
        )

        if not self.sync_write_goal_position(target_pose):
            self.get_logger().error("goal position send failed")

    def cleanup(self):
        self.get_logger().info("cleanup: move to open pose")

        open_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        try:
            target_pose = make_target_pose(
                self.open_pose,
                self.close_pose,
                open_ratios,
            )
            self.sync_write_goal_position(target_pose)
            time.sleep(1.0)

            self.get_logger().info("cleanup: torque off")

            all_ids = []
            for ids in FINGER_IDS.values():
                all_ids.extend(ids)

            for dxl_id in all_ids:
                self.set_torque(dxl_id, TORQUE_OFF)

        finally:
            self.port_handler.closePort()
            self.get_logger().info("port closed")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Dynamixel port, example: /dev/ttyUSB0",
    )

    parser.add_argument(
        "--baudrate",
        type=int,
        default=57600,
        help="Dynamixel baudrate",
    )

    parser.add_argument(
        "--profile-velocity",
        type=int,
        default=20,
        help="Dynamixel profile velocity",
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = None

    try:
        node = LeftHandFingerRatioNode(args)
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