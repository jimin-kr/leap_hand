import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState


# Shared calibration (open = 2048 for all motors)
MAP_FILE = (
    Path(__file__).resolve().parent.parent
    / "leap_teleop" / "config" / "left_hand_motor_map.json"
)

FINGER_IDS = {
    "thumb": [15, 14, 13, 12],
    "index": [3, 2, 1, 0],
    "middle": [7, 6, 5, 4],
    "ring": [11, 10, 9, 8],
}

FINGER_ORDER = ["thumb", "index", "middle", "ring"]

# Matches URDF joint names in assets/leap_hand/leap_hand_left.urdf ("0".."15")
JOINT_NAMES = {i: str(i) for i in range(16)}


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0

    return delta


def clamp_ratio(value):
    value = float(value)

    if value < 0.0:
        return 0.0
    if value > 1.0:
        return 1.0

    return value


class LeftHandFingerJointStatePublisher(Node):
    def __init__(self):
        super().__init__("left_hand_finger_joint_state_publisher")

        if not MAP_FILE.exists():
            raise FileNotFoundError(f"motor map 파일 없음: {MAP_FILE}")

        with open(MAP_FILE, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self.open_pose = self.data["open_pose_deg"]
        self.close_pose = self.data["close_pose_deg"]
        self.finger_ids = self.data.get("ids", FINGER_IDS)
        self.urdf_joint = self.data.get("urdf_joint", JOINT_NAMES)
        self.urdf_sign = self.data.get("urdf_sign", {})

        self.finger_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.subscription = self.create_subscription(
            Float32MultiArray,
            "/left_hand/finger_ratios",
            self.finger_ratio_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info("Left Hand Finger Joint State Publisher started")
        self.get_logger().info("Subscribed: /left_hand/finger_ratios")
        self.get_logger().info("Publishing: /joint_states at 20 Hz")
        self.get_logger().info("data order: [thumb, index, middle, ring]")

    def finger_ratio_callback(self, msg):
        data = list(msg.data)

        if len(data) != 4:
            self.get_logger().error(
                f"finger_ratios length must be 4, got {len(data)}"
            )
            self.get_logger().error("expected: [thumb, index, middle, ring]")
            return

        self.finger_ratios = {
            "thumb": clamp_ratio(data[0]),
            "index": clamp_ratio(data[1]),
            "middle": clamp_ratio(data[2]),
            "ring": clamp_ratio(data[3]),
        }

        self.get_logger().info(
            "joint state finger ratios updated: "
            f"thumb={self.finger_ratios['thumb']:.2f}, "
            f"index={self.finger_ratios['index']:.2f}, "
            f"middle={self.finger_ratios['middle']:.2f}, "
            f"ring={self.finger_ratios['ring']:.2f}"
        )

    def get_ratio_for_id(self, dxl_id):
        for finger_name, ids in self.finger_ids.items():
            if dxl_id in ids:
                return self.finger_ratios[finger_name]

        return 0.0

    def get_joint_position_rad(self, dxl_id):
        key = str(dxl_id)

        open_deg = float(self.open_pose[key])
        close_deg = float(self.close_pose[key])

        delta_deg = shortest_angle_delta(open_deg, close_deg)
        ratio = self.get_ratio_for_id(dxl_id)
        sign = float(self.urdf_sign.get(key, 1))

        return math.radians(sign * delta_deg * ratio)

    def publish_joint_states(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        names = []
        positions = []

        for dxl_id in range(16):
            names.append(self.urdf_joint.get(str(dxl_id), str(dxl_id)))
            positions.append(self.get_joint_position_rad(dxl_id))

        msg.name = names
        msg.position = positions

        self.publisher.publish(msg)


def main():
    rclpy.init()

    node = LeftHandFingerJointStatePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[CTRL+C] shutdown requested")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()