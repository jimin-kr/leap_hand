import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


MAP_FILE = Path("left_hand_motor_map.json")


FINGER_IDS = {
    "thumb": [0, 1, 2, 3],
    "index": [4, 5, 6, 7],
    "middle": [8, 9, 10, 11],
    "ring": [12, 13, 14, 15],
}


JOINT_NAMES = {
    0: "left_thumb_0",
    1: "left_thumb_1",
    2: "left_thumb_2",
    3: "left_thumb_3",

    4: "left_index_0",
    5: "left_index_1",
    6: "left_index_2",
    7: "left_index_3",

    8: "left_middle_0",
    9: "left_middle_1",
    10: "left_middle_2",
    11: "left_middle_3",

    12: "left_ring_0",
    13: "left_ring_1",
    14: "left_ring_2",
    15: "left_ring_3",
}


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    return delta


class LeftHandJointStatePublisher(Node):
    def __init__(self):
        super().__init__("left_hand_joint_state_publisher")

        if not MAP_FILE.exists():
            raise FileNotFoundError("left_hand_motor_map.json 파일이 같은 폴더에 없음")

        with open(MAP_FILE, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self.open_pose = self.data["open_pose_deg"]
        self.close_pose = self.data["close_pose_deg"]

        self.current_ratio = 0.0

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.subscription = self.create_subscription(
            Float32,
            "/left_hand/close_ratio",
            self.ratio_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info("Left Hand Joint State Publisher started")
        self.get_logger().info("Subscribed: /left_hand/close_ratio")
        self.get_logger().info("Publishing: /joint_states at 20 Hz")

    def ratio_callback(self, msg):
        ratio = float(msg.data)

        if ratio < 0.0 or ratio > 1.0:
            self.get_logger().warn(f"input ratio {ratio} out of range, clamped")

        self.current_ratio = max(0.0, min(1.0, ratio))
        self.get_logger().info(f"joint state ratio updated = {self.current_ratio:.3f}")

    def get_joint_position_rad(self, dxl_id):
        key = str(dxl_id)

        open_deg = float(self.open_pose[key])
        close_deg = float(self.close_pose[key])

        delta_deg = shortest_angle_delta(open_deg, close_deg)

        # RViz용 상대각: open pose를 0 rad로 보고 close 방향 변화량만 표시
        joint_deg = delta_deg * self.current_ratio
        joint_rad = math.radians(joint_deg)

        return joint_rad

    def publish_joint_states(self):
        msg = JointState()

        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        names = []
        positions = []

        for dxl_id in range(16):
            names.append(JOINT_NAMES[dxl_id])
            positions.append(self.get_joint_position_rad(dxl_id))

        msg.name = names
        msg.position = positions

        self.publisher.publish(msg)


def main():
    rclpy.init()

    node = LeftHandJointStatePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n[CTRL+C] shutdown requested")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()