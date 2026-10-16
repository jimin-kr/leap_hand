import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


TOPIC_NAME = "/left_hand/finger_ratios"


class LeftHandFingerDemoPublisher(Node):
    def __init__(self):
        super().__init__("left_hand_finger_demo_publisher")

        self.publisher = self.create_publisher(
            Float32MultiArray,
            TOPIC_NAME,
            10,
        )

        self.get_logger().info("Left Hand Finger Demo Publisher started")
        self.get_logger().info(f"Publishing to {TOPIC_NAME}")
        self.get_logger().info("data order: [thumb, index, middle, ring]")

    def publish_finger_ratios(self, ratios, hold=2.0, label=""):
        msg = Float32MultiArray()
        msg.data = [float(v) for v in ratios]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"{label} -> "
            f"thumb={msg.data[0]:.2f}, "
            f"index={msg.data[1]:.2f}, "
            f"middle={msg.data[2]:.2f}, "
            f"ring={msg.data[3]:.2f}"
        )

        time.sleep(hold)


def main():
    rclpy.init()

    node = LeftHandFingerDemoPublisher()

    try:
        # ROS publisher/subscriber discovery 대기
        time.sleep(1.0)

        sequence = [
            ([0.0, 0.0, 0.0, 0.0], 2.0, "open"),

            ([0.0, 1.0, 0.0, 0.0], 2.0, "index only"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([0.0, 0.0, 1.0, 0.0], 2.0, "middle only"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([0.0, 0.0, 0.0, 1.0], 2.0, "ring only"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([1.0, 0.0, 0.0, 0.0], 2.0, "thumb only"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([0.0, 1.0, 1.0, 0.0], 2.0, "index + middle"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([1.0, 1.0, 0.0, 0.0], 2.0, "thumb + index"),
            ([0.0, 0.0, 0.0, 0.0], 1.5, "open"),

            ([1.0, 1.0, 1.0, 1.0], 3.0, "full fist"),
            ([0.5, 0.5, 0.5, 0.5], 2.0, "half close"),
            ([0.0, 0.0, 0.0, 0.0], 2.0, "final open"),
        ]

        for ratios, hold, label in sequence:
            node.publish_finger_ratios(ratios, hold, label)

        node.get_logger().info("finger demo finished")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()