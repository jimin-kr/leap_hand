import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LeftHandRatioDemoPublisher(Node):
    def __init__(self):
        super().__init__("left_hand_ratio_demo_publisher")

        self.publisher = self.create_publisher(
            Float32,
            "/left_hand/close_ratio",
            10,
        )

        self.get_logger().info("Left Hand Ratio Demo Publisher started")
        self.get_logger().info("Publishing to /left_hand/close_ratio")

    def publish_ratio(self, ratio, hold=2.0):
        msg = Float32()
        msg.data = float(ratio)

        self.publisher.publish(msg)
        self.get_logger().info(f"published ratio = {ratio}")

        time.sleep(hold)


def main():
    rclpy.init()

    node = LeftHandRatioDemoPublisher()

    try:
        sequence = [
            (0.0, 2.0),   # open
            (0.2, 2.0),   # 20% close
            (0.5, 2.0),   # 50% close
            (1.0, 3.0),   # 100% fist
            (0.5, 2.0),   # half open
            (0.0, 2.0),   # open
        ]

        time.sleep(1.0)

        for ratio, hold in sequence:
            node.publish_ratio(ratio, hold)

        node.get_logger().info("demo finished")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()