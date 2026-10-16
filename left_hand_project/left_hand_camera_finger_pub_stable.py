import argparse
import math
import time

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


TOPIC_NAME = "/left_hand/finger_ratios"
FINGER_NAMES = ["thumb", "index", "middle", "ring"]


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def make_zero_ratios():
    return {
        "thumb": 0.0,
        "index": 0.0,
        "middle": 0.0,
        "ring": 0.0,
    }


def angle_deg(a, b, c):
    bax = a[0] - b[0]
    bay = a[1] - b[1]
    bcx = c[0] - b[0]
    bcy = c[1] - b[1]

    dot = bax * bcx + bay * bcy
    mag1 = math.sqrt(bax * bax + bay * bay)
    mag2 = math.sqrt(bcx * bcx + bcy * bcy)

    if mag1 < 1e-6 or mag2 < 1e-6:
        return 180.0

    cos_value = dot / (mag1 * mag2)
    cos_value = clamp(cos_value, -1.0, 1.0)

    return math.degrees(math.acos(cos_value))


def angle_to_ratio(angle, open_angle=170.0, closed_angle=80.0):
    ratio = (open_angle - angle) / (open_angle - closed_angle)
    return clamp(ratio)


def compute_finger_ratios(hand_landmarks, image_width, image_height):
    pts = []

    for lm in hand_landmarks.landmark:
        pts.append((lm.x * image_width, lm.y * image_height))

    thumb_angle = angle_deg(pts[2], pts[3], pts[4])
    index_angle = angle_deg(pts[5], pts[6], pts[7])
    middle_angle = angle_deg(pts[9], pts[10], pts[11])
    ring_angle = angle_deg(pts[13], pts[14], pts[15])

    thumb = angle_to_ratio(thumb_angle, open_angle=165.0, closed_angle=80.0)
    index = angle_to_ratio(index_angle, open_angle=170.0, closed_angle=80.0)
    middle = angle_to_ratio(middle_angle, open_angle=170.0, closed_angle=80.0)
    ring = angle_to_ratio(ring_angle, open_angle=170.0, closed_angle=80.0)

    return {
        "thumb": thumb,
        "index": index,
        "middle": middle,
        "ring": ring,
    }


def exponential_smooth(prev, current, alpha):
    if prev is None:
        return current.copy()

    output = {}

    for name in FINGER_NAMES:
        output[name] = prev[name] * (1.0 - alpha) + current[name] * alpha

    return output


def apply_deadzone(ratios, deadzone):
    output = {}

    for name in FINGER_NAMES:
        value = ratios[name]

        if value < deadzone:
            value = 0.0

        output[name] = value

    return output


def apply_max_ratio(ratios, max_ratio):
    output = {}

    for name in FINGER_NAMES:
        output[name] = clamp(ratios[name], 0.0, max_ratio)

    return output


def step_limit(prev_output, target, max_step):
    output = {}

    for name in FINGER_NAMES:
        prev = prev_output[name]
        goal = target[name]
        diff = goal - prev

        if diff > max_step:
            diff = max_step
        elif diff < -max_step:
            diff = -max_step

        output[name] = clamp(prev + diff)

    return output


def draw_text(image, target, output, publish_enabled, hand_detected):
    status = "PUBLISH ON" if publish_enabled else "PUBLISH OFF"
    detect_status = "HAND OK" if hand_detected else "NO HAND"

    color = (0, 255, 0) if publish_enabled else (0, 0, 255)

    lines = [
        f"{status} / {detect_status}",
        "press s: toggle publish",
        "press q: quit",
        "",
        "TARGET",
        f"T thumb : {target['thumb']:.2f}",
        f"T index : {target['index']:.2f}",
        f"T middle: {target['middle']:.2f}",
        f"T ring  : {target['ring']:.2f}",
        "",
        "OUTPUT",
        f"O thumb : {output['thumb']:.2f}",
        f"O index : {output['index']:.2f}",
        f"O middle: {output['middle']:.2f}",
        f"O ring  : {output['ring']:.2f}",
    ]

    y = 30

    for line in lines:
        cv2.putText(
            image,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        y += 27


class CameraFingerStablePublisher(Node):
    def __init__(self):
        super().__init__("left_hand_camera_finger_stable_publisher")

        self.publisher = self.create_publisher(
            Float32MultiArray,
            TOPIC_NAME,
            10,
        )

        self.get_logger().info("Left Hand Camera Finger Stable Publisher started")
        self.get_logger().info(f"Publishing to {TOPIC_NAME}")
        self.get_logger().info("data order: [thumb, index, middle, ring]")

    def publish_ratios(self, ratios):
        msg = Float32MultiArray()
        msg.data = [
            float(ratios["thumb"]),
            float(ratios["index"]),
            float(ratios["middle"]),
            float(ratios["ring"]),
        ]

        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--mirror", action="store_true")

    parser.add_argument(
        "--max-ratio",
        type=float,
        default=0.7,
        help="maximum output ratio",
    )

    parser.add_argument(
        "--publish-rate",
        type=float,
        default=15.0,
        help="ROS publish rate",
    )

    parser.add_argument(
        "--alpha",
        type=float,
        default=0.25,
        help="smoothing factor. lower = smoother",
    )

    parser.add_argument(
        "--max-step",
        type=float,
        default=0.05,
        help="maximum ratio change per publish",
    )

    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.05,
        help="small ratios below this become zero",
    )

    parser.add_argument(
        "--lost-timeout",
        type=float,
        default=0.4,
        help="after no hand for this time, target becomes open",
    )

    args = parser.parse_args()

    rclpy.init()
    node = CameraFingerStablePublisher()

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError(f"camera open failed: index {args.camera}")

    raw_smoothed = None
    target_ratios = make_zero_ratios()
    output_ratios = make_zero_ratios()

    publish_enabled = False

    last_hand_time = 0.0
    last_publish_time = 0.0
    last_print_time = 0.0

    node.publish_ratios(make_zero_ratios())

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6,
    ) as hands:

        print("[INFO] stable camera finger publisher started")
        print("[INFO] press s to toggle publish")
        print("[INFO] press q to quit")
        print("[INFO] publish OFF at start")
        print(f"[INFO] max_ratio={args.max_ratio}")
        print(f"[INFO] alpha={args.alpha}")
        print(f"[INFO] max_step={args.max_step}")
        print(f"[INFO] deadzone={args.deadzone}")
        print(f"[INFO] lost_timeout={args.lost_timeout}")

        try:
            while True:
                ok, frame = cap.read()

                if not ok:
                    print("[ERROR] failed to read camera frame")
                    break

                if args.mirror:
                    frame = cv2.flip(frame, 1)

                height, width = frame.shape[:2]

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                result = hands.process(rgb)

                now = time.time()
                hand_detected = False

                if result.multi_hand_landmarks:
                    hand_detected = True
                    last_hand_time = now

                    hand_landmarks = result.multi_hand_landmarks[0]

                    raw_ratios = compute_finger_ratios(
                        hand_landmarks,
                        width,
                        height,
                    )

                    raw_ratios = apply_max_ratio(raw_ratios, args.max_ratio)
                    raw_ratios = apply_deadzone(raw_ratios, args.deadzone)

                    raw_smoothed = exponential_smooth(
                        raw_smoothed,
                        raw_ratios,
                        args.alpha,
                    )

                    target_ratios = raw_smoothed.copy()

                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                    )

                else:
                    if now - last_hand_time > args.lost_timeout:
                        target_ratios = make_zero_ratios()

                output_ratios = step_limit(
                    output_ratios,
                    target_ratios,
                    args.max_step,
                )

                if publish_enabled and now - last_publish_time >= 1.0 / args.publish_rate:
                    last_publish_time = now
                    node.publish_ratios(output_ratios)

                draw_text(
                    frame,
                    target_ratios,
                    output_ratios,
                    publish_enabled,
                    hand_detected,
                )

                if now - last_print_time > 0.5:
                    last_print_time = now
                    print(
                        "[stable output] "
                        f"thumb={output_ratios['thumb']:.2f}, "
                        f"index={output_ratios['index']:.2f}, "
                        f"middle={output_ratios['middle']:.2f}, "
                        f"ring={output_ratios['ring']:.2f}, "
                        f"publish={publish_enabled}, "
                        f"hand={hand_detected}"
                    )

                cv2.imshow("left hand stable camera finger publisher", frame)

                key = cv2.waitKey(1) & 0xFF

                if key == ord("s"):
                    publish_enabled = not publish_enabled

                    if not publish_enabled:
                        output_ratios = make_zero_ratios()
                        target_ratios = make_zero_ratios()
                        raw_smoothed = None
                        node.publish_ratios(make_zero_ratios())

                    print(f"[INFO] publish_enabled = {publish_enabled}")

                if key == ord("q"):
                    break

        finally:
            node.publish_ratios(make_zero_ratios())
            cap.release()
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()