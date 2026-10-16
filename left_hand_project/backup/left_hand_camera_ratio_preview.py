import argparse
import math
import time

import cv2
import mediapipe as mp


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def angle_deg(a, b, c):
    """
    angle ABC, in degrees
    a, b, c are (x, y)
    """
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
    cos_value = max(-1.0, min(1.0, cos_value))

    return math.degrees(math.acos(cos_value))


def angle_to_ratio(angle, open_angle=170.0, closed_angle=80.0):
    """
    finger open: angle close to 170 deg
    finger bent: angle close to 80 deg
    """
    ratio = (open_angle - angle) / (open_angle - closed_angle)
    return clamp(ratio)


def compute_finger_ratios(hand_landmarks, image_width, image_height):
    pts = []

    for lm in hand_landmarks.landmark:
        pts.append((lm.x * image_width, lm.y * image_height))

    # MediaPipe hand landmark indices
    # thumb: 2 MCP, 3 IP, 4 TIP
    # index: 5 MCP, 6 PIP, 7 DIP
    # middle: 9 MCP, 10 PIP, 11 DIP
    # ring: 13 MCP, 14 PIP, 15 DIP

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
        "angles": {
            "thumb": thumb_angle,
            "index": index_angle,
            "middle": middle_angle,
            "ring": ring_angle,
        },
    }


def smooth(prev, current, alpha=0.35):
    if prev is None:
        return current

    return {
        "thumb": prev["thumb"] * (1.0 - alpha) + current["thumb"] * alpha,
        "index": prev["index"] * (1.0 - alpha) + current["index"] * alpha,
        "middle": prev["middle"] * (1.0 - alpha) + current["middle"] * alpha,
        "ring": prev["ring"] * (1.0 - alpha) + current["ring"] * alpha,
        "angles": current["angles"],
    }


def draw_ratios(image, ratios):
    lines = [
        f"thumb : {ratios['thumb']:.2f}",
        f"index : {ratios['index']:.2f}",
        f"middle: {ratios['middle']:.2f}",
        f"ring  : {ratios['ring']:.2f}",
    ]

    y = 30
    for line in lines:
        cv2.putText(
            image,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        y += 35


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="camera index, usually 0",
    )

    parser.add_argument(
        "--mirror",
        action="store_true",
        help="mirror camera image horizontally",
    )

    parser.add_argument(
        "--max-ratio",
        type=float,
        default=1.0,
        help="limit output ratio for safety preview",
    )

    args = parser.parse_args()

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError(f"camera open failed: index {args.camera}")

    smoothed = None
    last_print_time = 0.0

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6,
    ) as hands:

        print("[INFO] camera ratio preview started")
        print("[INFO] press q to quit")
        print("[INFO] data order: [thumb, index, middle, ring]")

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

            if result.multi_hand_landmarks:
                hand_landmarks = result.multi_hand_landmarks[0]

                ratios = compute_finger_ratios(
                    hand_landmarks,
                    width,
                    height,
                )

                ratios["thumb"] = clamp(ratios["thumb"], 0.0, args.max_ratio)
                ratios["index"] = clamp(ratios["index"], 0.0, args.max_ratio)
                ratios["middle"] = clamp(ratios["middle"], 0.0, args.max_ratio)
                ratios["ring"] = clamp(ratios["ring"], 0.0, args.max_ratio)

                smoothed = smooth(smoothed, ratios)

                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                )

                draw_ratios(frame, smoothed)

                now = time.time()
                if now - last_print_time > 0.5:
                    last_print_time = now
                    print(
                        "[ratios] "
                        f"thumb={smoothed['thumb']:.2f}, "
                        f"index={smoothed['index']:.2f}, "
                        f"middle={smoothed['middle']:.2f}, "
                        f"ring={smoothed['ring']:.2f}"
                    )

            else:
                cv2.putText(
                    frame,
                    "No hand detected",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("left hand camera ratio preview", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()