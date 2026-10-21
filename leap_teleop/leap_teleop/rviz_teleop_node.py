#!/usr/bin/env python3
"""
Colleague Exact MediaPipe 2D Ratio -> Motor Interpolation Teleoperation Node for LEAP Hand
(With Intel RealSense Camera Support)
========================================================================================
Replicates the exact ratio calculation and motor linear mapping from colleague's project:
  - left_hand_camera_finger_pub_stable.py
  - left_hand_finger_joint_state_pub.py
  - left_hand_motor_map.json

Pipeline:
  1. Image acquisition from RealSense (or OpenCV webcam fallback)
  2. 2D MediaPipe landmark detection from camera frame
  3. 3-point joint angle calculation for thumb, index, middle, ring
  4. Angle-to-ratio mapping (0.0 open ~ 1.0 closed)
  5. Exponential smoothing, deadzone, max_ratio, and max_step limiting
  6. Shortest angle delta calculation between open_pose_deg and close_pose_deg
  7. Publishing 16 joint positions in radians to `/joint_states` for RViz URDF simulation
     (motor ID -> LEAP URDF joint name via `urdf_joint` / `urdf_sign` in the motor map)
  8. Publishing [thumb, index, middle, ring] ratios to `/left_hand/finger_ratios`
     (consumed by left_hand_finger_ratio_node_safe.py to drive the real hand once ARMED)

Calibration: config/left_hand_motor_map.json (open = encoder 2048 for every motor).
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

TOPIC_NAME = "/joint_states"
RATIO_TOPIC = "/right_hand/finger_ratios"
FALLBACK_RATIO_TOPIC = "/left_hand/finger_ratios"
FINGER_NAMES = ["index", "middle", "ring", "thumb"]
RATIO_ORDER = ["thumb", "index", "middle", "ring"]  # order expected by the real-hand node
MOTOR_MAP_NAME = "right_hand_motor_map.json"


def default_motor_map_path():
    """Installed package share first, then the source tree (for running the .py directly)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("leap_teleop")) / "config" / MOTOR_MAP_NAME
        if p.exists():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "config" / MOTOR_MAP_NAME


def load_motor_map(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    for key in ("ids", "open_pose_deg", "close_pose_deg", "urdf_joint"):
        if key not in data:
            raise KeyError(f"motor map missing '{key}': {path}")
    return data


class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30, serial=None, preferred_model="d405"):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        ctx = rs.context()
        devices = ctx.query_devices()
        selected_desc = None

        if serial:
            self.config.enable_device(str(serial))
            selected_desc = f"Serial {serial}"
        elif preferred_model and preferred_model.lower() != "any":
            for dev in devices:
                dev_name = dev.get_info(rs.camera_info.name)
                dev_serial = dev.get_info(rs.camera_info.serial_number)
                if preferred_model.lower() in dev_name.lower():
                    self.config.enable_device(dev_serial)
                    selected_desc = f"{dev_name} (Serial: {dev_serial})"
                    break

        if not selected_desc and len(devices) > 0:
            dev = devices[0]
            dev_name = dev.get_info(rs.camera_info.name)
            dev_serial = dev.get_info(rs.camera_info.serial_number)
            self.config.enable_device(dev_serial)
            selected_desc = f"{dev_name} (Serial: {dev_serial})"

        self.selected_device = selected_desc or "Default Device"
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)


    def read(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return False, None
            color_image = np.asanyarray(color_frame.get_data())
            return True, color_image
        except Exception:
            return False, None

    def release(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


class OpenCVCamera:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)

    def isOpened(self):
        return self.cap.isOpened()

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def make_zero_ratios():
    return {
        "index": 0.0,
        "middle": 0.0,
        "ring": 0.0,
        "thumb": 0.0,
        "spread_index": 0.0,
        "spread_middle": 0.0,
        "spread_ring": 0.0,
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
    pts_3d = []
    for lm in hand_landmarks.landmark:
        pts.append((lm.x * image_width, lm.y * image_height))
        pts_3d.append(np.array([lm.x, lm.y, lm.z]))

    index_angle = angle_deg(pts[5], pts[6], pts[7])
    middle_angle = angle_deg(pts[9], pts[10], pts[11])
    ring_angle = angle_deg(pts[13], pts[14], pts[15])

    index = angle_to_ratio(index_angle, open_angle=170.0, closed_angle=80.0)
    middle = angle_to_ratio(middle_angle, open_angle=170.0, closed_angle=80.0)
    ring = angle_to_ratio(ring_angle, open_angle=170.0, closed_angle=80.0)

    # -------------------------------------------------------------
    # Dex-Retargeting 3D Vector & Distance Opposition for Thumb
    # -------------------------------------------------------------
    p4 = pts_3d[4]    # Thumb Tip
    p2 = pts_3d[2]    # Thumb MCP
    p5 = pts_3d[5]    # Index MCP
    p17 = pts_3d[17]  # Pinky MCP

    # Normalize by palm width (Index MCP <-> Pinky MCP)
    palm_width = np.linalg.norm(p5 - p17)
    if palm_width < 1e-6:
        palm_width = 1.0

    # 1. 3D Opposition Distance: Thumb Tip to Pinky MCP normalized by palm width
    thumb_dist = np.linalg.norm(p4 - p17) / palm_width
    thumb_dist_ratio = clamp((1.45 - thumb_dist) / (1.45 - 0.50))

    # 2. 3D Thumb Joint Bending Angle
    v23 = pts_3d[3] - pts_3d[2]
    v34 = pts_3d[4] - pts_3d[3]
    mag23 = np.linalg.norm(v23)
    mag34 = np.linalg.norm(v34)
    if mag23 > 1e-6 and mag34 > 1e-6:
        cos_thumb = np.dot(v23, v34) / (mag23 * mag34)
        thumb_3d_angle = math.degrees(math.acos(clamp(cos_thumb, -1.0, 1.0)))
    else:
        thumb_3d_angle = 0.0

    thumb_angle_ratio = clamp(thumb_3d_angle / 70.0)

    # Combined 3D Dex-Retargeting Thumb Ratio
    thumb = clamp(0.5 * thumb_dist_ratio + 0.5 * thumb_angle_ratio)

    # -------------------------------------------------------------
    # 3D Palm Coordinates & Finger Spread (Abduction/Adduction)
    # -------------------------------------------------------------
    p0 = pts_3d[0]
    p6 = pts_3d[6]    # Index PIP
    p9 = pts_3d[9]    # Middle MCP
    p10 = pts_3d[10]  # Middle PIP
    p13 = pts_3d[13]  # Ring MCP
    p14 = pts_3d[14]  # Ring PIP

    v_knuckle = p5 - p17
    norm_knuckle = np.linalg.norm(v_knuckle)
    u_knuckle = v_knuckle / norm_knuckle if norm_knuckle > 1e-6 else np.array([1.0, 0.0, 0.0])

    v_long = p9 - p0
    norm_long = np.linalg.norm(v_long)
    u_long = v_long / norm_long if norm_long > 1e-6 else np.array([0.0, 1.0, 0.0])

    u_z = np.cross(u_knuckle, u_long)
    norm_z = np.linalg.norm(u_z)
    u_z = u_z / norm_z if norm_z > 1e-6 else np.array([0.0, 0.0, 1.0])

    u_y = u_long
    u_x = np.cross(u_y, u_z)
    norm_x = np.linalg.norm(u_x)
    u_x = u_x / norm_x if norm_x > 1e-6 else np.array([1.0, 0.0, 0.0])

    v_idx = p6 - p5
    v_mid = p10 - p9
    v_ring = p14 - p13

    theta_idx = math.degrees(math.atan2(np.dot(v_idx, u_x), np.dot(v_idx, u_y)))
    theta_mid = math.degrees(math.atan2(np.dot(v_mid, u_x), np.dot(v_mid, u_y)))
    theta_ring = math.degrees(math.atan2(np.dot(v_ring, u_x), np.dot(v_ring, u_y)))

    d_idx = theta_idx - theta_mid
    d_ring = -(theta_ring - theta_mid)

    if d_idx > 0:
        spread_index = min(1.0, d_idx / 16.0)
    else:
        spread_index = max(-1.0, d_idx / 8.0)

    if d_ring > 0:
        spread_ring = min(1.0, d_ring / 16.0)
    else:
        spread_ring = max(-1.0, d_ring / 8.0)

    spread_middle = max(-1.0, min(1.0, theta_mid / 10.0))

    return {
        "index": index,
        "middle": middle,
        "ring": ring,
        "thumb": thumb,
        "spread_index": spread_index,
        "spread_middle": spread_middle,
        "spread_ring": spread_ring,
    }


def exponential_smooth(prev, current, alpha):
    if prev is None:
        return current.copy()

    output = {}
    for k in current:
        p = prev.get(k, 0.0)
        output[k] = p * (1.0 - alpha) + current[k] * alpha

    return output


def apply_deadzone(ratios, deadzone):
    output = {}
    for k, value in ratios.items():
        if k.startswith("spread"):
            if abs(value) < deadzone:
                value = 0.0
        else:
            if value < deadzone:
                value = 0.0
        output[k] = value

    return output


def apply_max_ratio(ratios, max_ratio):
    output = {}
    for k, value in ratios.items():
        if k.startswith("spread"):
            output[k] = max(-1.0, min(1.0, float(value)))
        else:
            output[k] = clamp(value, 0.0, max_ratio)

    return output


def step_limit(prev_output, target, max_step):
    output = {}
    for k in target:
        prev = prev_output.get(k, 0.0)
        goal = target[k]
        diff = goal - prev

        if diff > max_step:
            diff = max_step
        elif diff < -max_step:
            diff = -max_step

        if k.startswith("spread"):
            output[k] = max(-1.0, min(1.0, prev + diff))
        else:
            output[k] = clamp(prev + diff)

    return output


def shortest_angle_delta(open_deg, close_deg):
    delta = close_deg - open_deg

    if delta > 180.0:
        delta -= 360.0
    elif delta < -180.0:
        delta += 360.0

    return delta


def get_joint_position_rad(motor_map, dxl_id, ratios):
    """Motor travel (open -> close, scaled by finger ratio) as a URDF joint angle."""
    key = str(dxl_id)
    if dxl_id in (1, 5, 9):
        spread_map = {1: "spread_index", 5: "spread_middle", 9: "spread_ring"}
        s_val = float(ratios.get(spread_map[dxl_id], 0.0))
        sign = float(motor_map.get("spread_sign", {}).get(key, 1.0)) * float(motor_map.get("urdf_sign", {}).get(key, 1))
        # 10 degrees = 0.1745 rad
        return math.radians(sign * s_val * 10.0)

    open_deg = float(motor_map["open_pose_deg"][key])
    close_deg = float(motor_map["close_pose_deg"][key])

    delta_deg = shortest_angle_delta(open_deg, close_deg)

    ratio = 0.0
    for finger_name, ids in motor_map["ids"].items():
        if dxl_id in ids:
            ratio = ratios.get(finger_name, 0.0)
            break

    sign = float(motor_map.get("urdf_sign", {}).get(key, 1))
    return math.radians(sign * delta_deg * ratio)


def draw_text(image, target, output, publish_enabled, hand_detected, source_name):
    status = "PUBLISH ON" if publish_enabled else "PUBLISH OFF"
    detect_status = "HAND OK" if hand_detected else "NO HAND"

    color = (0, 255, 0) if publish_enabled else (0, 0, 255)

    lines = [
        f"[{source_name.upper()}] {status} / {detect_status}",
        "press s: toggle publish",
        "press q: quit",
        "",
        "TARGET RATIOS:",
        f"  index : {target.get('index', 0.0):.2f}",
        f"  middle: {target.get('middle', 0.0):.2f}",
        f"  ring  : {target.get('ring', 0.0):.2f}",
        f"  thumb : {target.get('thumb', 0.0):.2f}",
        f"  spread: I={target.get('spread_index', 0.0):+.2f} M={target.get('spread_middle', 0.0):+.2f} R={target.get('spread_ring', 0.0):+.2f}",
        "",
        "OUTPUT RATIOS:",
        f"  index : {output.get('index', 0.0):.2f}",
        f"  middle: {output.get('middle', 0.0):.2f}",
        f"  ring  : {output.get('ring', 0.0):.2f}",
        f"  thumb : {output.get('thumb', 0.0):.2f}",
        f"  spread: I={output.get('spread_index', 0.0):+.2f} M={output.get('spread_middle', 0.0):+.2f} R={output.get('spread_ring', 0.0):+.2f}",
    ]

    y = 30
    for line in lines:
        cv2.putText(
            image,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )
        y += 24


class ColleagueRatioJointStatePublisher(Node):
    def __init__(self, motor_map):
        super().__init__("leap_joint_state_publisher")
        self.motor_map = motor_map
        self.publisher = self.create_publisher(JointState, TOPIC_NAME, 10)
        self.ratio_publisher = self.create_publisher(Float32MultiArray, RATIO_TOPIC, 10)
        self.spread_publisher = self.create_publisher(Float32MultiArray, "/right_hand/finger_spread", 10)
        self.fallback_spread_publisher = self.create_publisher(Float32MultiArray, "/left_hand/finger_spread", 10)
        self.get_logger().info("Colleague Ratio LEAP Teleop Node started.")
        self.get_logger().info(f"Publishing 16 joint states to {TOPIC_NAME}")
        self.get_logger().info(f"Publishing finger ratios to {RATIO_TOPIC}")
        self.get_logger().info("Publishing finger spread to /right_hand/finger_spread")

    def publish_ratios(self, ratios):
        # 1. Publish URDF joint states for RViz simulation
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = []
        positions = []
        for dxl_id in range(16):
            urdf_joint = self.motor_map.get("urdf_joint", {}).get(str(dxl_id), str(dxl_id))
            names.append(urdf_joint)
            positions.append(get_joint_position_rad(self.motor_map, dxl_id, ratios))
        msg.name = names
        msg.position = positions
        self.publisher.publish(msg)

        # 2. Publish finger ratios for real-hand safe node (order: thumb, index, middle, ring, spread_idx, spread_mid, spread_rng)
        ratio_msg = Float32MultiArray()
        ratio_msg.data = [
            float(ratios.get("thumb", 0.0)),
            float(ratios.get("index", 0.0)),
            float(ratios.get("middle", 0.0)),
            float(ratios.get("ring", 0.0)),
            float(ratios.get("spread_index", 0.0)),
            float(ratios.get("spread_middle", 0.0)),
            float(ratios.get("spread_ring", 0.0)),
        ]
        self.ratio_publisher.publish(ratio_msg)
        self.fallback_ratio_publisher.publish(ratio_msg)

        # 3. Dedicated spread publisher
        spread_msg = Float32MultiArray()
        spread_msg.data = [
            float(ratios.get("spread_index", 0.0)),
            float(ratios.get("spread_middle", 0.0)),
            float(ratios.get("spread_ring", 0.0)),
        ]
        self.spread_publisher.publish(spread_msg)
        self.fallback_spread_publisher.publish(spread_msg)


def rotate_frame(frame, rot):
    if rot == "cw":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif rot == "ccw":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rot == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def main():
    parser = argparse.ArgumentParser(description="Colleague Ratio LEAP Teleop Node (RealSense / Webcam)")
    parser.add_argument("--source", type=str, default="realsense", choices=["realsense", "webcam"], help="Camera source")
    parser.add_argument("--camera", type=int, default=0, help="Webcam device index if source is webcam")
    parser.add_argument("--camera-model", type=str, default="d405", help="Preferred RealSense camera model (default: d405)")
    parser.add_argument("--camera-serial", type=str, default=None, help="Specific RealSense camera serial number")
    parser.add_argument("--rot", type=str, default="none", choices=["none", "cw", "ccw", "180"], help="Rotation of camera frame")
    parser.add_argument("--mirror", action="store_true", help="Mirror camera preview")
    parser.add_argument("--max-ratio", type=float, default=1.0, help="Maximum finger ratio limit")
    parser.add_argument("--publish-rate", type=float, default=30.0, help="ROS publish rate (Hz)")
    parser.add_argument("--alpha", type=float, default=0.25, help="Smoothing factor")
    parser.add_argument("--max-step", type=float, default=0.05, help="Maximum step change per publish")
    parser.add_argument("--deadzone", type=float, default=0.05, help="Deadzone threshold")
    parser.add_argument("--lost-timeout", type=float, default=0.4, help="Hand lost timeout")
    parser.add_argument("--no-gui", action="store_true", help="Disable OpenCV window GUI")
    parser.add_argument("--disable-auto-publish", action="store_true", help="Start with publish OFF (press 's' to enable)")
    parser.add_argument("--motor-map", type=str, default=str(default_motor_map_path()), help="Path to motor map JSON")

    args, unknown = parser.parse_known_args()

    motor_map = load_motor_map(Path(args.motor_map))

    rclpy.init()
    node = ColleagueRatioJointStatePublisher(motor_map)

    cam = None
    cam_type = args.source

    if cam_type == "realsense":
        try:
            node.get_logger().info("Initializing Intel RealSense camera...")
            cam = RealSenseCamera(width=640, height=480, fps=30, serial=args.camera_serial, preferred_model=args.camera_model)
            node.get_logger().info(f"Intel RealSense camera [{cam.selected_device}] started successfully.")
        except Exception as e:
            node.get_logger().warn(f"Failed to start RealSense camera: {e}")
            node.get_logger().warn(f"Falling back to OpenCV Webcam (index {args.camera})...")
            cam_type = "webcam"


    if cam_type == "webcam":
        cam = OpenCVCamera(args.camera)
        if not cam.isOpened():
            node.get_logger().error(f"Failed to open webcam index {args.camera}")
            rclpy.shutdown()
            sys.exit(1)
        node.get_logger().info(f"OpenCV Webcam (index {args.camera}) started successfully.")

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils

    raw_smoothed = None
    target_ratios = make_zero_ratios()
    output_ratios = make_zero_ratios()

    publish_enabled = not args.disable_auto_publish

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
        node.get_logger().info("MediaPipe Hand Tracking initialized.")
        node.get_logger().info(f"Publish state: {'ON' if publish_enabled else 'OFF (press s to enable)'}")

        try:
            while rclpy.ok():
                ok, frame = cam.read()
                if not ok or frame is None:
                    time.sleep(0.01)
                    rclpy.spin_once(node, timeout_sec=0.001)
                    continue

                if args.rot != "none":
                    frame = rotate_frame(frame, args.rot)

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
                    raw_ratios = compute_finger_ratios(hand_landmarks, width, height)

                    raw_ratios = apply_max_ratio(raw_ratios, args.max_ratio)
                    raw_ratios = apply_deadzone(raw_ratios, args.deadzone)

                    raw_smoothed = exponential_smooth(
                        raw_smoothed,
                        raw_ratios,
                        args.alpha,
                    )
                    target_ratios = raw_smoothed.copy()

                    if not args.no_gui:
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

                if publish_enabled and (now - last_publish_time >= 1.0 / args.publish_rate):
                    last_publish_time = now
                    node.publish_ratios(output_ratios)

                if not args.no_gui:
                    draw_text(
                        frame,
                        target_ratios,
                        output_ratios,
                        publish_enabled,
                        hand_detected,
                        cam_type,
                    )

                    try:
                        cv2.imshow("LEAP Hand Colleague Ratio Teleop", frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord("s"):
                            publish_enabled = not publish_enabled
                            if not publish_enabled:
                                output_ratios = make_zero_ratios()
                                target_ratios = make_zero_ratios()
                                raw_smoothed = None
                                node.publish_ratios(make_zero_ratios())
                            node.get_logger().info(f"publish_enabled = {publish_enabled}")
                        elif key == ord("q"):
                            break
                    except cv2.error:
                        pass

                if now - last_print_time > 1.0:
                    last_print_time = now
                    node.get_logger().info(
                        f"[{cam_type.upper()}] Ratios -> index: {output_ratios['index']:.2f}, "
                        f"middle: {output_ratios['middle']:.2f}, "
                        f"ring: {output_ratios['ring']:.2f}, "
                        f"thumb: {output_ratios['thumb']:.2f} | "
                        f"Publish: {publish_enabled}, Hand: {hand_detected}"
                    )

                # Process ROS callbacks
                rclpy.spin_once(node, timeout_sec=0.001)

        finally:
            node.publish_ratios(make_zero_ratios())
            cam.release()
            cv2.destroyAllWindows()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()