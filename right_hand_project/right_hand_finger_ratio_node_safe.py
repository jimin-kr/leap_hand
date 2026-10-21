import argparse
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState


from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
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
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

# Position PID / Goal Current (RAM, 2 bytes each)
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102

LEN_GOAL_POSITION = 4
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_VELOCITY = 4
LEN_PRESENT_POSITION = 4

ADDR_SYNC_READ_START = 126
LEN_SYNC_READ = 10  # Current(2) + Velocity(4) + Position(4)

TORQUE_OFF = 0
TORQUE_ON = 1

FINGER_TOPIC = "/right_hand/finger_ratios"
ARM_TOPIC = "/right_hand/arm"
SPREAD_TOPIC = "/right_hand/finger_spread"
FALLBACK_FINGER_TOPIC = "/left_hand/finger_ratios"
FALLBACK_ARM_TOPIC = "/left_hand/arm"
FALLBACK_SPREAD_TOPIC = "/left_hand/finger_spread"

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


def make_target_ticks(motor_map, ratios, base_ticks, spreads=None):
    """target = base_open_tick(시작 시 스캔) + map delta * ratio (and spread for 1, 5, 9)"""
    target = {}
    spread_ids = {1: "index", 5: "middle", 9: "ring"}
    spread_signs = motor_map.get("spread_sign", {"1": -1.0, "5": -1.0, "9": 1.0})
    joint_limits = motor_map.get("joint_limits_tick", {
        "1": [1934, 2162],
        "5": [1934, 2162],
        "9": [1934, 2162]
    })

    if spreads is None:
        spreads = {"index": 0.0, "middle": 0.0, "ring": 0.0}

    for finger in FINGER_NAMES:
        ratio = clamp(ratios[finger])

        for dxl_id in motor_map["ids"][finger]:
            dxl_id = int(dxl_id)
            key = str(dxl_id)

            if dxl_id in spread_ids:
                # Motor 1, 5, 9: MCP side (spread)
                # Reference center is 2048 (open state).
                # Travel limit is strictly ±10 degrees (±114 ticks) -> [1934, 2162]
                finger_key = spread_ids[dxl_id]
                s_val = max(-1.0, min(1.0, float(spreads.get(finger_key, 0.0))))
                sign = float(spread_signs.get(key, -1.0 if dxl_id in (1, 5) else 1.0))

                base = float(base_ticks.get(dxl_id, 2048))
                raw_tick = int(round(base + sign * s_val * 114.0))

                limits = joint_limits.get(key, [1934, 2162])
                low_limit = int(limits[0])
                high_limit = int(limits[1])
                clamped_tick = max(low_limit, min(high_limit, raw_tick))
                target[dxl_id] = clamped_tick % 4096
            else:
                delta = map_delta_tick(motor_map, key)
                target[dxl_id] = int(round(base_ticks[dxl_id] + delta * ratio)) % 4096

    return target


class LeftHandSafeFingerRatioNode(Node):
    def __init__(self, args):
        super().__init__("left_hand_finger_ratio_node_safe")

        self.args = args
        self.motor_map = load_motor_map(Path(args.motor_map))

        self.baudrate = args.baudrate if args.baudrate is not None else int(self.motor_map.get("baudrate", 4000000))
        self.port_name = self.resolve_port(args.port, self.baudrate)
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

        self.comm_lock = threading.Lock()
        self.sync_read = None
        self.feedback_rate = float(getattr(args, "feedback_rate", 30.0))

        self.urdf_joint = self.motor_map.get("urdf_joint", {str(i): str(i) for i in range(16)})
        self.urdf_sign = self.motor_map.get("urdf_sign", {})

        self.armed = False

        self.last_ratios = {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }
        self.last_spreads = {
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
        }

        self.open_hardware()

        # Real-time state feedback publishers for VLA & visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/right_hand/joint_states",
            10,
        )
        self.current_pub = self.create_publisher(
            Float32MultiArray,
            "/right_hand/motor_currents_mA",
            10,
        )

        # High-speed feedback loop timer
        self.feedback_timer = self.create_timer(
            1.0 / self.feedback_rate,
            self.feedback_callback,
        )

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

        self.spread_sub = self.create_subscription(
            Float32MultiArray,
            SPREAD_TOPIC,
            self.spread_callback,
            10,
        )
        self.fallback_spread_sub = self.create_subscription(
            Float32MultiArray,
            FALLBACK_SPREAD_TOPIC,
            self.spread_callback,
            10,
        )

        self.get_logger().info("Right Hand SAFE Finger Ratio Node started")
        self.get_logger().info(f"Subscribed: {FINGER_TOPIC} (fallback: {FALLBACK_FINGER_TOPIC})")
        self.get_logger().info(f"Subscribed: {ARM_TOPIC} (fallback: {FALLBACK_ARM_TOPIC})")
        self.get_logger().info(f"Publishing: /right_hand/joint_states (SyncRead @ {self.feedback_rate:.1f} Hz)")
        self.get_logger().info("Initial state: DISARMED")
        self.get_logger().info("Arm command:")
        self.get_logger().info(
            'ros2 topic pub --once /right_hand/arm std_msgs/msg/Bool "{data: true}"'
        )
        self.get_logger().info("Disarm command:")
        self.get_logger().info(
            'ros2 topic pub --once /right_hand/arm std_msgs/msg/Bool "{data: false}"'
        )


    def resolve_port(self, preferred_port, baudrate):
        import glob
        pkt = PacketHandler(PROTOCOL_VERSION)
        candidates = []
        if preferred_port:
            candidates.append(preferred_port)
        for p in sorted(glob.glob("/dev/ttyUSB*")):
            if p not in candidates:
                candidates.append(p)
        for p in candidates:
            try:
                ph = PortHandler(p)
                if ph.openPort():
                    if ph.setBaudRate(baudrate):
                        _, res, _ = pkt.ping(ph, 0)
                        ph.closePort()
                        if res == COMM_SUCCESS:
                            self.get_logger().info(f"Dynamixel response detected on {p}")
                            return p
                    ph.closePort()
            except Exception:
                pass
        return preferred_port or "/dev/ttyUSB0"


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
        self.setup_sync_read()
        self.capture_open_baseline()

        self.get_logger().info("hardware ready, torque OFF, DISARMED")

    def setup_sync_read(self):
        self.sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_SYNC_READ_START,
            LEN_SYNC_READ,
        )
        for dxl_id in self.dxl_ids:
            ok = self.sync_read.addParam(int(dxl_id))
            if not ok:
                raise RuntimeError(f"ID {dxl_id} GroupSyncRead addParam failed")
        self.get_logger().info("GroupSyncRead initialized for 16 motors (Cur+Vel+Pos, 10 bytes)")


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
                hw, r_hw, _ = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, 70)
                if r_hw == COMM_SUCCESS and hw != 0:
                    self.get_logger().warn(f"ID {dxl_id} HW ERROR 0x{hw:02X} detected! Auto-rebooting...")
                    self.packet_handler.reboot(self.port_handler, dxl_id)
                    time.sleep(0.2)
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
        with self.comm_lock:
            if self.sync_read is not None:
                res = self.sync_read.txRxPacket()
                if res == COMM_SUCCESS:
                    positions = {}
                    for dxl_id in self.dxl_ids:
                        if self.sync_read.isAvailable(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                            pos = self.sync_read.getData(int(dxl_id), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                            positions[int(dxl_id)] = int(pos) % 4096
                        else:
                            positions[int(dxl_id)] = 2048
                    return positions

            # Fallback to single read4 if sync_read not available or failed
            positions = {}
            for dxl_id in self.dxl_ids:
                tick = self.read4(dxl_id, ADDR_PRESENT_POSITION, "present position")
                positions[int(dxl_id)] = int(tick) % 4096
            return positions

    def feedback_callback(self):
        if self.sync_read is None:
            return

        with self.comm_lock:
            res = self.sync_read.txRxPacket()
            if res != COMM_SUCCESS:
                return

            now = self.get_clock().now().to_msg()
            joint_msg = JointState()
            joint_msg.header.stamp = now

            names = []
            positions = []
            velocities = []
            efforts = []
            currents_list = []

            for dxl_id in range(16):
                key = str(dxl_id)
                names.append(self.urdf_joint.get(key, key))

                # Position in radians relative to calibrated open baseline
                if self.sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    tick = self.sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) % 4096
                    base = self.base_ticks.get(dxl_id, 2048) if hasattr(self, "base_ticks") else 2048
                    delta_tick = tick - base
                    if delta_tick > 2048:
                        delta_tick -= 4096
                    elif delta_tick < -2048:
                        delta_tick += 4096
                    rad = float(self.urdf_sign.get(key, 1)) * delta_tick * (2.0 * math.pi / 4096.0)
                    positions.append(float(rad))
                else:
                    positions.append(0.0)

                # Velocity in rad/s (XC330 raw unit is 0.229 rpm -> rad/s: * 0.02398)
                if self.sync_read.isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
                    vel = self.sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
                    if vel > 2147483647:
                        vel -= 4294967296
                    velocities.append(float(vel) * 0.02398)
                else:
                    velocities.append(0.0)

                # Present Current (mA)
                if self.sync_read.isAvailable(dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT):
                    cur = self.sync_read.getData(dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
                    if cur > 32767:
                        cur -= 65536
                    efforts.append(float(cur))
                    currents_list.append(float(cur))
                else:
                    efforts.append(0.0)
                    currents_list.append(0.0)

            joint_msg.name = names
            joint_msg.position = positions
            joint_msg.velocity = velocities
            joint_msg.effort = efforts

            self.joint_state_pub.publish(joint_msg)

            cur_msg = Float32MultiArray()
            cur_msg.data = currents_list
            self.current_pub.publish(cur_msg)

    def sync_write_goal_ticks(self, goal_ticks):
        with self.comm_lock:
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

    def spread_callback(self, msg):
        if len(msg.data) < 3:
            return
        self.last_spreads = {
            "index": max(-1.0, min(1.0, float(msg.data[0]))),
            "middle": max(-1.0, min(1.0, float(msg.data[1]))),
            "ring": max(-1.0, min(1.0, float(msg.data[2]))),
        }
        if not self.armed:
            return
        try:
            target_ticks = make_target_ticks(
                self.motor_map, self.last_ratios, self.base_ticks, self.last_spreads
            )
            self.sync_write_goal_ticks(target_ticks)
        except Exception as e:
            self.get_logger().error(f"failed to send finger spread: {e}")

    def finger_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn(
                "finger ratio message must have at least 4 values: [thumb, index, middle, ring]"
            )
            return

        ratios = {
            "thumb": clamp(msg.data[0]),
            "index": clamp(msg.data[1]),
            "middle": clamp(msg.data[2]),
            "ring": clamp(msg.data[3]),
        }

        if len(msg.data) >= 7:
            self.last_spreads = {
                "index": max(-1.0, min(1.0, float(msg.data[4]))),
                "middle": max(-1.0, min(1.0, float(msg.data[5]))),
                "ring": max(-1.0, min(1.0, float(msg.data[6]))),
            }

        self.last_ratios = ratios

        if not self.armed:
            self.get_logger().info(
                "finger ratio received but node is DISARMED. command ignored."
            )
            return

        try:
            target_ticks = make_target_ticks(
                self.motor_map, ratios, self.base_ticks, self.last_spreads
            )
            self.sync_write_goal_ticks(target_ticks)

            self.get_logger().info(
                f"sent ratios: "
                f"T={ratios['thumb']:.2f}, "
                f"I={ratios['index']:.2f}, "
                f"M={ratios['middle']:.2f}, "
                f"R={ratios['ring']:.2f} | "
                f"spread: I={self.last_spreads['index']:+.2f}, "
                f"M={self.last_spreads['middle']:+.2f}, "
                f"R={self.last_spreads['ring']:+.2f}"
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
    parser.add_argument(
        "--feedback-rate",
        type=float,
        default=30.0,
        help="모터 상태(SyncRead) 피드백 및 토픽 발행 주기 [Hz] (기본: 30)",
    )

    args, ros_args = parser.parse_known_args()


    rclpy.init(args=ros_args)

    node = None

    try:
        node = LeftHandSafeFingerRatioNode(args)
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
