import json
from pathlib import Path


MAP_FILE = (
    Path(__file__).resolve().parent.parent
    / "leap_teleop" / "config" / "left_hand_motor_map.json"
)


def shortest_angle_delta(open_deg, close_deg):
    """
    open -> close로 갈 때 360도 경계에서 짧은 방향으로 가는 차이 계산
    예: 360.53 -> 32.87 은 +32.34도 방향으로 처리
    """
    delta = close_deg - open_deg

    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360

    return delta


def interpolate_pose(open_pose, close_pose, ratio):
    result = {}

    for motor_id in open_pose:
        open_deg = float(open_pose[motor_id])
        close_deg = float(close_pose[motor_id])

        delta = shortest_angle_delta(open_deg, close_deg)
        target = open_deg + delta * ratio

        # 0~360 범위로 정리
        target = target % 360

        result[motor_id] = round(target, 2)

    return result


def main():
    with open(MAP_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    open_pose = data["open_pose_deg"]
    close_pose = data["close_pose_deg"]

    for ratio in [0.2, 0.5, 1.0]:
        print(f"\n===== close {int(ratio * 100)}% pose =====")
        pose = interpolate_pose(open_pose, close_pose, ratio)

        for motor_id in sorted(pose.keys(), key=lambda x: int(x)):
            print(f"ID {motor_id:>2}: {pose[motor_id]:>7} deg")


if __name__ == "__main__":
    main()