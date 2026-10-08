#!/usr/bin/env python3
"""
D405 hand-visibility check (hand-held camera, no mount needed)
==============================================================
Hold the D405 where the mount would be (back of hand / wrist, lens toward the fingertips),
perform a few poses, and see how well MediaPipe + D405 depth see the fingers.

Install:
    pip install numpy opencv-python mediapipe pyrealsense2

Run (examples):
    python d405_hand_check.py --placement dorsal --rotate cw
    python d405_hand_check.py --placement palm   --rotate cw --save-video

Keys (focus the video window):
    0 free  1 open  2 fist  3 pinch  4 thumb_opp  5 spread  6 wrist_flex   -> set current pose label
    r  cycle image rotation (none -> cw -> ccw -> 180)   (find what MediaPipe likes best)
    m  toggle MediaPipe model complexity 0/1
    s  save snapshot (annotated + raw png)
    q  quit and write results

Output folder hand_check_<placement>_<time>/ :
    frames.csv   per-frame metrics
    summary.csv  per-pose summary (also printed on exit)
    raw.mp4      raw color video (only with --save-video)

Metrics
    detect%        frames where MediaPipe found a hand
    in-frame%      detected AND all 21 landmarks inside the image (no cropped fingers)
    tips-depth%    all 5 fingertips have valid depth >= 7 cm (D405 minimum range)
    too-close%     (among detected) at least one fingertip closer than 7 cm
    jitter mm      median frame-to-frame fingertip movement in the wrist frame
                   (hold the pose still: lower = more stable)
"""
import argparse
import csv
import time
from collections import deque
from pathlib import Path

import numpy as np

MIN_DEPTH_M = 0.07            # D405 minimum working distance (approx.)
TIPS = [4, 8, 12, 16, 20]     # thumb, index, middle, ring, pinky tips
ROT_CYCLE = [None, "cw", "ccw", "180"]
LABEL_KEYS = {ord("0"): "free", ord("1"): "open", ord("2"): "fist", ord("3"): "pinch",
              ord("4"): "thumb_opp", ord("5"): "spread", ord("6"): "wrist_flex"}

OPERATOR2MANO_RIGHT = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
OPERATOR2MANO_LEFT = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], dtype=float)


# ------------------------------------------------------------------ helpers
def rotated_pixel_to_original(u_r, v_r, W, H, rot):
    if rot is None:
        return u_r, v_r
    if rot == "cw":
        return v_r, H - 1 - u_r
    if rot == "ccw":
        return W - 1 - v_r, u_r
    if rot == "180":
        return W - 1 - u_r, H - 1 - v_r
    raise ValueError(rot)


def depth_median(depth_m, x, y, k=2):
    h, w = depth_m.shape
    xi, yi = int(round(x)), int(round(y))
    if not (0 <= xi < w and 0 <= yi < h):
        return 0.0
    patch = depth_m[max(0, yi - k): yi + k + 1, max(0, xi - k): xi + k + 1]
    patch = patch[patch > 0]
    return float(np.median(patch)) if patch.size else 0.0


def estimate_frame(kp):
    pts = kp[[0, 5, 9], :]
    x_vec = pts[0] - pts[2]
    pts = pts - pts.mean(axis=0, keepdims=True)
    _, _, vt = np.linalg.svd(pts)
    normal = vt[2, :]
    x = x_vec - np.sum(x_vec * normal) * normal
    x /= np.linalg.norm(x)
    z = np.cross(x, normal)
    if np.sum(z * (pts[1] - pts[2])) < 0:
        normal, z = -normal, -z
    return np.stack([x, normal, z], axis=1)


def to_wrist_frame(kp, hand):
    kp = kp - kp[0:1, :]
    o2m = OPERATOR2MANO_RIGHT if hand == "right" else OPERATOR2MANO_LEFT
    return kp @ estimate_frame(kp) @ o2m


def summarize(rows):
    """rows: list of dicts with keys label, detected, n_out, tips_ok, too_close, jitter_mm."""
    out = []
    labels = []
    for r in rows:
        if r["label"] not in labels:
            labels.append(r["label"])
    for lab in labels + ["ALL"]:
        rs_ = rows if lab == "ALL" else [r for r in rows if r["label"] == lab]
        n = len(rs_)
        det = [r for r in rs_ if r["detected"]]
        jit = [r["jitter_mm"] for r in rs_ if r["jitter_mm"] == r["jitter_mm"]]  # drop NaN
        out.append({
            "label": lab,
            "frames": n,
            "detect%": 100.0 * len(det) / n if n else 0.0,
            "in-frame%": 100.0 * sum(1 for r in det if r["n_out"] == 0) / n if n else 0.0,
            "tips-depth%": 100.0 * sum(1 for r in det if r["tips_ok"] == 5) / n if n else 0.0,
            "too-close%": 100.0 * sum(1 for r in det if r["too_close"]) / len(det) if det else 0.0,
            "jitter_mm": float(np.median(jit)) if jit else float("nan"),
        })
    return out


def print_summary(summ):
    print("\n{:<11}{:>8}{:>10}{:>11}{:>13}{:>12}{:>12}".format(
        "pose", "frames", "detect%", "in-frame%", "tips-depth%", "too-close%", "jitter mm"))
    for s in summ:
        print("{:<11}{:>8}{:>10.1f}{:>11.1f}{:>13.1f}{:>12.1f}{:>12.2f}".format(
            s["label"], s["frames"], s["detect%"], s["in-frame%"], s["tips-depth%"],
            s["too-close%"], s["jitter_mm"]))


# ------------------------------------------------------------------ main
def main():
    import cv2
    import mediapipe as mp
    import pyrealsense2 as rs

    ap = argparse.ArgumentParser()
    ap.add_argument("--placement", default="dorsal", help="free text tag, e.g. dorsal / palm / wrist")
    ap.add_argument("--rotate", choices=["none", "cw", "ccw", "180"], default="cw")
    ap.add_argument("--hand", choices=["right", "left"], default="right")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--save-video", action="store_true", help="save raw color video")
    args = ap.parse_args()

    W, H = args.width, args.height
    rot_idx = ROT_CYCLE.index(None if args.rotate == "none" else args.rotate)
    cv_rot = {"cw": cv2.ROTATE_90_CLOCKWISE, "ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "180": cv2.ROTATE_180}

    outdir = Path(f"hand_check_{args.placement}_{time.strftime('%Y%m%d_%H%M%S')}")
    outdir.mkdir(parents=True, exist_ok=True)

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, args.fps)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, args.fps)
    prof = pipe.start(cfg)
    depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    complexity = 1

    def make_hands(c):
        return mp.solutions.hands.Hands(max_num_hands=1, model_complexity=c,
                                        min_detection_confidence=0.5, min_tracking_confidence=0.5)

    hands = make_hands(complexity)
    drawer, conns = mp.solutions.drawing_utils, mp.solutions.hands.HAND_CONNECTIONS

    writer = None
    if args.save_video:
        writer = cv2.VideoWriter(str(outdir / "raw.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W, H))

    label = "free"
    rows, prev_tips, snap_n = [], None, 0
    live = deque(maxlen=90)       # (detected, in_frame, tips_ok5) for the HUD
    fcsv = open(outdir / "frames.csv", "w", newline="")
    wcsv = csv.writer(fcsv)
    wcsv.writerow(["t", "label", "rotate", "complexity", "detected", "n_out", "tips_ok",
                   "min_tip_depth_m", "too_close", "jitter_mm"])

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            cf, df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df:
                continue
            raw = np.asanyarray(cf.get_data())
            depth_m = np.asanyarray(df.get_data()).astype(np.float32) * depth_scale
            rot = ROT_CYCLE[rot_idx]
            view = cv2.rotate(raw, cv_rot[rot]) if rot else raw.copy()
            hr, wr = view.shape[:2]
            res = hands.process(cv2.cvtColor(view, cv2.COLOR_BGR2RGB))

            detected = bool(res.multi_hand_landmarks)
            n_out, tips_ok, min_tip, too_close, jitter = 0, 0, float("nan"), False, float("nan")
            if detected:
                hl = res.multi_hand_landmarks[0]
                lms = hl.landmark
                n_out = sum(1 for p in lms if p.x < 0 or p.x > 1 or p.y < 0 or p.y > 1)
                zs, pts = [], []
                for p in lms:
                    x, y = rotated_pixel_to_original(p.x * wr, p.y * hr, W, H, rot)
                    z = depth_median(depth_m, x, y)
                    zs.append(z)
                    pts.append(rs.rs2_deproject_pixel_to_point(intr, [x, y], z) if z > 0
                               else [np.nan, np.nan, np.nan])
                zs = np.array(zs)
                tz = zs[TIPS]
                tips_ok = int(np.sum(tz >= MIN_DEPTH_M))
                valid_tz = tz[tz > 0]
                min_tip = float(valid_tz.min()) if valid_tz.size else float("nan")
                too_close = bool(((tz > 0) & (tz < MIN_DEPTH_M)).any())
                kp3 = np.array(pts, dtype=float)
                if np.isfinite(kp3).all():
                    tips = to_wrist_frame(kp3, args.hand)[TIPS]
                    if prev_tips is not None:
                        jitter = float(np.mean(np.linalg.norm(tips - prev_tips, axis=1)) * 1000.0)
                    prev_tips = tips
                else:
                    prev_tips = None
                drawer.draw_landmarks(view, hl, conns)
                for i, t in enumerate(TIPS):
                    px, py = int(lms[t].x * wr), int(lms[t].y * hr)
                    ok = zs[t] >= MIN_DEPTH_M
                    txt = "--" if zs[t] <= 0 else f"{zs[t] * 100:.0f}"
                    cv2.putText(view, txt, (px + 6, py - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0) if ok else (0, 0, 255), 2)
            else:
                prev_tips = None

            row = {"label": label, "detected": detected, "n_out": n_out, "tips_ok": tips_ok,
                   "too_close": too_close, "jitter_mm": jitter}
            rows.append(row)
            live.append((detected, detected and n_out == 0, detected and tips_ok == 5))
            wcsv.writerow([f"{time.time():.3f}", label, rot or "none", complexity, int(detected), n_out,
                           tips_ok, f"{min_tip:.3f}", int(too_close), f"{jitter:.2f}"])
            if writer is not None:
                writer.write(raw)

            n = len(live)
            hud = [f"{args.placement} | rot={rot or 'none'} | model={complexity} | pose={label}",
                   f"last {n}f: detect {100 * sum(a for a, _, _ in live) / n:.0f}%  "
                   f"in-frame {100 * sum(b for _, b, _ in live) / n:.0f}%  "
                   f"tips-depth {100 * sum(c for _, _, c in live) / n:.0f}%"]
            for i, line in enumerate(hud):
                cv2.putText(view, line, (8, 20 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 255), 2)
            cv2.imshow("D405 hand check (q quit)", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k in LABEL_KEYS:
                label = LABEL_KEYS[k]
                prev_tips = None
            elif k == ord("r"):
                rot_idx = (rot_idx + 1) % len(ROT_CYCLE)
                prev_tips = None
            elif k == ord("m"):
                complexity = 1 - complexity
                hands.close()
                hands = make_hands(complexity)
            elif k == ord("s"):
                snap_n += 1
                cv2.imwrite(str(outdir / f"snap_{snap_n:02d}.png"), view)
                cv2.imwrite(str(outdir / f"snap_{snap_n:02d}_raw.png"), raw)
    finally:
        fcsv.close()
        if writer is not None:
            writer.release()
        pipe.stop()
        cv2.destroyAllWindows()
        if rows:
            summ = summarize(rows)
            print_summary(summ)
            with open(outdir / "summary.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(summ[0].keys()))
                w.writeheader()
                w.writerows(summ)
            print(f"\nsaved to {outdir}/")


if __name__ == "__main__":
    main()
