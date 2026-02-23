#!/usr/bin/env python3
"""Minimal CLI control for hand poses."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_DIR = ROOT / "sim"
if str(SIM_DIR) not in sys.path:
    sys.path.insert(0, str(SIM_DIR))

from joint_controller import JointController


def setup_logging(level: str, log_file: str | None) -> logging.Logger:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        Path(log_file).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        handlers=handlers,
    )
    return logging.getLogger("cli_control")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal hand CLI control")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
    parser.add_argument("--poses", default="config/poses.yaml", help="Path to poses YAML")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--cam-distance", type=float, default=0.4)
    parser.add_argument("--cam-yaw", type=float, default=90.0)
    parser.add_argument("--cam-pitch", type=float, default=-15.0)
    parser.add_argument("--cam-target-x", type=float, default=0.0)
    parser.add_argument("--cam-target-y", type=float, default=0.0)
    parser.add_argument("--cam-target-z", type=float, default=0.05)
    parser.add_argument("--transition-time", type=float, default=0.4, help="Seconds for smooth transition")
    parser.add_argument("--transition-steps", type=int, default=40, help="Interpolation steps")
    parser.add_argument("--sim-steps-per-waypoint", type=int, default=6, help="Simulation steps per waypoint")
    parser.add_argument("--hold", type=float, default=0.0, help="Seconds to hold after reaching target")
    parser.add_argument("--middle-ratio", type=float, default=0.0)
    parser.add_argument("--others-ratio", type=float, default=1.0)
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    parser.add_argument("--log-file", default=None, help="Optional log output file path")

    sub = parser.add_subparsers(dest="command", required=True)
    pose = sub.add_parser("pose", help="Apply a pose")
    pose.add_argument("name", help="Pose name: middle/home/open/close/... from poses.yaml")
    return parser.parse_args()


def load_poses(path: Path) -> dict[str, list[float]]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("Missing dependency: pyyaml. Run: pip install -r requirements.txt") from exc

    if not path.exists():
        raise FileNotFoundError(f"Pose file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("Pose file must map pose_name -> list[float]")

    poses: dict[str, list[float]] = {}
    for name, values in data.items():
        if not isinstance(values, list):
            raise ValueError(f"Pose '{name}' must be a list")
        poses[str(name)] = [float(v) for v in values]
    return poses


def ratio_to_target(lower: float, upper: float, ratio: float) -> float:
    ratio = max(0.0, min(1.0, ratio))
    return lower + ratio * (upper - lower)


def build_middle_finger_pose(ctrl: JointController, middle_ratio: float, others_ratio: float) -> list[float]:
    targets: list[float] = []
    for joint in ctrl.joints:
        is_middle = "middle" in joint.name
        ratio = middle_ratio if is_middle else others_ratio
        targets.append(ratio_to_target(joint.lower, joint.upper, ratio))
    return targets


def smooth_move(
    ctrl: JointController,
    target: list[float],
    duration_s: float,
    steps: int,
    sim_steps_per_waypoint: int,
    gui: bool,
) -> None:
    start = ctrl.get_joint_positions()
    n = len(ctrl.joints)
    if len(target) != n:
        raise ValueError(f"Target length mismatch: expected {n} got {len(target)}")

    steps = max(1, int(steps))
    dt = max(0.0, float(duration_s)) / steps
    for i in range(1, steps + 1):
        alpha = i / steps
        waypoint = [start[j] + alpha * (target[j] - start[j]) for j in range(n)]
        ctrl.set_all_joints(waypoint)
        ctrl.step(steps=max(1, int(sim_steps_per_waypoint)))
        if gui and dt > 0:
            time.sleep(dt)


def resolve_pose_target(args: argparse.Namespace, ctrl: JointController, poses: dict[str, list[float]]) -> tuple[str, list[float]]:
    name = args.name.strip()
    if name == "middle":
        return "middle", build_middle_finger_pose(ctrl, args.middle_ratio, args.others_ratio)

    if name in poses:
        return name, poses[name]

    raise ValueError(f"Unknown pose '{name}'. Use 'middle' or one of: {', '.join(sorted(poses.keys()))}")


def main() -> int:
    args = parse_args()
    logger = setup_logging(args.log_level, args.log_file)

    ctrl = JointController(args.urdf, gui=not args.headless, logger=logging.getLogger("joint_controller"))
    try:
        if not ctrl.joints:
            logger.error("No movable joints found")
            return 2

        if not args.headless:
            ctrl.p.resetDebugVisualizerCamera(
                cameraDistance=args.cam_distance,
                cameraYaw=args.cam_yaw,
                cameraPitch=args.cam_pitch,
                cameraTargetPosition=[args.cam_target_x, args.cam_target_y, args.cam_target_z],
            )

        poses = load_poses(Path(args.poses))
        pose_name, target = resolve_pose_target(args, ctrl, poses)
        smooth_move(
            ctrl,
            target,
            duration_s=args.transition_time,
            steps=args.transition_steps,
            sim_steps_per_waypoint=args.sim_steps_per_waypoint,
            gui=not args.headless,
        )

        hold_steps = int(max(0.0, args.hold) * 240)
        if hold_steps > 0:
            for _ in range(hold_steps):
                ctrl.step()
                if not args.headless:
                    time.sleep(1 / 240.0)

        current = ctrl.get_joint_positions()
        logger.info(
            "Pose applied: %s first_joint target=%+.3f current=%+.3f",
            pose_name,
            target[0],
            current[0],
        )
        return 0
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
