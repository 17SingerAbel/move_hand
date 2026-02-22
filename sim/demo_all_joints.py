#!/usr/bin/env python3
"""Run whole-hand pose transitions from a YAML pose file."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

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
    return logging.getLogger("demo_all_joints")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Whole-hand pose demo")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
    parser.add_argument("--poses", default="config/poses.yaml", help="Path to poses YAML")
    parser.add_argument(
        "--sequence",
        default="open,half_close,close,half_close,open",
        help="Comma-separated pose sequence",
    )
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--hold", type=float, default=0.8)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cam-distance", type=float, default=0.4)
    parser.add_argument("--cam-yaw", type=float, default=90.0)
    parser.add_argument("--cam-pitch", type=float, default=-15.0)
    parser.add_argument("--cam-target-x", type=float, default=0.0)
    parser.add_argument("--cam-target-y", type=float, default=0.0)
    parser.add_argument("--cam-target-z", type=float, default=0.05)
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    parser.add_argument("--log-file", default=None, help="Optional log output file path")
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

    out: dict[str, list[float]] = {}
    for name, values in data.items():
        if not isinstance(values, list):
            raise ValueError(f"Pose '{name}' must be a list")
        out[str(name)] = [float(v) for v in values]
    return out


def synthesize_default_poses(joint_count: int) -> dict[str, list[float]]:
    return {
        "open": [0.0] * joint_count,
        "half_close": [0.35] * joint_count,
        "close": [0.7] * joint_count,
    }


def main() -> int:
    args = parse_args()
    logger = setup_logging(args.log_level, args.log_file)

    ctrl = JointController(args.urdf, gui=not args.headless, logger=logging.getLogger("joint_controller"))
    try:
        if not args.headless:
            ctrl.p.resetDebugVisualizerCamera(
                cameraDistance=args.cam_distance,
                cameraYaw=args.cam_yaw,
                cameraPitch=args.cam_pitch,
                cameraTargetPosition=[args.cam_target_x, args.cam_target_y, args.cam_target_z],
            )

        joint_count = len(ctrl.joints)
        logger.info("Movable joints: %d", joint_count)
        if joint_count == 0:
            logger.error("No movable joints found")
            return 2

        poses = load_poses(Path(args.poses))
        needs_auto = any(len(v) == 0 for v in poses.values())
        if not poses or needs_auto:
            logger.warning("Pose file is empty or placeholder. Using synthesized default poses.")
            poses = synthesize_default_poses(joint_count)

        for pose_name, values in poses.items():
            if len(values) != joint_count:
                logger.error(
                    "Pose '%s' length mismatch: expected %d got %d",
                    pose_name,
                    joint_count,
                    len(values),
                )
                return 2

        sequence = [name.strip() for name in args.sequence.split(",") if name.strip()]
        for name in sequence:
            if name not in poses:
                logger.error("Pose '%s' not found in %s", name, args.poses)
                return 2

        logger.info("Running sequence: %s", " -> ".join(sequence))
        for cycle in range(args.cycles):
            for pose_name in sequence:
                applied = ctrl.set_all_joints(poses[pose_name])
                for _ in range(int(max(1, args.hold * 240))):
                    ctrl.step()
                    if not args.headless:
                        time.sleep(1 / 240.0)
                current = ctrl.get_joint_positions()
                logger.info(
                    "[CYCLE %02d] pose=%s first_joint target=%+.3f applied=%+.3f current=%+.3f",
                    cycle + 1,
                    pose_name,
                    poses[pose_name][0],
                    applied[0],
                    current[0],
                )

        logger.info("Whole-hand demo completed")
        return 0
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
