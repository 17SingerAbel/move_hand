#!/usr/bin/env python3
"""Middle-finger pose demo: keep middle finger straight, curl others."""

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
    return logging.getLogger("demo_middle_finger")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Middle finger pose demo")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
    parser.add_argument("--hold", type=float, default=3.0, help="Seconds to hold target pose")
    parser.add_argument(
        "--middle-ratio",
        type=float,
        default=0.0,
        help="0.0=middle fully straight (lower limit), 1.0=middle fully curled (upper limit)",
    )
    parser.add_argument(
        "--others-ratio",
        type=float,
        default=1.0,
        help="0.0=others straight, 1.0=others fully curled",
    )
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


def ratio_to_target(lower: float, upper: float, ratio: float) -> float:
    ratio = max(0.0, min(1.0, ratio))
    return lower + ratio * (upper - lower)


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

        if not ctrl.joints:
            logger.error("No movable joints found")
            return 2

        targets: list[float] = []
        for joint in ctrl.joints:
            # URDF names include "middle_mcp_pitch" and "middle_dip" for middle finger.
            is_middle = "middle" in joint.name
            ratio = args.middle_ratio if is_middle else args.others_ratio
            target = ratio_to_target(joint.lower, joint.upper, ratio)
            targets.append(target)
            logger.info(
                "joint=%-18s role=%s target=%+.3f range=[%+.3f, %+.3f]",
                joint.name,
                "middle" if is_middle else "other",
                target,
                joint.lower,
                joint.upper,
            )

        applied = ctrl.set_all_joints(targets)
        steps = int(max(1, args.hold * 240))
        for _ in range(steps):
            ctrl.step()
            if not args.headless:
                time.sleep(1 / 240.0)

        current = ctrl.get_joint_positions()
        logger.info("Pose applied. Sample first joint: target=%+.3f applied=%+.3f current=%+.3f", targets[0], applied[0], current[0])
        logger.info("Middle-finger demo completed")
        return 0
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
