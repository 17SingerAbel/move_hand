#!/usr/bin/env python3
"""Toggle one joint between min/max limits for quick validation."""

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
    return logging.getLogger("demo_single_joint")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Single-joint oscillation demo")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
    parser.add_argument(
        "--joint",
        type=int,
        default=0,
        help="Index in movable-joint list (not raw URDF joint index)",
    )
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--hold", type=float, default=0.6)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cam-distance", type=float, default=0.4, help="Camera distance")
    parser.add_argument("--cam-yaw", type=float, default=90.0, help="Camera yaw in degrees")
    parser.add_argument("--cam-pitch", type=float, default=-15.0, help="Camera pitch in degrees")
    parser.add_argument("--cam-target-x", type=float, default=0.0, help="Camera target x")
    parser.add_argument("--cam-target-y", type=float, default=0.0, help="Camera target y")
    parser.add_argument("--cam-target-z", type=float, default=0.05, help="Camera target z")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    parser.add_argument("--log-file", default=None, help="Optional log output file path")
    return parser.parse_args()


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
                cameraTargetPosition=[
                    args.cam_target_x,
                    args.cam_target_y,
                    args.cam_target_z,
                ],
            )

        if not ctrl.joints:
            logger.error("No movable joints found")
            return 2
        if args.joint < 0 or args.joint >= len(ctrl.joints):
            logger.error("--joint must be in [0, %d]", len(ctrl.joints) - 1)
            return 2

        joint = ctrl.joints[args.joint]
        logger.info("Testing joint[%d] -> %s idx=%d", args.joint, joint.name, joint.index)
        logger.info("Range [%.3f, %.3f]", joint.lower, joint.upper)

        targets = [joint.lower, joint.upper]
        for c in range(args.cycles):
            target = targets[c % 2]
            applied = ctrl.set_joint(args.joint, target)
            for _ in range(int(max(1, args.hold * 240))):
                ctrl.step()
                if not args.headless:
                    time.sleep(1 / 240.0)
            pos = ctrl.get_joint_positions()[args.joint]
            logger.info(
                "[CYCLE %02d] target=%+.3f applied=%+.3f current=%+.3f",
                c + 1,
                target,
                applied,
                pos,
            )

        logger.info("Single-joint demo completed")
        return 0
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
