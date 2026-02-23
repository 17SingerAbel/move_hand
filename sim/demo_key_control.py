#!/usr/bin/env python3
"""Interactive key control: middle-finger pose and restore boot pose."""

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
    return logging.getLogger("demo_key_control")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive key control demo")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
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


def build_middle_finger_pose(ctrl: JointController, middle_ratio: float, others_ratio: float) -> list[float]:
    targets: list[float] = []
    for joint in ctrl.joints:
        is_middle = "middle" in joint.name
        ratio = middle_ratio if is_middle else others_ratio
        target = ratio_to_target(joint.lower, joint.upper, ratio)
        targets.append(target)
    return targets


def is_key_triggered(events: dict[int, int], key_code: int, key_triggered_flag: int) -> bool:
    return key_code in events and bool(events[key_code] & key_triggered_flag)


def main() -> int:
    args = parse_args()
    logger = setup_logging(args.log_level, args.log_file)

    ctrl = JointController(args.urdf, gui=True, logger=logging.getLogger("joint_controller"))
    try:
        ctrl.p.resetDebugVisualizerCamera(
            cameraDistance=args.cam_distance,
            cameraYaw=args.cam_yaw,
            cameraPitch=args.cam_pitch,
            cameraTargetPosition=[args.cam_target_x, args.cam_target_y, args.cam_target_z],
        )
        if not ctrl.joints:
            logger.error("No movable joints found")
            return 2

        boot_pose = ctrl.get_joint_positions()
        middle_pose = build_middle_finger_pose(ctrl, args.middle_ratio, args.others_ratio)
        logger.info("Interactive control ready: M=middle-finger pose, O=restore boot pose, Q=quit")
        logger.info("Captured boot pose with %d joints", len(boot_pose))

        while True:
            events = ctrl.p.getKeyboardEvents()
            if (
                is_key_triggered(events, ord("m"), ctrl.p.KEY_WAS_TRIGGERED)
                or is_key_triggered(events, ord("M"), ctrl.p.KEY_WAS_TRIGGERED)
            ):
                ctrl.set_all_joints(middle_pose)
                logger.info("Applied middle-finger pose")

            if (
                is_key_triggered(events, ord("o"), ctrl.p.KEY_WAS_TRIGGERED)
                or is_key_triggered(events, ord("O"), ctrl.p.KEY_WAS_TRIGGERED)
            ):
                ctrl.set_all_joints(boot_pose)
                logger.info("Restored boot pose")

            if (
                is_key_triggered(events, ord("q"), ctrl.p.KEY_WAS_TRIGGERED)
                or is_key_triggered(events, ord("Q"), ctrl.p.KEY_WAS_TRIGGERED)
            ):
                logger.info("Quit requested")
                break

            ctrl.step()
            time.sleep(1 / 240.0)

        return 0
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 0
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
