#!/usr/bin/env python3
"""Load Linker Hand URDF in PyBullet and print controllable joints."""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path


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
    return logging.getLogger("sim_load")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Load URDF and list movable joints")
    parser.add_argument("--urdf", required=True, help="Absolute or relative path to hand URDF")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without GUI (useful in CI)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to keep simulation alive (default: 10)",
    )
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

    urdf_path = Path(args.urdf).expanduser().resolve()
    if not urdf_path.exists():
        logger.error("URDF not found: %s", urdf_path)
        return 2

    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        logger.error("Missing dependency: pybullet")
        logger.info("Install with: linkhand/bin/pip install -r requirements.txt")
        return 3

    connection_mode = p.DIRECT if args.headless else p.GUI
    client = p.connect(connection_mode)
    if client < 0:
        logger.error("Failed to connect to PyBullet")
        return 4

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        hand_id = p.loadURDF(str(urdf_path), useFixedBase=True)

        if not args.headless:
            p.resetDebugVisualizerCamera(
                cameraDistance=args.cam_distance,
                cameraYaw=args.cam_yaw,
                cameraPitch=args.cam_pitch,
                cameraTargetPosition=[
                    args.cam_target_x,
                    args.cam_target_y,
                    args.cam_target_z,
                ],
            )

        logger.info("Loaded URDF: %s", urdf_path)
        logger.info("body_id=%s num_joints=%d", hand_id, p.getNumJoints(hand_id))
        logger.debug(
            "Camera: dist=%.3f yaw=%.1f pitch=%.1f target=(%.3f, %.3f, %.3f)",
            args.cam_distance,
            args.cam_yaw,
            args.cam_pitch,
            args.cam_target_x,
            args.cam_target_y,
            args.cam_target_z,
        )

        movable = []
        for joint_idx in range(p.getNumJoints(hand_id)):
            info = p.getJointInfo(hand_id, joint_idx)
            joint_type = info[2]
            if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                name = info[1].decode("utf-8")
                lower = info[8]
                upper = info[9]
                max_force = info[10]
                max_velocity = info[11]
                movable.append((joint_idx, name, lower, upper, max_force, max_velocity))

        logger.info("Movable joints (%d):", len(movable))
        for joint in movable:
            idx, name, lower, upper, max_force, max_velocity = joint
            logger.info(
                "idx=%02d name=%-20s range=[%+.3f, %+.3f] maxF=%.3f maxV=%.3f",
                idx,
                name,
                lower,
                upper,
                max_force,
                max_velocity,
            )

        logger.info("Simulation loop running for %.2f seconds", max(args.duration, 0.0))
        end_time = time.time() + max(args.duration, 0.0)
        while time.time() < end_time:
            p.stepSimulation()
            if not args.headless:
                time.sleep(1.0 / 240.0)

        logger.info("Simulation loop completed")
        return 0
    finally:
        p.disconnect()
        logger.info("PyBullet disconnected")


if __name__ == "__main__":
    sys.exit(main())
