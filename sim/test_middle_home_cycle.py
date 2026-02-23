#!/usr/bin/env python3
"""Headless regression: cycle middle-finger pose and home pose repeatedly."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from joint_controller import JointController


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Regression test for middle/home pose cycling")
    parser.add_argument("--urdf", required=True, help="Path to hand URDF")
    parser.add_argument("--poses", default="config/poses.yaml", help="Path to poses YAML (must include 'home')")
    parser.add_argument("--cycles", type=int, default=100, help="Number of M->O cycles")
    parser.add_argument("--transition-steps", type=int, default=40, help="Interpolation steps per transition")
    parser.add_argument("--sim-steps-per-waypoint", type=int, default=6, help="Simulation steps for each waypoint")
    parser.add_argument("--settle-steps", type=int, default=30, help="Extra settle steps after each transition")
    parser.add_argument("--middle-ratio", type=float, default=0.0)
    parser.add_argument("--others-ratio", type=float, default=1.0)
    parser.add_argument("--final-error-threshold", type=float, default=0.05, help="Max allowed final abs error")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity",
    )
    return parser.parse_args()


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


def load_home_pose(path: Path) -> list[float]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("Missing dependency: pyyaml. Run: pip install -r requirements.txt") from exc

    if not path.exists():
        raise FileNotFoundError(f"Pose file not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict) or "home" not in data:
        raise ValueError("Pose file must include key 'home'")
    home = data["home"]
    if not isinstance(home, list):
        raise ValueError("Pose 'home' must be a list")
    return [float(v) for v in home]


def clip_value(value: float, lower: float, upper: float) -> float:
    if lower <= upper:
        return max(lower, min(upper, value))
    return value


def smooth_move_with_stats(
    ctrl: JointController,
    target: list[float],
    steps: int,
    sim_steps_per_waypoint: int,
    clip_tolerance: float = 1e-8,
) -> tuple[int, float]:
    start = ctrl.get_joint_positions()
    joint_count = len(ctrl.joints)
    if len(target) != joint_count:
        raise ValueError(f"Target length mismatch: expected {joint_count} got {len(target)}")

    steps = max(1, int(steps))
    clip_events = 0
    max_clip_delta = 0.0
    for i in range(1, steps + 1):
        alpha = i / steps
        waypoint: list[float] = []
        for j, joint in enumerate(ctrl.joints):
            requested = start[j] + alpha * (target[j] - start[j])
            clipped = clip_value(requested, joint.lower, joint.upper)
            delta = abs(clipped - requested)
            if delta > clip_tolerance:
                clip_events += 1
                if delta > max_clip_delta:
                    max_clip_delta = delta
            waypoint.append(requested)
        ctrl.set_all_joints(waypoint)
        ctrl.step(steps=max(1, int(sim_steps_per_waypoint)))
    return clip_events, max_clip_delta


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    logger = logging.getLogger("test_middle_home_cycle")
    logging.getLogger("joint_controller").setLevel(logging.ERROR)

    ctrl = JointController(args.urdf, gui=False, logger=logging.getLogger("joint_controller"))
    try:
        if not ctrl.joints:
            logger.error("No movable joints found")
            return 2

        home_pose = load_home_pose(Path(args.poses))
        if len(home_pose) != len(ctrl.joints):
            logger.error("home length mismatch: expected %d got %d", len(ctrl.joints), len(home_pose))
            return 2
        middle_pose = build_middle_finger_pose(ctrl, args.middle_ratio, args.others_ratio)

        total_clip_events = 0
        max_clip_delta = 0.0
        for _ in range(max(1, args.cycles)):
            clip_count, clip_delta = smooth_move_with_stats(
                ctrl,
                middle_pose,
                args.transition_steps,
                args.sim_steps_per_waypoint,
            )
            total_clip_events += clip_count
            max_clip_delta = max(max_clip_delta, clip_delta)
            ctrl.step(steps=max(1, int(args.settle_steps)))

            clip_count, clip_delta = smooth_move_with_stats(
                ctrl,
                home_pose,
                args.transition_steps,
                args.sim_steps_per_waypoint,
            )
            total_clip_events += clip_count
            max_clip_delta = max(max_clip_delta, clip_delta)
            ctrl.step(steps=max(1, int(args.settle_steps)))

        current = ctrl.get_joint_positions()
        final_max_abs_error = max(abs(current[i] - home_pose[i]) for i in range(len(home_pose)))
        passed = total_clip_events == 0 and final_max_abs_error <= args.final_error_threshold

        logger.info(
            "cycles=%d transitions=%d transition_steps=%d clip_events=%d max_clip_delta=%.6f final_max_abs_error=%.6f",
            args.cycles,
            args.cycles * 2,
            args.transition_steps,
            total_clip_events,
            max_clip_delta,
            final_max_abs_error,
        )
        if passed:
            print(
                "PASS test_middle_home_cycle "
                f"cycles={args.cycles} clip_events={total_clip_events} "
                f"max_clip_delta={max_clip_delta:.6f} final_max_abs_error={final_max_abs_error:.6f}"
            )
            return 0

        print(
            "FAIL test_middle_home_cycle "
            f"cycles={args.cycles} clip_events={total_clip_events} "
            f"max_clip_delta={max_clip_delta:.6f} final_max_abs_error={final_max_abs_error:.6f}"
        )
        return 2
    finally:
        ctrl.close()


if __name__ == "__main__":
    sys.exit(main())
