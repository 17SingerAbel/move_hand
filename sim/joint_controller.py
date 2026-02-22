#!/usr/bin/env python3
"""Minimal joint-level controller for Linker Hand in PyBullet."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


@dataclass
class JointLimit:
    index: int
    name: str
    lower: float
    upper: float


class JointController:
    def __init__(self, urdf_path: str, gui: bool = True, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        try:
            import pybullet as p
            import pybullet_data
        except ImportError as exc:
            raise RuntimeError(
                "pybullet not installed. Run: linkhand/bin/pip install -r requirements.txt"
            ) from exc

        self.p = p
        self._client = p.connect(p.GUI if gui else p.DIRECT)
        if self._client < 0:
            raise RuntimeError("Failed to connect to PyBullet")

        path = Path(urdf_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"URDF not found: {path}")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.hand_id = p.loadURDF(str(path), useFixedBase=True)
        self.joints = self._collect_joint_limits()
        self.logger.info(
            "JointController ready: gui=%s hand_id=%s movable_joints=%d",
            gui,
            self.hand_id,
            len(self.joints),
        )

    def _collect_joint_limits(self) -> List[JointLimit]:
        limits: List[JointLimit] = []
        for i in range(self.p.getNumJoints(self.hand_id)):
            info = self.p.getJointInfo(self.hand_id, i)
            joint_type = info[2]
            if joint_type in (self.p.JOINT_REVOLUTE, self.p.JOINT_PRISMATIC):
                limits.append(
                    JointLimit(
                        index=i,
                        name=info[1].decode("utf-8"),
                        lower=float(info[8]),
                        upper=float(info[9]),
                    )
                )
        return limits

    def _clip(self, joint: JointLimit, target: float) -> float:
        # URDF may leave limits open; if not finite treat target as-is.
        if joint.lower <= joint.upper:
            clipped = max(joint.lower, min(joint.upper, target))
            if clipped != target:
                self.logger.warning(
                    "Command clipped: joint=%s requested=%.4f clipped=%.4f range=[%.4f, %.4f]",
                    joint.name,
                    target,
                    clipped,
                    joint.lower,
                    joint.upper,
                )
            return clipped
        return target

    def set_joint(self, joint_list_index: int, target_rad: float, max_force: float = 2.0) -> float:
        if joint_list_index < 0 or joint_list_index >= len(self.joints):
            raise IndexError(f"joint_list_index out of range: {joint_list_index}")

        joint = self.joints[joint_list_index]
        clipped = self._clip(joint, float(target_rad))
        self.p.setJointMotorControl2(
            bodyUniqueId=self.hand_id,
            jointIndex=joint.index,
            controlMode=self.p.POSITION_CONTROL,
            targetPosition=clipped,
            force=max_force,
        )
        self.logger.debug(
            "set_joint: list_idx=%d urdf_idx=%d name=%s target=%.4f applied=%.4f force=%.3f",
            joint_list_index,
            joint.index,
            joint.name,
            target_rad,
            clipped,
            max_force,
        )
        return clipped

    def set_all_joints(self, targets: List[float], max_force: float = 2.0) -> List[float]:
        if len(targets) != len(self.joints):
            raise ValueError(f"Expected {len(self.joints)} targets, got {len(targets)}")

        applied = []
        for i, target in enumerate(targets):
            applied.append(self.set_joint(i, float(target), max_force=max_force))
        return applied

    def get_joint_positions(self) -> List[float]:
        states = []
        for joint in self.joints:
            states.append(float(self.p.getJointState(self.hand_id, joint.index)[0]))
        return states

    def step(self, steps: int = 1) -> None:
        for _ in range(max(1, steps)):
            self.p.stepSimulation()

    def close(self) -> None:
        self.p.disconnect()
        self.logger.info("PyBullet disconnected")
