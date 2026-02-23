#!/usr/bin/env python3
"""Minimal ROS2 sim driver node for Linker Hand."""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _import_joint_controller():
    repo_root = Path(__file__).resolve().parents[4]
    sim_dir = repo_root / "sim"
    if str(sim_dir) not in sys.path:
        sys.path.insert(0, str(sim_dir))
    from joint_controller import JointController

    return JointController


class SimDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_driver_node")

        self.declare_parameter(
            "urdf",
            "external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf",
        )
        self.declare_parameter("headless", True)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("sim_steps_per_cycle", 2)
        self.declare_parameter("max_force", 2.0)

        urdf_param = self.get_parameter("urdf").get_parameter_value().string_value
        self._headless = bool(self.get_parameter("headless").value)
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._sim_steps_per_cycle = int(self.get_parameter("sim_steps_per_cycle").value)
        self._max_force = float(self.get_parameter("max_force").value)

        JointController = _import_joint_controller()
        self._ctrl = JointController(urdf_path=urdf_param, gui=not self._headless)
        self._joint_names = [j.name for j in self._ctrl.joints]
        self._target: Optional[list[float]] = None

        self._sub = self.create_subscription(
            Float64MultiArray,
            "/hand/command",
            self._on_command,
            10,
        )
        self._pub = self.create_publisher(JointState, "/hand/state", 10)

        period_s = 1.0 / max(1e-3, self._publish_rate_hz)
        self._timer = self.create_timer(period_s, self._on_timer)
        self.get_logger().info(
            "sim_driver_node ready | urdf=%s joints=%d publish_rate=%.1fHz",
            urdf_param,
            len(self._joint_names),
            self._publish_rate_hz,
        )

    def _on_command(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self._joint_names):
            self.get_logger().warning(
                "Command length mismatch: expected %d got %d",
                len(self._joint_names),
                len(msg.data),
            )
            return
        self._target = [float(v) for v in msg.data]

    def _on_timer(self) -> None:
        if self._target is not None:
            try:
                self._ctrl.set_all_joints(self._target, max_force=self._max_force)
            except Exception as exc:
                self.get_logger().error("Failed to apply command: %s", str(exc))

        self._ctrl.step(steps=max(1, self._sim_steps_per_cycle))
        positions = self._ctrl.get_joint_positions()

        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = self._joint_names
        state.position = positions
        self._pub.publish(state)

    def destroy_node(self) -> bool:
        try:
            self._ctrl.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SimDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
