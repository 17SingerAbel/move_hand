#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKER_DIR="${ROS2_WS_DIR}/docker"

echo "[1/3] Building docker image..."
(cd "${DOCKER_DIR}" && docker compose build)

echo "[2/3] Running ROS2 smoke test in container..."
(cd "${DOCKER_DIR}" && docker compose run --rm ros2-test bash -lc "
  set -euo pipefail
  cd /ws/ros2_ws
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  rm -rf build install log
  colcon build --packages-select hand_bridge
  set +u
  source install/setup.bash
  set -u

  ros2 run hand_bridge sim_driver_node --ros-args \
    -p urdf:=/ws/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
    -p headless:=true \
    -p publish_rate_hz:=20.0 > /tmp/sim_driver.log 2>&1 &
  NODE_PID=\$!
  sleep 2

  ros2 topic pub --once /hand/command std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
  ros2 topic pub --once /hand/command std_msgs/msg/Float64MultiArray \
    '{data: [0.0, 0.0]}'

  timeout 5s ros2 topic echo --once /hand/state > /tmp/hand_state.log
  timeout 5s ros2 topic echo --once /hand/health > /tmp/hand_health.log

  kill \$NODE_PID || true
  sleep 1

  grep -q 'name:' /tmp/hand_state.log
  grep -Eq 'bad_length=[1-9][0-9]*' /tmp/hand_health.log

  echo '--- sim_driver.log ---'
  tail -n 5 /tmp/sim_driver.log || true
  echo '--- hand_health.log ---'
  cat /tmp/hand_health.log
")

echo "[3/3] PASS: Docker ROS2 smoke test completed."
