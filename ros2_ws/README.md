# ROS2 Minimal Bridge (`hand_bridge`)

这个目录是 ROS2 适配骨架。最终客户运行平台是 Windows 原生；这里的 Docker 仅用于 ROS2 联调测试。

## 包与节点
- 包：`ros2_ws/src/hand_bridge`
- 节点：`sim_driver_node`
- 输入 topic：`/hand/command` (`std_msgs/msg/Float64MultiArray`)
- 输出 topic：`/hand/state` (`sensor_msgs/msg/JointState`)
- 健康 topic：`/hand/health` (`std_msgs/msg/String`)

接口细节见：`../docs/ros2_interface.md`

## A) 本机 ROS2（Linux/macOS）运行

### 1) 构建
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hand_bridge
source install/setup.bash
```

### 2) 启动节点
注意：下面示例默认你当前目录是 `ros2_ws`，因此 `urdf` 用相对路径 `../external/...`。

```bash
ros2 run hand_bridge sim_driver_node \
  --ros-args \
  -p urdf:=../external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  -p headless:=true \
  -p publish_rate_hz:=50.0
```

### 3) 发送命令
```bash
ros2 topic pub --once /hand/command std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 4) 查看状态
```bash
ros2 topic echo /hand/state
```

## B) Docker（仅联调测试）

### 一键测试（推荐）
```bash
bash ros2_ws/scripts/docker_smoke_test.sh
```

该脚本会自动完成：
- Docker build
- `colcon build`
- 启动 `sim_driver_node`
- 发布合法/非法 command
- 校验 `/hand/state` 与 `/hand/health`

### 1) 构建镜像
```bash
cd ros2_ws/docker
docker compose build
```

### 2) 进入容器并构建包
```bash
docker compose run --rm ros2-test bash -lc "
  cd /ws/ros2_ws && \
  source /opt/ros/humble/setup.bash && \
  colcon build --packages-select hand_bridge
"
```

### 3) 启动节点（容器内）
```bash
docker compose run --rm ros2-test bash -lc "
  cd /ws/ros2_ws && \
  source /opt/ros/humble/setup.bash && \
  source install/setup.bash && \
  ros2 run hand_bridge sim_driver_node \
    --ros-args \
    -p urdf:=../external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
    -p headless:=true
"
```
