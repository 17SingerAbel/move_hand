# ROS2 Minimal Bridge (`hand_bridge`)

说明：
- 最终交付运行平台是 Windows 原生。
- 本目录下 Docker 仅用于 ROS2 联调测试。

## 1) 构建
在已安装 ROS2 的终端执行：

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

## 2) 启动仿真驱动节点
```bash
ros2 run hand_bridge sim_driver_node \
  --ros-args \
  -p urdf:=external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  -p headless:=true \
  -p publish_rate_hz:=50.0
```

## 3) 发布命令（/hand/command）
`/hand/command` 类型：`std_msgs/msg/Float64MultiArray`

```bash
ros2 topic pub /hand/command std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 4) 读取状态（/hand/state）
`/hand/state` 类型：`sensor_msgs/msg/JointState`

```bash
ros2 topic echo /hand/state
```

## 5) Docker 方式（仅 ROS2 测试）
```bash
cd ros2_ws/docker
docker compose build
docker compose run --rm ros2-test
```

容器内执行：
```bash
cd /ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run hand_bridge sim_driver_node --ros-args -p urdf:=external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf -p headless:=true
```

另开一个容器终端（或新 shell）测试 topic：
```bash
cd /ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /hand/command std_msgs/msg/Float64MultiArray "{data: [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}"
ros2 topic echo /hand/state
```
