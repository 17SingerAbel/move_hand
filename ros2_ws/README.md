# ROS2 Minimal Bridge (`hand_bridge`)

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
