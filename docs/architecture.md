# Architecture and Data Flow

## Components
- `sim/joint_controller.py`: PyBullet 封装层（关节控制、仿真步进、状态读取）。
- `sim/*demo*.py`: 不同入口（加载、单关节、多关节、中指、按键）。
- `tools/cli_control.py`: 命令式入口（适合集成与脚本化调用）。
- `ros2_ws/src/hand_bridge/hand_bridge/sim_driver_node.py`: ROS2 到 PyBullet 的桥。
- `config/poses.yaml`: 姿态向量源（home/open/half_close/close）。

## Data Flow 1: Keyboard/CLI -> PyBullet
1. 用户输入（Key / CLI）。
2. 目标 pose 向量生成或加载（11 joints）。
3. 插值（smooth transition）。
4. `set_all_joints` + `stepSimulation`。
5. 读取 joint state 反馈日志。

## Data Flow 2: ROS2 -> PyBullet
1. 上游节点发布 `/hand/command`。
2. `sim_driver_node` 校验向量长度与类型。
3. `JointController` 应用目标到仿真。
4. 节点定频发布 `/hand/state`。
5. 上游节点消费状态闭环。

## Runtime Modes
- Windows native (final delivery): Python + PyBullet (+ optional ROS2 native)
- Docker (dev only): ROS2 integration testing
