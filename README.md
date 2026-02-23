# move_hand

Linker Hand 06 Lite 的最小可交付控制工程（PyBullet 仿真 + 可选 ROS2 适配）。

## 当前状态
- Phase 1-4 已落地：仿真加载、单关节、多关节、按键控制、CLI、回归测试都可用。
- Phase 5 已有最小 ROS2 骨架：`sim_driver_node` 可订阅 `/hand/command` 并发布 `/hand/state`。
- 最终目标平台是 Windows 原生；Docker 仅用于 ROS2 联调测试。

## 代码结构
```text
move_hand/
├─ sim/
│  ├─ joint_controller.py          # PyBullet 控制核心（set_joint/set_all_joints/step/state）
│  ├─ sim_load.py                  # 加载 URDF + 打印可动关节
│  ├─ demo_single_joint.py         # 单关节往返 demo
│  ├─ demo_all_joints.py           # 姿态序列 demo（读取 config/poses.yaml）
│  ├─ demo_middle_finger.py        # 中指手势 demo
│  ├─ demo_key_control.py          # 键盘交互：M=middle, O=home, Q=quit（平滑过渡）
│  └─ test_middle_home_cycle.py    # 回归测试：M->O 循环
├─ tools/
│  └─ cli_control.py               # 最小 CLI：pose middle/home/open/close...
├─ config/
│  └─ poses.yaml                   # home/open/half_close/close 姿态向量
├─ ros2_ws/
│  ├─ src/hand_bridge/             # ROS2 最小桥接包（ament_python）
│  ├─ docker/                      # ROS2 测试容器（仅测试）
│  └─ README.md                    # ROS2 使用说明
├─ docs/
│  ├─ windows_runbook.md           # Windows 最终运行手册
│  ├─ architecture.md              # 系统架构与数据流
│  └─ ros2_interface.md            # ROS2 接口约定（contract）
└─ logs/                           # 运行日志与测试产物
```

## 数据流（Data Flow）

### 1) 纯仿真（不经过 ROS2）
1. 用户输入（键盘或 CLI）
2. 目标姿态向量（11 维）
3. `JointController.set_all_joints(...)`
4. PyBullet step simulation
5. 读取 `get_joint_positions()` 作为状态反馈

### 2) ROS2 仿真桥接
1. 上游节点发布 `/hand/command` (`std_msgs/Float64MultiArray`)
2. `sim_driver_node` 订阅并校验长度（必须 = 11）
3. `JointController` 应用到 PyBullet
4. `sim_driver_node` 周期发布 `/hand/state` (`sensor_msgs/JointState`)
5. 上游节点读取状态闭环

## 快速开始（仿真）

### 1) 创建环境
```bash
cd move_hand
conda create -n linkhand python=3.10 -y
conda activate linkhand
pip install -r requirements.txt
```

### 2) 加载模型
```bash
python sim/sim_load.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf
```

### 3) 按键控制（推荐演示入口）
```bash
python sim/demo_key_control.py \
  --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --transition-time 0.4 \
  --transition-steps 40
```

键位：
- `M`: 中指手势
- `O`: 恢复 `home`
- `Q`: 退出

### 4) CLI 控制
```bash
python tools/cli_control.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf --poses config/poses.yaml pose middle
python tools/cli_control.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf --poses config/poses.yaml pose home
```

### 5) 回归测试
```bash
python sim/test_middle_home_cycle.py \
  --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --cycles 100 \
  --transition-steps 40 \
  --sim-steps-per-waypoint 6 \
  --settle-steps 30
```

成功信号：输出 `PASS test_middle_home_cycle ...`

## ROS2 最小骨架
- 包路径：`ros2_ws/src/hand_bridge`
- 节点：`sim_driver_node`
- 订阅：`/hand/command` (`std_msgs/msg/Float64MultiArray`)
- 发布：`/hand/state` (`sensor_msgs/msg/JointState`)
- 健康：`/hand/health` (`std_msgs/msg/String`)

详细见：
- `ros2_ws/README.md`
- `docs/ros2_interface.md`

一键 ROS2 联调测试（Docker）：
```bash
bash ros2_ws/scripts/docker_smoke_test.sh
```

## 平台说明
- Windows 最终运行：`docs/windows_runbook.md`
- Docker：仅用于 ROS2 联调测试，不作为最终交付运行方式。
