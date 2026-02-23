# move_hand

Linker Hand 06 Lite 最小可交付控制工程（PyBullet 仿真 + 可选 ROS2 适配）。

适用对象：不懂 ROS，也要先把手模型跑起来的人。

## 你先看哪里
- `README.md`（本文件）：怎么安装、怎么运行、怎么自检。
- `plan.md`：项目计划与阶段状态（管理视角）。
- `docs/`：代码相关解释。
  - `docs/architecture.md`：系统结构和数据流。
  - `docs/ros2_interface.md`：ROS2 topic 契约（接口定义）。
  - `docs/windows_runbook.md`：Windows 最终运行手册。
  - `docs/runtime_and_tests.md`：为什么用 Docker/ROS2，以及每个测试在验证什么。

## 当前状态（结论）
- 仿真主链路已可用：模型加载、按键控制、CLI 控制、回归测试。
- ROS2 已有最小桥接骨架（`ros2_ws/src/hand_bridge`），用于联调。
- 最终交付目标是 Windows 原生运行；Docker 仅用于 ROS2 联调测试。

## 目录速览
```text
move_hand/
├─ sim/                            # 仿真与控制核心
├─ tools/                          # CLI 与自检脚本
├─ config/                         # 姿态配置（poses.yaml）
├─ ros2_ws/                        # ROS2 最小桥接
├─ docs/                           # 架构/接口/Windows 运行说明
├─ plan.md                         # 项目计划
└─ logs/                           # 日志与测试产物
```

## 快速开始（不需要 ROS）

### 1) 创建并激活 Python 环境
推荐 Python 3.10+。

```bash
cd move_hand
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

如果你用 Conda，也可以：
```bash
conda create -n linkhand python=3.10 -y
conda activate linkhand
python -m pip install -r requirements.txt
```

### 2) 一键自检（推荐先跑）
```bash
python tools/self_check.py
```

通过信号：最后输出 `SELF_CHECK PASS`。

### 3) 仅加载模型（最小验证）
```bash
python sim/sim_load.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf
```

无界面环境（服务器/CI）可用：
```bash
python sim/sim_load.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf --headless --duration 1.0
```

### 4) 按键交互（演示入口）
```bash
python sim/demo_key_control.py \
  --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --transition-time 0.4 \
  --transition-steps 40
```

键位：
- `M`：中指手势
- `O`：恢复 `home`
- `Q`：退出

### 5) CLI 控制（脚本化调用）
```bash
python tools/cli_control.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf --poses config/poses.yaml pose middle
python tools/cli_control.py --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf --poses config/poses.yaml pose home
```

### 6) 回归测试
```bash
python sim/test_middle_home_cycle.py \
  --urdf external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --cycles 100 \
  --transition-steps 40 \
  --sim-steps-per-waypoint 6 \
  --settle-steps 30
```

通过信号：输出 `PASS test_middle_home_cycle ...`。

## ROS2（可选，不影响你先跑仿真）

ROS2 相关只在你需要对接上层系统时再看：
- `ros2_ws/README.md`
- `docs/ros2_interface.md`

最小桥接节点：
- 订阅 `/hand/command` (`std_msgs/msg/Float64MultiArray`)
- 发布 `/hand/state` (`sensor_msgs/msg/JointState`)
- 发布 `/hand/health` (`std_msgs/msg/String`)

Docker 一键联调测试：
```bash
bash ros2_ws/scripts/docker_smoke_test.sh
```

## 常见问题
- 报错 `No module named pybullet`：
  - 说明依赖未安装到当前环境。
  - 处理：确认环境已激活后执行 `python -m pip install -r requirements.txt`。
- URDF 找不到：
  - 检查路径是否存在：`external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf`。
- 你只想验证代码结构，不想实际跑仿真：
  - 执行 `python tools/self_check.py --skip-runtime`。

## 平台约束
- 最终客户运行：Windows 原生（见 `docs/windows_runbook.md`）。
- Docker：仅用于 ROS2 联调测试，不作为最终交付运行方式。
