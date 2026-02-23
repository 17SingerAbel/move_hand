# move_hand

Linker Hand 06 Lite 在 macOS 上的最小仿真控制 demo（PyBullet）。

## 1) 创建 conda 环境
```bash
cd /Users/abelsang/yaki-robotics/move_hand
conda create -n linkhand python=3.10 -y
conda activate linkhand
pip install -r requirements.txt
```

如果你怀疑没有真正进入 conda 环境，先检查：
```bash
which python
which pip
python -V
```

`python` 和 `pip` 应该指向 `.../envs/linkhand/...`，不应该是 `/Library/Frameworks/...`。

最稳妥方式是直接用 `conda run`（不会被 shell 别名干扰）：
```bash
conda run -n linkhand python -m pip install --upgrade pip
conda run -n linkhand python -m pip install -r requirements.txt
```

## 2) 拉取官方 URDF（你之前还没 clone，这步必须先做）
```bash
git clone https://github.com/linker-bot/linkerhand-urdf external/linkerhand-urdf
```

查看仓库里有哪些 URDF 文件：
```bash
find external/linkerhand-urdf -name "*.urdf"
```

## 3) 加载模型并打印可动关节
把下面命令里的 `<URDF绝对路径>` 换成你上一步 `find` 找到的某个文件：

```bash
python sim/sim_load.py --urdf <URDF绝对路径>
```

示例（仅示意，实际以你的 `find` 结果为准）：
```bash
python sim/sim_load.py --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/xxx.urdf
```

## 4) 单关节控制 demo
```bash
python sim/demo_single_joint.py --urdf <URDF绝对路径> --joint 0 --cycles 10
```

## 5) 常用说明
- `--joint` 不是 URDF 原始 joint id，而是 `sim_load.py` 打印出来的“可动关节列表索引”。
- 控制命令会按 URDF 限位自动裁剪（避免越界）。
- 如果依赖安装失败，优先检查网络与 conda/pip 镜像源配置。
- 如果看到系统 Python 路径（如 `/Library/Frameworks/...`），说明当前不是目标 conda 环境，请改用 `conda run -n linkhand ...`。

## 6) 视角太远时怎么调
脚本默认已经拉近镜头；如果你还觉得远，可以手动调：

```bash
python sim/sim_load.py --urdf <URDF绝对路径> --cam-distance 0.2 --cam-yaw 140 --cam-pitch -15 --cam-target-z 0.08
```

`demo_single_joint.py` 也支持同样的相机参数。

## 7) 日志与健康判定
两个脚本都支持：
- `--log-level`：`DEBUG|INFO|WARNING|ERROR`
- `--log-file`：写入日志文件

示例：
```bash
python sim/sim_load.py --urdf <URDF绝对路径> --log-level DEBUG --log-file logs/sim_load.log
python sim/demo_single_joint.py --urdf <URDF绝对路径> --joint 0 --cycles 10 --log-level DEBUG --log-file logs/demo_joint.log
```

如果你在多 Python 环境下，建议直接用：
```bash
conda run -n linkhand python sim/sim_load.py --urdf <URDF绝对路径> --log-level DEBUG --log-file logs/sim_load.log
conda run -n linkhand python sim/demo_single_joint.py --urdf <URDF绝对路径> --joint 0 --cycles 10 --log-level DEBUG --log-file logs/demo_joint.log
```

建议你判断“运行正常”看这几个信号：
- 出现 `Loaded URDF` 和 `Movable joints`。
- 出现 `Simulation loop completed`（sim_load）。
- 连续出现 `[CYCLE xx] target/applied/current`（demo_single_joint）。
- 没有 `ERROR` 日志。

如果看到 `Command clipped`（WARNING），表示命令超出关节限位，系统已自动裁剪，这通常是可接受的保护行为。

## 8) 整手向量控制（Phase 3）
先确认依赖包含 `PyYAML`：
```bash
pip install -r requirements.txt
```

运行整手姿态序列 demo：
```bash
python sim/demo_all_joints.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --sequence open,half_close,close,half_close,open \
  --cycles 2 \
  --log-level DEBUG \
  --log-file logs/demo_all_joints.log
```

参数说明（`demo_all_joints.py`）：

| 参数 | 作用 | 默认值 | 备注 |
|---|---|---|---|
| `--urdf` | 机械手 URDF 路径 | 无（必填） | 决定关节数量和关节限位 |
| `--poses` | 姿态配置 YAML 路径 | `config/poses.yaml` | 文件中是 `pose_name -> 关节向量` |
| `--sequence` | 姿态执行顺序 | `open,half_close,close,half_close,open` | 用逗号分隔 |
| `--cycles` | 序列重复次数 | `2` | 每轮都会走完整个 sequence |
| `--hold` | 每个姿态停留秒数 | `0.8` | 越大动作越慢、越容易观察 |
| `--headless` | 无 GUI 运行 | `False` | 调试时通常不加这个参数 |
| `--cam-distance` | 相机距离 | `0.4` | 仅 GUI 模式生效 |
| `--cam-yaw` | 相机水平角 | `90` | 仅 GUI 模式生效 |
| `--cam-pitch` | 相机俯仰角 | `-15` | 仅 GUI 模式生效 |
| `--cam-target-x/y/z` | 相机目标点 | `0/0/0.05` | 仅 GUI 模式生效 |
| `--log-level` | 日志级别 | `INFO` | 建议调试时用 `DEBUG` |
| `--log-file` | 日志文件路径 | 不写文件 | 例如 `logs/demo_all_joints.log` |

最小可用命令（只保留必要参数）：
```bash
python sim/demo_all_joints.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf
```

常用调试命令（含详细日志）：
```bash
python sim/demo_all_joints.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --sequence open,half_close,close,half_close,open \
  --cycles 2 \
  --log-level DEBUG \
  --log-file logs/demo_all_joints.log
```

多环境下推荐强制指定：
```bash
conda run -n linkhand python sim/demo_all_joints.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --poses config/poses.yaml \
  --sequence open,half_close,close,half_close,open \
  --cycles 2 \
  --log-level DEBUG \
  --log-file logs/demo_all_joints.log
```

说明：
- `config/poses.yaml` 目前是占位空列表，脚本会自动生成默认姿态（open/half_close/close）。
- 你后续可以把每个姿态改成“按关节的实际向量”，长度必须等于可动关节数。

## 9) 中指手势 demo（客户指定）
需求：中指保持伸直，其他手指最大弯曲。

运行：
```bash
python sim/demo_middle_finger.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --hold 3 \
  --log-level DEBUG \
  --log-file logs/demo_middle_finger.log
```

参数说明：
- `--middle-ratio`：中指弯曲比例，默认 `0.0`（完全伸直）。
- `--others-ratio`：其他关节弯曲比例，默认 `1.0`（最大弯曲）。

可微调示例（中指稍微弯一点）：
```bash
python sim/demo_middle_finger.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --middle-ratio 0.1 \
  --others-ratio 1.0
```

## 10) 双按键交互控制（中指/复原）
运行：
```bash
python sim/demo_key_control.py \
  --urdf /Users/abelsang/yaki-robotics/move_hand/external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf \
  --log-level INFO \
  --log-file logs/demo_key_control.log
```

键位：
- `M`：中指手势（中指伸直，其他手指弯曲）
- `O`：恢复到脚本启动时捕获的“开机初始姿态”
- `Q`：退出脚本

说明：
- 该脚本需要 GUI（用于键盘事件），不适用于 `--headless`。
