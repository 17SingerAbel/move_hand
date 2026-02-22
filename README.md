# move_hand

Linker Hand 06 Lite 在 macOS 上的最小仿真控制 demo（PyBullet）。

## 1) 创建 conda 环境
```bash
cd /Users/abelsang/yaki-robotics/move_hand
conda create -n linkhand python=3.10 -y
conda activate linkhand
pip install -r requirements.txt
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

说明：
- `config/poses.yaml` 目前是占位空列表，脚本会自动生成默认姿态（open/half_close/close）。
- 你后续可以把每个姿态改成“按关节的实际向量”，长度必须等于可动关节数。
