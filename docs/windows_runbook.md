# Windows Runbook (Final Runtime)

本文档用于最终 Windows 运行，Docker 仅用于 ROS2 测试，不作为交付运行环境。

## 1) 代码准备
```powershell
cd C:\work
git clone <your-repo-url> move_hand
cd move_hand
```

## 2) Python 环境（Windows）
```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 3) 仿真快速验证（Windows 原生）
```powershell
python sim\sim_load.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf
```

## 4) 交互控制（Windows 原生）
```powershell
python sim\demo_key_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml
```

键位：
- `M`：中指手势
- `O`：恢复 home
- `Q`：退出

## 5) 自动回归（Windows 原生）
```powershell
python sim\test_middle_home_cycle.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml --cycles 100
```

期望输出：
- `PASS test_middle_home_cycle ...`

## 6) 最小 CLI（Windows 原生）
```powershell
python tools\cli_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml pose middle
python tools\cli_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml pose home
```

## 7) ROS2（Windows 原生，后续真机阶段）
如果要在 Windows 原生运行 ROS2，请安装官方 ROS2（建议 Humble/Jazzy，对应你的环境），然后在项目 `ros2_ws` 下进行 `colcon build`。

当前仓库提供的是最小适配骨架：
- 订阅：`/hand/command` (`std_msgs/msg/Float64MultiArray`)
- 发布：`/hand/state` (`sensor_msgs/msg/JointState`)

## 8) 重要约束
- 最终客户运行平台：Windows 原生。
- Docker：仅用于开发机上的 ROS2 联调测试。
