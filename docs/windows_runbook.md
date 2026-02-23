# Windows Runbook (Final Runtime)

本手册对应最终 Windows 原生运行。Docker 仅用于 ROS2 联调测试。

## 1) 获取代码
```powershell
cd <your-parent-dir>
git clone <your-repo-url> move_hand
cd move_hand
```

## 2) Python 环境
```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 3) 仿真最小验证
```powershell
python sim\sim_load.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf
```

成功信号：
- 打印 `Movable joints` 列表
- 打印 `Simulation loop completed`

## 4) 交互控制（客户演示入口）
```powershell
python sim\demo_key_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml
```

键位：
- `M`: 中指手势
- `O`: 恢复 home
- `Q`: 退出

## 5) 回归测试
```powershell
python sim\test_middle_home_cycle.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml --cycles 100
```

成功信号：
- 输出 `PASS test_middle_home_cycle ...`

## 6) CLI 控制
```powershell
python tools\cli_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml pose middle
python tools\cli_control.py --urdf external\linkerhand-urdf\o6\right\linkerhand_o6_right.urdf --poses config\poses.yaml pose home
```

## 7) ROS2（Windows 原生，后续真机阶段）
当前仓库已提供最小桥接骨架 `ros2_ws/src/hand_bridge`：
- 订阅 `/hand/command` (`std_msgs/msg/Float64MultiArray`)
- 发布 `/hand/state` (`sensor_msgs/msg/JointState`)
- 健康 `/hand/health` (`std_msgs/msg/String`)

建议先固定接口约定，再替换底层驱动为真机 SDK。

## 8) Windows 联合验证（多人协作）
推荐两人分工：
- `Operator A`：负责启动节点与观察日志。
- `Operator B`：负责发送命令并观察 topic 输出。

### 步骤
1. `Operator A` 启动节点（Windows 原生 ROS2）：
```powershell
cd ros2_ws
colcon build --packages-select hand_bridge
call install\\setup.bat
ros2 run hand_bridge sim_driver_node --ros-args -p urdf:=../external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf -p headless:=true
```
2. `Operator B` 发送合法命令：
```powershell
ros2 topic pub --once /hand/command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
3. `Operator B` 发送非法命令（长度错误）：
```powershell
ros2 topic pub --once /hand/command std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```
4. `Operator B` 检查状态与健康：
```powershell
ros2 topic echo --once /hand/state
ros2 topic echo --once /hand/health
```

### 通过标准
- `/hand/state` 有 11 个关节名和位置数据。
- `/hand/health` 里 `bad_length` 至少为 1（说明非法命令被识别并统计）。
- 节点不中断，可继续接收后续合法命令。

## 9) 约束（必须遵守）
- 最终客户运行：Windows 原生。
- Docker：仅用于开发机上 ROS2 联调测试。
