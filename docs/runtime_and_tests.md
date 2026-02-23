# Runtime Modes and Test Guide

这份文档回答三个管理层常见问题：
1. 为什么需要 Docker？
2. ROS2 在这个项目里到底做什么？
3. 每个测试在验证什么、通过标准是什么？

## 1) 为什么需要 Docker

短答案：为了让 ROS2 联调测试环境可复制、可对齐、少踩坑。

详细原因：
- 统一 ROS2 版本与依赖：容器里固定 Humble + 构建链，避免“我机器能跑你机器不行”。
- 隔离开发机差异：宿主机 Python/系统库不同，不影响 ROS2 联调结论。
- 便于 CI/自动化：同一脚本可在不同机器重复执行。
- 降低客户机风险：Docker 仅用于开发联调，不会污染客户 Windows 最终运行环境。

项目约束：
- 最终交付运行是 Windows 原生（非 Docker）。
- Docker 仅用于 ROS2 联调 smoke test。

## 2) ROS2 在本项目中的角色

ROS2 不是“必须先学会才能控制手”，它在这里是“接口层”。

职责：
- 上层系统通过 ROS2 topic 发命令，不直接碰 PyBullet 细节。
- `sim_driver_node` 负责把 `/hand/command` 转成关节控制。
- 节点持续发布 `/hand/state` 和 `/hand/health`，便于上层闭环与监控。

当前接口（v1）：
- 输入：`/hand/command` (`std_msgs/msg/Float64MultiArray`, 长度必须=11)
- 输出：`/hand/state` (`sensor_msgs/msg/JointState`)
- 健康：`/hand/health` (`std_msgs/msg/String`)

这使得后续从“仿真驱动”替换为“真机驱动”时，上层调用尽量不变。

## 3) 测试清单：每个测试在干什么

## A. 本地仿真链路测试（不依赖 ROS2）

### `sim/sim_load.py`
- 目的：验证 URDF 能加载、关节可识别、仿真循环可运行。
- 主要风险覆盖：模型路径错误、依赖缺失、PyBullet 初始化失败。
- 通过信号：日志包含 `Movable joints` 和 `Simulation loop completed`。

### `tools/cli_control.py ... pose <name>`
- 目的：验证“给定姿态命令 -> 关节控制执行”链路。
- 主要风险覆盖：姿态文件格式错误、姿态长度不匹配、关节控制异常。
- 通过信号：退出码为 0，日志包含 `Pose applied: ...`。

### `sim/test_middle_home_cycle.py`
- 目的：做回归稳定性检查（中指姿态 <-> home 循环切换）。
- 主要风险覆盖：命令越界裁剪、重复切换后漂移、最终姿态误差过大。
- 关键指标：
  - `clip_events == 0`
  - `final_max_abs_error <= final_error_threshold`（默认 0.05）
- 通过信号：输出 `PASS test_middle_home_cycle ...`。

## B. ROS2 联调 smoke test（Docker）

### `ros2_ws/scripts/docker_smoke_test.sh`
- 目的：在统一容器环境验证“ROS2 输入输出接口契约”。
- 它实际做的步骤：
  1. 构建 Docker 镜像。
  2. 容器内 `colcon build` 构建 `hand_bridge`。
  3. 启动 `sim_driver_node`（headless）。
  4. 发布一条合法命令（11 维）。
  5. 发布一条非法命令（2 维，故意触发校验）。
  6. 读取一次 `/hand/state` 和 `/hand/health`。
  7. 断言：
     - `/hand/state` 包含 `name` 字段（状态成功发布）
     - `/hand/health` 中 `bad_length` 大于 0（非法命令被识别）
- 通过信号：脚本最后输出 `PASS: Docker ROS2 smoke test completed.`。

## 4) 建议的执行顺序（给不懂 ROS 的使用者）

1. 先跑本地仿真与回归（确认基础控制链路正常）。
2. 再跑 Docker ROS2 smoke test（确认接口层契约正常）。
3. 最后按 `docs/windows_runbook.md` 走 Windows 原生流程（最终交付路径）。

## 5) 哪些文档给谁看

- `README.md`：首次上手，先跑起来。
- `docs/runtime_and_tests.md`（本文件）：解释“为什么这样分层、每个测试值不值得信”。
- `docs/ros2_interface.md`：接口定义（给开发/联调）。
- `docs/windows_runbook.md`：客户现场执行手册（给实施/交付）。
- `plan.md`：项目计划与阶段进度（给管理与排期）。
