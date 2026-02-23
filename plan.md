# Linker Hand 06 Lite 控制落地计划（macOS + 无实体手）

## 当前进度（截至当前代码）
- Phase 0: 已完成（关节接口和约束在 `sim/joint_controller.py` 与文档中落地）。
- Phase 1: 已完成（`sim/sim_load.py` 可稳定加载并打印关节）。
- Phase 2: 已完成（`sim/demo_single_joint.py` 可复现单关节闭环）。
- Phase 3: 已完成（`sim/demo_all_joints.py` + `config/poses.yaml` + 回归验证）。
- Phase 4: 已完成（`sim/demo_key_control.py`、`tools/cli_control.py`、`sim/test_middle_home_cycle.py`）。
- Phase 5: 进行中（`ros2_ws/src/hand_bridge` 最小骨架已可在 Docker 中 build/run/topic 验证）。
- Phase 6: 未开始（目标为 Windows 真机驱动替换与现场联调）。

## 总体原则
- 当前目标不是复杂操作，而是“逐关节可控”。
- 先在 macOS 用 PyBullet 跑通，再切到客户 Windows 实机。
- 上层接口保持不变：后续只替换底层 driver。

## Phase 0: 需求冻结与接口约定（0.5 天）
### 目标
- 明确关节控制范围、命令格式、验收标准。

### 你要做什么
1. 确认机械手关节数量、关节名称、每个关节角度上下限（从 URDF 读取）。
2. 定义统一控制接口（建议）：
   - 输入：`joint_positions: [float...]`
   - 输出：当前关节状态、执行结果（成功/失败）
3. 写一页接口文档（字段、单位、范围、错误码）。

### 如何测试
- 测试 1：用 3 组合法/非法输入跑校验函数，非法输入必须返回明确报错。
- 测试 2：接口文档与 URDF 关节定义逐项对齐，无遗漏。

## Phase 1: 本地仿真环境搭建（0.5-1 天）
### 目标
- 在 macOS 启动 PyBullet，并加载 Linker Hand URDF。

### 你要做什么
1. 创建 Python 虚拟环境，安装 `pybullet`。
2. 拉取官方 URDF：`https://github.com/linker-bot/linkerhand-urdf`。
3. 写 `sim_load.py`：启动 GUI、加载地面和手模型、打印所有可动关节。

### 如何测试
- 测试 1：运行 `sim_load.py` 后，GUI 正常显示机械手。
- 测试 2：终端打印的关节数量 > 0，且名称列表稳定可复现。

## Phase 2: 逐关节控制最小闭环（1 天）
### 目标
- 能对任意单个关节下发角度命令并在仿真里看到动作。

### 你要做什么
1. 写 `joint_controller.py`：
   - 提供 `set_joint(index, target_rad)`
   - 做角度限幅（按 URDF）
2. 写 `demo_single_joint.py`：
   - 选定一个关节，循环在最小/最大角之间切换。
3. 写简单日志：命令值、限幅后值、当前值。

### 如何测试
- 测试 1：单关节切换 50 次，无崩溃。
- 测试 2：超限输入会被限幅，日志可见。
- 测试 3：其他关节基本保持静止（允许微小数值误差）。

## Phase 3: 多关节向量控制（1 天）
### 目标
- 一次命令控制全部关节（你要的核心能力）。

### 你要做什么
1. 扩展接口：`set_all_joints(target_positions)`。
2. 添加输入校验：
   - 长度必须等于关节数
   - 每项必须是数值且在范围内（或自动限幅）
3. 实现 3 组预设手势向量（仅用于回归测试）：
   - `open`
   - `close`
   - `half_close`

### 如何测试
- 测试 1：连续发送 100 次向量命令，模型始终可响应。
- 测试 2：错误长度输入必须失败并返回清晰错误信息。
- 测试 3：三个预设手势可重复切换且无异常跳变。

## Phase 4: 用户输入简化（0.5-1 天）
### 目标
- 让非技术用户无需懂 ROS/关节编号也能控制。

### 你要做什么
1. 做一个最小控制面板（二选一）：
   - CLI：`set j3 0.5`、`pose close`
   - Web：每个关节一个滑条 + 应用按钮
2. 提供“安全模式”：
   - 速度限制
   - 每次角度变化最大步长限制

### 如何测试
- 测试 1：随机操作 10 分钟，不出现越界命令。
- 测试 2：用户只看 README，5 分钟内能完成一次开合动作。

## Phase 5: ROS2 适配层（1 天）
### 目标
- 把当前控制逻辑封装成 ROS2 可替换 driver。

### 你要做什么
1. 定义 ROS2 topic（建议）：
   - 输入：`/hand/command`（关节向量）
   - 输出：`/hand/state`（当前关节状态）
2. 写 `sim_driver_node`：订阅命令，驱动 PyBullet。
3. 保持接口与未来 Windows 真机驱动一致。

### 如何测试
- 测试 1：`ros2 topic pub` 可驱动仿真手动作。
- 测试 2：`ros2 topic echo /hand/state` 连续稳定输出。
- 测试 3：断开/重连节点后可恢复控制。

## Phase 6: 切换到客户 Windows 真机（0.5-1 天，客户现场）
### 目标
- 不改上层输入与业务逻辑，仅替换底层驱动到官方 SDK。

### 你要做什么
1. 在客户 Windows 安装 ROS2 + 官方 `linkerhand-ros2-sdk`。
2. 用 `real_driver_node` 替换 `sim_driver_node`。
3. 对齐 topic 名称和关节顺序映射。

### 如何测试
- 测试 1：同一条 `/hand/command` 在仿真与真机行为一致（方向/幅度一致）。
- 测试 2：连续运行 30 分钟，无通信中断和异常抖动。
- 测试 3：故障演练（断开通信后恢复）可在 2 分钟内重新可控。

## 每阶段交付物清单
- Phase 0: `docs/interface.md`
- Phase 1: `sim/sim_load.py`
- Phase 2: `sim/joint_controller.py`, `sim/demo_single_joint.py`
- Phase 3: `sim/demo_all_joints.py`, `config/poses.yaml`
- Phase 4: `tools/cli_control.py` 或 `web_control/`
- Phase 5: `ros2_ws/src/hand_bridge/`（sim driver + msg/topic 约定）
- Phase 6: `docs/windows_deploy.md`, `docs/site_test_checklist.md`

## 最小里程碑（建议先冲这个）
- M1（2 天内）：Phase 0-3 完成，逐关节控制在 PyBullet 跑通。
- M2（第 3 天）：Phase 4 完成，客户可通过简化输入控制。
- M3（客户现场当天）：Phase 6 完成，真机可控。
