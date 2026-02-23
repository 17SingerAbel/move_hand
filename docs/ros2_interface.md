# ROS2 Interface Contract (v1)

## Purpose
固定 ROS2 接口契约（contract），保证后续从仿真驱动切换到真机驱动时，上层调用不变。

## Topics

### 1) `/hand/command`
- Type: `std_msgs/msg/Float64MultiArray`
- Semantics: 目标关节角（unit: rad）
- Constraint:
  - `len(data) == 11`（当前 o6 right 模型）
  - 顺序必须与 `/hand/state.name` 一致

示例：
```yaml
{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

### 2) `/hand/state`
- Type: `sensor_msgs/msg/JointState`
- Semantics:
  - `name`: 关节名顺序（contract source of truth）
  - `position`: 当前关节角（rad）
- Publish rate: `publish_rate_hz` 参数控制（默认 50Hz）

### 3) `/hand/health`
- Type: `std_msgs/msg/String`
- Semantics: 运行健康与输入质量统计
- Example payload:
  - `ok=1 joints=11 valid_commands=12 bad_length=1 bad_value=0 apply_errors=0 last_valid_cmd_age_sec=0.032`

## Parameters (`sim_driver_node`)
- `urdf` (string): URDF 路径
- `headless` (bool): 是否无 GUI
- `publish_rate_hz` (float): 状态发布频率
- `sim_steps_per_cycle` (int): 每周期仿真步数
- `max_force` (float): 位置控制力矩上限

## Error Handling (current)
- command 长度错误：忽略该消息、warning、`bad_length` 计数 +1。
- command 包含 `NaN/Inf`：忽略该消息、warning、`bad_value` 计数 +1。
- 控制应用异常：记录 `apply_errors` 计数并继续运行。
- 节点持续运行，后续合法命令可恢复控制。

## Not Included in v1
- velocity/effort command
- action interface
- trajectory message

后续若要升级，请在 v2 文档中增加字段并保持 backward compatibility。
