# ABI 指南（`motor_abi`）

## 构建

```bash
cargo build -p motor_abi --release
```

产物：

- Linux：`target/release/libmotor_abi.so`、`libmotor_abi.a`
- Windows：`target/release/motor_abi.dll`、`motor_abi.lib`
- 头文件：`motor_abi/include/motor_abi.h`

## 统一接口面

ABI 对外保持一套统一控制接口。

## ABI 元数据

- `motor_abi_version()` 返回当前加载的 ABI 库版本字符串。
- `motor_abi_capabilities_json()` 返回稳定 JSON 能力文档，描述已支持的
  transports、vendors、controller lifecycle、control modes 和厂商专用 features。
- Python 中对应 `motorbridge.abi_version()` 和 `motorbridge.abi_capabilities()`。
- C++ 中对应 `motorbridge::abi_version()` 和 `motorbridge::abi_capabilities_json()`。
- `bindings/api_surface.json` 用于跟踪 ABI、Python、C++ 和文档的接口对齐。

统一模式 ID（`motor_handle_ensure_mode`）：

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

统一控制单位：

- 位置：`rad`
- 速度：`rad/s`
- 力矩：`Nm`

通用控制/状态接口：

- `motor_handle_enable`
- `motor_handle_disable`
- `motor_handle_clear_error`
- `motor_handle_set_zero_position`
- `motor_handle_ensure_mode`
- `motor_handle_send_mit`
- `motor_handle_send_pos_vel`
- `motor_handle_send_vel`
- `motor_handle_send_force_pos`
- `motor_handle_request_feedback`
- `motor_handle_set_can_timeout_ms`
- `motor_handle_store_parameters`
- `motor_handle_get_state`

## 厂商入口

- Damiao：`motor_controller_add_damiao_motor(...)`
- Hexfellow：`motor_controller_add_hexfellow_motor(...)`（通过 `socketcanfd` 走 CAN-FD）
- RobStride：`motor_controller_add_robstride_motor(...)`
- MyActuator：`motor_controller_add_myactuator_motor(...)`
- HighTorque：`motor_controller_add_hightorque_motor(...)`
- CyberBeast：`motor_controller_add_cyberbeast_motor(...)`（仅经典 CAN 路径）

## 统一模式与厂商原生协议映射

| 统一模式 | Damiao 原生 | Hexfellow 原生 | RobStride 原生 | MyActuator 原生 | HighTorque 原生 | CyberBeast 原生 |
|---|---|---|---|---|---|---|
| `MIT` | `Mit` | 模式 `5` | `Mit` | 不支持 | 映射到原生 pos+vel+tqe | `Mit` |
| `POS_VEL` | `PosVel` | 模式 `1` | 映射到 `Position`（`run_mode=1`，`limit_spd=0x7017`，`loc_ref=0x7016`） | `Position` 设定流程 | 映射到原生 pos+vel+tqe | `Position`（速度限制成为速度上限） |
| `VEL` | `Vel` | 不支持 | `Velocity` | `Velocity` 设定流程 | 映射到原生速度命令 | `Velocity` |
| `FORCE_POS` | `ForcePos` | 不支持 | 不支持 | 不支持 | 映射到原生 pos+vel+tqe | 映射到原生力矩控制；只用比例系数（`torque = 比例 * 模型 MIT 力矩上限`） |

行为约定：

- 不支持的调用返回非 0，并可通过 `motor_last_error_message()` 获取清晰错误信息。
- 即使某厂商忽略部分参数，也保持统一函数签名不变。
- 例如：HighTorque 支持 `send_mit(pos, vel, kp, kd, tau)` 统一签名，但原生协议不使用 `kp/kd`。
- Hexfellow 的 ABI 路径支持 `MIT` 和 `POS_VEL`，`VEL` / `FORCE_POS` 会返回不支持。
- CyberBeast 的 ABI 路径四个模式都支持（`MIT` / `POS_VEL` / `VEL` / `FORCE_POS`）。`feedback_id` 仅为签名兼容保留、协议不使用：命令与反馈帧都按 `motor_id`（8 位 CAN 节点 ID）寻址。
- CyberBeast 的 `motor_handle_ensure_mode` 只校验统一模式码并发送 `START_MOTOR`；真正的模式由每帧控制指令携带（MIT / 位置 / 速度 / 力矩报文类型），不是写模式寄存器。
- CyberBeast 的参数访问是 16 位 ODrive SDO 端点上的 float32 通路（`motor_handle_cyberbeast_get_param_f32` / `_write_param_f32`）；完整端点表在**添加电机时**从设备读取（协议 4.8，`motor_controller_add_cyberbeast_motor` 因此在节点不应答时会失败）并由 `motor_handle_cyberbeast_endpoint_map` 返回。
- RobStride 的 ABI 路径支持 `POS_VEL`，语义映射为原生 Position：先设置 `run_mode=1`，再写 `limit_spd` 与 `loc_ref`。
- RobStride 的统一高层目前支持 `MIT/POS_VEL/VEL`；`TORQUE/CURRENT` 仍是参数级能力（通过 `robstride_write_param_*`），尚未开放统一模式。
- Damiao 置零顺序规则：先调用 `motor_handle_disable`，再调用 `motor_handle_set_zero_position`；否则会被核心防护拒绝。
- Damiao 置零稳定规则：`set_zero_position` 成功后，核心层内置固定稳定等待（约 `20ms`），ABI 不额外暴露等待参数。

## 厂商扩展接口

Damiao 寄存器接口：

- `motor_handle_write_register_f32`
- `motor_handle_write_register_u32`
- `motor_handle_get_register_f32`
- `motor_handle_get_register_u32`

RobStride 扩展接口：

- `motor_handle_robstride_ping`
- `motor_handle_robstride_ping_host_id`
- `motor_handle_robstride_set_device_id`
- `motor_handle_robstride_set_active_report`
- `motor_handle_robstride_get_fault_report`
- `motor_handle_robstride_get_param_f32_host_id`
- `motor_handle_robstride_write_param_i8/u8/u16/u32/f32`
- `motor_handle_robstride_get_param_i8/u8/u16/u32/f32`

CyberBeast 扩展接口（ODrive SDO 端点）：

- `motor_handle_cyberbeast_get_param_f32(motor, param_id, timeout_ms, out_value)`
- `motor_handle_cyberbeast_write_param_f32(motor, param_id, value)`（等待设备写确认，预算 200 ms）
- `motor_handle_cyberbeast_endpoint_map(motor, timeout_ms, out_total_len, out_version_crc)` 返回设备自报的 JSON 端点描述符（UTF-8；指针在本线程下次 ABI 调用前有效，失败返回 NULL）。添加电机时已缓存描述符，因此该调用通常**立即返回**，只有从未加载过时才发起传输。`out_total_len` / `out_version_crc` 可为 NULL，否则回传 `TotalLength` / `VersionCRC`。Python 侧：`Motor.cyberbeast_endpoint_map(timeout_ms)` + `motorbridge.cyberbeast_endpoints.parse_endpoint_descriptor()`。

## 典型调用顺序

1. 选择传输层构造器：
   - `motor_controller_new_socketcan(channel)`（通用路径）
   - `motor_controller_new_socketcanfd(channel)`（CAN-FD 路径；Hexfellow 必须使用）
   - `motor_controller_new_dm_serial(serial_port, baud)`（仅 Damiao 串口桥；跨平台，可用 `/dev/ttyACM0` 或 `COM3`）
2. `motor_controller_add_<vendor>_motor`
3. 可选：`motor_controller_enable_all`
4. 可选：`motor_handle_ensure_mode`
5. 发送控制命令 / 读取状态 / 调用厂商扩展接口
6. `motor_controller_shutdown`
   - 或在只关闭本地会话/总线时调用 `motor_controller_close_bus`
7. `motor_handle_free`
8. `motor_controller_free`

## 示例

- C ABI 示例：`examples/c/c_abi_demo.c`
- C++ ABI 示例：`examples/cpp/cpp_abi_demo.cpp`
- Python ctypes 示例：`examples/python/python_ctypes_demo.py`
