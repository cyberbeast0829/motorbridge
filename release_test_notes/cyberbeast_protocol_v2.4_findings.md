# CyberBeast 协议 v2.4 文档 vs 真机固件：差异与证据

面向对象：CyberBeast / MCS 固件与文档维护者
整理来源：motorbridge 真机测试（2026-09-25）

---

## 0. 测试环境（可复现）

| 项 | 值 |
|---|---|
| 转接器 | `16d0:117e` MCS **CyberBeast USB2CAN**（`iSerial = N32H473-0001`），slcan 固件，`/dev/ttyACM0` |
| 链路 | Linux SocketCAN（`slcand -o -c -s8`）→ `slcan0`，**1 Mbit/s**，RX errors 0 / dropped 0 |
| 节点 | 单轴，`axis0.config.can.node_id = 1`，`is_extended = true`（29-bit 扩展帧） |
| 主站 | `Source = 1`（协议默认 master id） |
| 固件 | `QUERY_DEVICE_INFO (0x46)` 回 `hw = 0x00040237`（4.2.55）、`fw = 0x00000609`（0.6.9） |
| 端点描述符 | `JSON_DESC_READ 0x24` + `JSON_DESC_DATA 0x25`：`TotalLength = 38433`，`VersionCRC = 0x3F82`，554 个端点 |
| 心跳 | `0x19200404`（Priority=6 STATUS、MsgType=0x48、Dest=1、Source=1、Seq=0），**实测 10.00 Hz**，`life` 计数器 0→7 递增 |

> 复核工具：`tools/cb_json_probe.py`（拉取并解析端点描述符）；
> `motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode read-param --endpoint <id>`
> （打印声明的类型 + 原始小端字节）；`--trace` 打印每一帧的 29-bit 位域与载荷。

---

## 1. ✅ PARAM_READ (0x20) 的 **value 字节序：v2.5 已澄清为小端（本项已关闭）**

> **状态：已关闭（2026-09-25）** —— 厂商文档 **v2.5** 已把 PARAM_READ/PARAM_WRITE 的
> **参数值明确为 Little-Endian**，并补充了 4.7 节的字节序说明、自检顺序与示例；
> 同时明确 JSON 描述符（4.8）的 Offset / TotalLength / VersionCRC / ChunkOffset 也为小端。
> SDK 实现与 v2.5 一致，**无需再向厂商确认**。

**（历史记录）当时文档 4.7 写的是**：
```
响应:
Byte 4..:   [Value]           (DataLen 字节, Big-Endian)
```

**实测（三条独立证据，均为 `Offset=0, ReqLen=4` 的响应帧）**：

| 端点 | 变量 | 响应原始帧（payload） | 按**小端**解 | 按**大端**解（v2.4 文档） |
|---|---|---|---|---|
| `0x0002` | `odrv.vbus_voltage` | `00 00 02 04 **26 B7 B8 41**` | **23.09 V** ✅ | 4.5e-15 ❌ |
| `0x00F7` | `axis0.motor.config.torque_constant` | `00 00 F7 04 **AF D9 A8 3D**` | **0.0824 Nm/A** ✅ | 1.5e-14 ❌ |
| `0x0186` | `axis0.encoder.config.cpr` | `00 01 86 04 **00 40 00 00**` | **16384** ✅ | 4 194 304 ❌ |

（payload 前 4 字节 = `Flags | EndpointID(2B BE) | DataLen`；上表加粗处是 value 4 字节。）

**请求侧**：Offset 字段（`Byte 4-7, uint32`）**大端**解析是可用的——
对 `serial_number`（uint64）先用 `Offset=0` 拿到 `Flags=0x80 (More), DataLen=4, bytes[0..3] = 50 34 66 0D`，
再用 `Offset=4` 补齐后 4 字节，拼出的 `uint64` = `108005767394384` 合理；即
**请求 offset 大端 / 响应 value 小端** 的组合。

**结论（v2.5 已澄清，无需厂商回答）**：文档 v2.5 明确 **PARAM_READ 与 PARAM_WRITE 的参数值都是小端**
（§4.7「参数值不是 Big-Endian」，并把“参数值一律按小端收发”写成实践规则），SDK 已按小端实现。

> ⚠ **我们自己的教训（2026-09-26）**：文档早已澄清，但**写**路径仍沿用 v2.4 时代的**大端**实现，
> 而我在扩展类型化写入时**没有回源核对 §4.7**，还根据旧注释写下了“文档写大端”的错误结论。
> 代价是实测付出：一次大端写 `heartbeat_rate_ms=100` 被固件存成 1677721600，直接打停了心跳（见 4.2，
> 已恢复）。⇒ **协议文档更新后必须回源核对，代码注释不能当作依据。**

### 1.1 ⚠ MIT 命令的取整方式：v2.5 明确为「向零截断」（SDK 原先用四舍五入，已修）

**文档 v2.5 4.1.1（新增说明）**：
```
int_val = clamp(trunc((float_val - offset) / span * (2^bits - 1)), 0, 2^bits-1)
```
> ⚠ 取整方式 = 向零截断 (C 式 `(int)`)，不是四舍五入。例如 `pos=0` / `mit_max_pos=12.5`
> 编码为 `32767 (0x7FFF)` 而非 `32768 (0x8000)`。

**真机印证**：设备 idle 时对 `QUERY_STATUS (0x40)` 回的 MIT 响应帧为
`7F FF 7F F0 80 02 4E 50` → `p_int = 0x7FFF`（若按四舍五入应为 `0x8000`）。

**SDK 修复**：`float_to_uint()` 去掉 `.round()`，改为 `as u32`（向零截断），并加守卫测试
（断言 `pos=0` → `0x7FFF`，且往返误差 < 1e-3 rad）。影响：此前中位附近有 1 LSB 偏差
（4π 量程下约 0.0004 rad），现与固件一致。

---

## 2. ⚠ 端点 ID 不是固定编号，必须走 JSON 描述符（文档方向正确，实现提示不足）

**文档 4.7/4.8** 说端点 “兼容 Fibre endpoint 体系”，并提供了 `JSON_DESC_READ 0x24 / JSON_DESC_DATA 0x25`。

**实测**：描述符可用且完整（38433 字节 / 554 端点 / `VersionCRC = 0x3F82`），
但**端点 ID 与“序数编号”无关**。我们早期按“看起来自然”的编号访问全部失败，对照如下：

| 变量 | 自然编号（❌ 实测无效） | **描述符给出的真实 ID** | 类型 |
|---|---|---|---|
| `odrv.error` | — | `0x0001` | uint8 rw |
| `odrv.vbus_voltage` | — | `0x0002` | float r |
| `odrv.serial_number` | — | `0x0005` | uint64 r |
| `axis0.current_state` | `0x0000` | **`0x008E`** | uint8 r |
| `axis0.requested_state` | `0x0001` | **`0x008F`** | uint8 rw |
| `axis0.config.watchdog_timeout` | — | `0x0099` | float rw |
| `axis0.config.enable_watchdog` | — | `0x009A` | bool rw |
| `axis0.motor.config.torque_constant` | `0x0019` | **`0x00F7`** | float rw |
| `axis0.motor.config.current_lim` | `0x001C` | **`0x00F9`** | float rw |
| `axis0.controller.config.control_mode` | `0x0030` | **`0x011F`** | uint8 rw |
| `axis0.controller.config.pos_gain` | `0x0035` | **`0x0123`** | float rw |
| `axis0.encoder.config.cpr` | — | `0x0186` | int32 rw |

**请厂商确认**：
1. 是否有**官方**的“端点 ID ↔ 名称/类型”表可供 SDK 内置（而不是只能运行时从描述符解析）？
2. 描述符 `VersionCRC` 变化时，主站应如何判断“端点映射已变、需重新拉取”（推荐做法）？

**SDK 现状（2026-09-25）**：已实现描述符读取，不再依赖外部脚本：

- Rust: `CyberBeastMotor::read_endpoint_descriptor(timeout)` /
  `read_endpoint_descriptor_raw(timeout)`（协议 4.8，含元数据帧重复出现与续传处理）
- CLI: `motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode endpoint-map [--out map.json] [--dump]`
- 真机结果：`38433 bytes, VersionCRC=0x3F82, 554 个 "id"`；与独立实现
  （`tools/cb_json_probe.py`，裸 SocketCAN 直接收发）解析后的 JSON **结构完全一致**

---

## 3. ⚠ MIT 响应帧的 **Current 量程**依赖配置（文档已说明；SDK 默认值待修正）

**文档 4.1.2 备注**：`max_current = mit_max_torque / torque_constant`，钳位上限 80 A；
`torque_constant` 无效时用默认 ±40 A。

**实测本机**：`mit_max_torque = 50 Nm`（0x0151）、`torque_constant = 0.0824 Nm/A`（0x00F7）
⇒ `50 / 0.0824 = 606 A` → **钳位到 80 A**，即 MIT 响应的电流分辨率应按 **±80 A** 解读。

**请厂商确认**：钳位值 80 A 是否为固定常量？（若是可配置，请给出读取方式。）
> SDK 侧：`DEFAULT_MIT_CURRENT_LIMIT = 40`（用于响解）需改为“优先按描述符/端点推导，回退 40”。

---

## 4. ✅ 已与文档**一致**、经真机验证的部分（供交叉确认）

| 项目 | 文档 | 实测 |
|---|---|---|
| Classic CAN 心跳 8 字节 | `Life3\|Err5 / State4\|Mode4 / MtrTmp(uint8-50) / Pos int16 BE turns×100 / Vel int16 BE turns/s×100 / Iq int8 0.5A` | ✅ 逐字节吻合（`A0 13 4E …` → life=5、err=0、28 °C、pos=0、vel≈±0.02 turns/s、iq=0） |
| 心跳周期 | 默认 100 ms（`can.heartbeat_rate_ms`） | ✅ **实测 10.00 Hz**，端点 `axis0.config.can.heartbeat_rate_ms = 100` |
| CAN ID 布局 | `Priority[28:26]\|MsgType[25:18]\|Dest[17:10]\|Source[9:2]\|Seq[1:0]` | ✅ 用 `(6<<26)\|(0x48<<18)\|(1<<10)\|(1<<2)\|0` 精确复现设备心跳 ID `0x19200404` |
| MIT 响应 (0x00) | `Pos16 \| Vel12\|Err4 \| Cur12\|Mode4 \| MotorTmp \| MOSTmp` | ✅ 实测 `7FFF7FF080024E50` → pos≈0、vel≈0、err=0、mode=2(IDLE)、28 °C / 30 °C |
| `ModeState` 枚举 | `0x2 = IDLE` | ✅ 设备 idle 时回 2；且 `axis0.current_state = 1` 与心跳 `byte1` 高半字节 = 1 互相印证 |
| POS_CONTROL Classic | 输出端**度数** / int16 RPM / int16 0.1A | ✅ 与实现一致（尚未做运动验证） |
| `QUERY_DEVICE_INFO` Classic | `Byte 0-3 HW uint32 = (MAJOR<<16)\|(MINOR<<8)\|VARIANT`；`Byte 4-7 FW` 同构 | ✅ `hw 4.2.55` / `fw 0.6.9`，与描述符里的 `hw_version_*` / `fw_version_*` 字节一致 |
| `QUERY_POS_VEL` | float32 BE，**电机端 turns / turns/s** | ✅ 语义一致（SDK 已在 state 层统一换算为 rad，见 §5） |
| 端点分段读 | ReqLen/Offset + `Flags bit7 = More`；Classic 下 4 字节/块 | ✅ `serial_number` 两次请求拼出 uint64 |

---

## 4.1 端点表已改为“连接即加载”（真机验证）

以前 SDK 只带一张 34 项的手工表，读其它端点只能猜类型。现在**添加电机时就把描述符全量读下、解析、缓存**，
`read-param` / `write-param` 只查缓存表，不再有“表里没有”的分支。真机（slcan0 / fw 0.6.9 / node 1）实测：

| 项 | 结果 |
|---|---|
| 连接即加载 | `--mode status` 先打印 `endpoint map: 554 endpoints (521 values), 38433 bytes, VersionCRC=0x3F82` |
| 描述符成本 | 每次连接约 **2.3 s**（5~6 个续传周期）；`--no-endpoint-map` 下同一条命令 **0.9 s** 且不发任何帧 |
| 按名字读 | `--mode read-param --endpoint gear_ratio` → `0x00F2 (axis0.motor.config.gear_ratio) declared=float access=rw value=7.75` |
| 分段 uint64 | `--endpoint serial_number` → `0x0005 declared=uint64 value=108005767394384`（两段拼接） |
| 窄类型 | `--endpoint 0x008E` → `axis0.current_state declared=uint8 value=1`（不再被猜成 float） |
| 写保护 | `--mode write-param --endpoint current_state --value 8` → **拒绝**：`declared access="r"`（rc=1，未写入） |
| 缓存复用 | `--mode find-endpoint` / `--mode endpoint-map --out` 用缓存，输出与独立探针 `tools/cb_json_probe.py` **逐字节一致** |
| Python | `add_cyberbeast_motor` 约 2.3 s（此时已建表），随后 `cyberbeast_endpoint_map()` 毫秒级返回；不存在的 node 0x09 约 0.5 s 报错 |
| 静默节点 | 若始终收不到元数据帧则**立即**失败（原重试 64 次≈ 32 s 已改为 1 个静默窗口） |

类型分布（描述符实测）：521 个值 = float 223 / uint32 124 / bool 81 / uint8 47 / int32 21 / uint16 20 / uint64 4 / int64 1，
另有 26 个 function、6 个 endpoint_ref、1 个 json。

顺带修掉一个缓存 bug：同一端点连读两次时，第二次曾在超时窗口内直接返回上一次的值（C ABI / Python 受影响）；
现在起始请求会清掉旧值/旧回执/旧形状（有回归用例，把修复注释掉则用例必红）。

---

## 4.2 运动测试：使能、单位与减速比（真机，2026-09-26）

环境：slcan0 / 1 Mbit/s / fw 0.6.9 / node 1，转轴固定且空载。增益保持很小（最大力矩 ≈1 Nm = 39% `torque_lim`），
全程电流 0.02~0.137 A，无振荡。

### 使能路径

| 帧 | Priority | 结果 |
|---|---|---|
| `START_MOTOR` (0x62) | 0 / 2 | **被忽略**（`current_state` 保持 1 = IDLE） |
| `START_MOTOR` | 3 / 4 / 5 / 6 | ✅ 进入闭环（`current_state` = 8） |
| `STOP_MOTOR` (0x63) | 4 / 6 | ✅ 回到 IDLE |
| `requested_state = 8`（SDO 写 0x008F） | — | ❌ 不产生状态跳转（写被消费，读回 0） |

- 我们的 SDK 原来用 `Priority::Ctrl` = **3** ✓ 本来就能生效，但 **CLI `enable` 模式在退出时调用 `shutdown()`**，
  而 `shutdown()` 会给每个电机发 `StopMotor` ⇒ 刚发的 StartMotor 立刻被撤销（`--mode enable` 从未真正使能过）。
  另外 `Priority ≤ 2` 会被固件丢弃 —— 早期手工测试用 prio 2 得出“StartMotor 无效”的结论是错的。
- 建议文档 3.5 节补上系统管理类报文的**要求 Priority**（实测 ≥3）。

### 单位与减速比（实验反解）

MIT 命令 `pos` 为输出端 rad；MIT 响应也是输出端；心跳 / `QUERY_POS_VEL` 为电机端 rad。两段小幅步进：

| 命令 pos (输出端 rad) | 电机端实测 (QUERY_POS_VEL, rad) | MIT 响应 pos |
|---|---|---|
| 0.1（kp=5, kd=0.5, tau=0） | 0.586006 | 0.0761 |
| 0.2（同上） | 1.336601 | 0.1734 |

- 两次之差：Δ命令 0.1 → **Δ电机端 0.750595**；响应侧 Δ0.0973 ⇒ **比值 7.71**（描述符声明 `gear_ratio` = 7.75）⇒
  **`motor_rad = output_rad × gear_ratio` 得到验证**（差值法把静摩擦导致的稳态误差抵消掉了）。
- 因此文档 4.1.1 的“命令为输出端、反馈为电机端”得到确认；SDK 现把 MIT 响应乘上 `gear_ratio` 后入库，
  使 `state.pos/vel` 恒为电机端（此前会因最后到达的帧不同而差 7.75 倍）。
- 稳态误差：命令 0.1 输出端 rad 时响应停在 0.0973 ⇒ 误差 0.0027 rad × kp 5 = **0.013 Nm** 左右的持用力矩，
  说明 7.75:1 齿轮箱的静摩擦大约在此量级（kp=1、目标 0.02 rad 时完全不动，力矩仅有 0.02 Nm）。

### 报文单位对照（真机实测）

| 报文 | 单位/字节序 |
|---|---|
| `QUERY_POS_VEL` 响应 | float32 **BE**，电机端 turns / turns/s（已 ×2π 转 rad） |
| SDO `PARAM_READ` 值 | **LE**（vbus / torque_constant / cpr / serial 均逐字节核对） |
| SDO `PARAM_WRITE` 值 | **LE**（大端写入会把 100 存成 1677721600 并把心跳打死） |
| MIT 命令/响应位域 | 由设备声明量程缩放（12.5 / 65 / 50 / 500 / 5） |

---

## 5. 需要厂商确认的两个“语义边界”（不是 bug，但影响上位机实现）

1. **电机端 vs 输出端**：文档 4.1.1 说明 MIT/POS/VEL 命令为**输出端**单位（固件内部 `× gear_ratio / 2π`），
   而心跳/`QUERY_POS_VEL` 上报的是**电机端 turns**。
   **（已解决）** 减速比可从设备自身描述符读到：端点 `axis0.motor.config.gear_ratio` = **0x00F2**
   （float rw），本机实测 **7.75**（原始字节 `00 00 F8 40`，LE float32）；
   用 `motor_cli --mode find-endpoint --name gear` 即可查到。
   SDK 仍把状态量定义为“**电机端 rad**”、不做减速比修正（保证与 `get_param_f32` 等同层读数一致），
   上位机换算到输出端请除以本值：`output_rad = motor_rad / gear_ratio`。
   建议文档在 4.1.1 直接写明该端点 id。
   （该值本身已读实；它是否就是固件换算所用的那个系数，仍需运动实验反解验证。）

2. **ESTOP 的 Dest**：文档 4.9 定义 `Priority = 0 (CRITICAL)`、`Dest = 0xFF`（全局广播），
   并说明收到后进入 IDLE 且**锁存** `ERROR_ESTOP_REQUESTED`。
   请确认：`Dest` 是否**必须**为 `0xFF`？固件是否会忽略“非 0xFF 的 ESTOP”？
   （我们已按文档改为 `Priority=0 + Dest=0xFF + MsgType=0xC0`；尚未在真机上触发，因为会锁存故障。）

3. **看门狗**：实测 `enable_watchdog = false`、`watchdog_timeout = 0` ⇒ 该轴当前不会因通信中断而保护。
   请确认：`watchdog_timeout = 0` 的语义是“禁用”还是“立即超时”？（文档未给出）

---

## 6. 复现实验的最小步骤

```bash
# 1) 建链路（1 Mbit/s）
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0 && sudo ip link set slcan0 up

# 2) 拉取端点描述符（554 项，含名称/类型/access）
python3 tools/cb_json_probe.py slcan0 /tmp/endpoints.json

# 2b) SDK 现在连接时就建表，可直接按名字读（无需先拉 JSON）
motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode find-endpoint --name gear
motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode read-param --endpoint gear_ratio

# 3) 读参数：打印声明类型 + 原始小端字节（可与上位机/odrivetool 对拍）
motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode read-param --endpoint 0x0002

# 4) 看原始帧位域（含心跳解码）
motor_cli --vendor cyberbeast --channel slcan0 --motor-id 1 --mode status --trace
```
