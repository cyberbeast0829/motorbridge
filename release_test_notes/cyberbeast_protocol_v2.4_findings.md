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

### 4.1.1 ⚠ 分块传输丢帧：SDK 侧曾经的错法（症状已复现，已修）

线缆/适配器丢一帧是正常事件，而协议 4.8 **没有逐块序号也没有校验**，只有元数据帧里的 `TotalLength` /
`VersionCRC`。SDK 原来的重组方式在丢帧时必然出错：

1. 每块按自己的 `ChunkOffset` 落位，**没收到的地方用 0 填**（`resize(.., 0)`）；
2. 续传请求用 `bytes.len()` 当偏移 —— 那是**已收到块的末尾**（高水位），不是“第一个缺失字节”，
   于是空洞再也不会被补上；
3. 最终只检查“长度够了就收工”，因此带洞的缓冲被当成完整描述符送去解析。

症状与现场一致：JSON 报 `control character (\u0000-\u001F)` / `expected ':'`，位置正好落在空洞处，
约每 10 次连接出现一次。**真机复现方式（确定性）**：连接过程中，由第二个进程再发一次
`JSON_DESC_READ`（偏移 20000，`cansend slcan0 10900404#204e000000000000`）。协议规定**再次请求会把传输
重置到该偏移**，于是 7200~20000 这段永远不来：

| 二进制 | 结果（各 4 次） |
|---|---|
| 修复前 | **4/4 失败**：`not valid JSON: control character (\u0000-\u001F) ... column 7207` / `expected ':' ... column 7195`；描述符请求数 2 |
| 修复后 | **4/4 成功**，`554 endpoints`；描述符请求数 3（1 次起始 + 2 次补洞续传） |

修复后的规则（`motor.rs::JsonDescCache`）：

- `bytes` **只保存从偏移 0 开始的连续段**，永不补零 ⇒ `bytes.len()` 就是“第一个缺失字节”；
- 非连续帧一律丢弃（上一次中断传输的残留、重复帧、越过空洞的块都不落盘），丢弃无代价：
  再发一次 `JSON_DESC_READ` 会让设备**从该偏移重新开始**；
- 空洞由“**静默窗口到期且仍未收满**”触发续传（不会因为残留帧而乱发请求）；同一偏移连续 3 次请求
  都没有新字节则诚实报错，并给出停在第几字节、越洞帧数、残留帧数；
- 出现**第二个不同的元数据帧**（两版描述符同时在线上，只有中断过传输才会这样）说明流被污染：
  丢弃重组结果、重新请求（至多 2 次），再失败就如实报错；
- 长度必须恰好等于 `TotalLength`，**绝不接受部分结果**；`TotalLength > 65535` 直接拒绝
  （`ChunkOffset` 只有 u16，固件会静默回绕）。

另外两条来自真机/同业 SDK（JoinSDK `cb_jsondesc_fetch.c` / `jsdk_desc.c`）的经验也已落地：
**会话首帧可能丢**（适配器刚打开时还在配置自己的控制器，而 `JSON_DESC_READ` 恰好是这条链路的第一帧）
⇒ 未收到元数据帧时按较短静默窗口重发（至多 4 次，静默节点约 1 s 报错，不会退化成 30 s 卡顿）。

真机回归（slcan0 / fw 0.6.9 / node 1）：**15 次连续连接 + 6 次“传输中途 kill -9 后立即重连”，全部成功**，
每次只发 **1 个**描述符请求（设备自主流式发送）；整条描述符流实测 6407 帧 / **约 3.2 s**。
建议（低优先级，非缺陷）：v2.6 若给 0x25 加一个块序号或整段 CRC，上位机就能一眼看出丢帧，而不必靠“静默 + 补洞”。

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

### 4.3 ⚠ `can.config.break_timeout`：CAN 协议级看门狗（真机验证，2026-09-26）

端点 `can.config.break_timeout`（id **0x0049**，uint16 rw，单位 **ms**；`0` = 超时检测**禁用**，
不是“100 ms”）。真机（slcan0 / fw 0.6.9 / node 1）实测：

| 场景 | 结果 |
|---|---|
| `= 0`，控制帧停发（SIGKILL，未发 StopMotor） | **不动作**：8.3 s 内心跳一直 `axis_state=8`（闭合环）、`errflags=0` ⇒ 主站死了电机**一直通着电**（安全点） |
| **只写 500 ms，之后不发任何帧**（全新重启、error=0 起步） | **2.5 s 内就锁存** `axis0.error = 0x00100000` ⇒ 修正结论：**武装靠“写入非零值”，不是“收到第一条控制帧”** |
| 边喂边武装 500 ms，再停发 | **约 0.5 s 触发**（实测 +0.57 s，心跳 100 ms 分辨率）：`axis0.error = 1048576 = 0x00100000`（**bit20 `CAN_BUS_FAILED`**）、心跳 `errflags` bit0(axis)、轴回 IDLE |
| 谁算“喂狗” | **只有控制类帧**（MIT/POS/VEL/TORQUE）；PARAM_READ / PARAM_WRITE / status 查询都**不算**（实测它们不阻止触发） |
| 周期 800 ms > 500 ms | **运行中就触发**，且之后控制帧无法重新使能 ⇒ **控制周期必须 < break_timeout** |
| 锁存后的表现 | 固件**拒绝进闭环**：控制帧照发、循环照跑、进度行照打，**轴一动不动**（实测：`--loop-ms 800` 跑满 6 s、`axis_state=1` IDLE、心跳 `errflags=0x01`）。这是最容易误判成“功能没实现”的一种失败 |
| 故障清除 | 会**锁存**：`CLEAR_ERRORS(0x65)` 单独用可能清不掉（狗还武装着就清会**立刻重新锁存**）。可用顺序：`break_timeout=0` → `STOP_MOTOR` → `CLEAR_ERRORS`；或 `0x64 RESET_DEVICE` |

**两条容易踩的帧语义**（文档 3.5 表只给了名字）：

| 帧 | 语义 | 代价（实测） |
|---|---|---|
| `0x64` **RESET_DEVICE** | 复位设备（重启固件） | 配置/标定**保留**（gear_ratio 7.75、current_lim 40 A、`pre_calibrated` 仍 true、`is_ready` 回 true）；但**位置基准归零**（31.27 → 0 motor turns，轴未动） |
| `0x23` **CONFIG_RESET** | 擦除配置、恢复出厂并重启 | ⚠ **电机会变成未标定**，必须重新标定 —— 不要用它“复位故障” |

SDK 侧已同步：

* `--mode reset`（0x64，需 `--yes`）：不载端点表、探测连接也不发多余帧，就是发这一帧；用于清不掉故障的场景；
* 控制类模式（mit/pos/vel/torque）启动前有**两道前置检查**（都只看设备自己的表，`--no-endpoint-map` 时无法解析名字、按文档自动跳过）：
  1. **周期规则**：`--loop-ms` ≥ `can.config.break_timeout` 直接拒绝，并给出
     “改小 `--loop-ms` 或 `--mode write-param --endpoint break_timeout --value 0 --yes`” 的指引；
  2. **锁存故障**：`axis0.error != 0` 直接拒绝（并列出 bit 名与恢复步骤），否则就是上面那条“循环照跑、轴不动”的静默失败；
  3. 若 `break_timeout` 非零但周期合法，仍会打印一条提示（非零值下本 CLI 连接耗时可能已让设备锁存）；
* `--mode clear-error` 会补印正确的恢复顺序；
* `send_reset_device()` 的文档里写明它**不是** `CONFIG_RESET (0x23)`（有断言帧类型 ≠ 0x23 的用例）。

验证用脚本（均未提交）：`cb_break_timeout.sh` / `cb_break_timeout2.sh`（触发时延与慢周期）、
`cb_followups.sh`（周期规则 5 例 + `--no-endpoint-map` 反例 + 复位模式 + 移动）、
`cb_t2_diag.sh` + `tools/cb_trip_timing.py`（“写入即武装”的对照实验与逐帧间隔）、
`cb_fault_guard.sh`（锁存故障下拒绝启动、以及健康轴上真的进闭环）。

---

### 4.4 slcan「首帧丢失」在 motorbridge 上的复现尝试（结论：不是本仓库的机制）

起因：台架上一度看到「`--mode mit --no-endpoint-map` 的 StartMotor 不在总线上」，
据此推断「slcan 适配器打开端口时重置输入缓冲、丢掉会话头一两帧」；JointSDK 正是这个现象
（`jsdk_context_warmup`，实测 acks/nacks 恒 0 ⇒ 主机侧无信号），所以先按它的做法查。

实测结论（2026-09-26）：

| 检查 | 结果 |
|---|---|
| 20 轮 `--mode status --no-endpoint-map --trace` + 限时 `candump`，看会话**第一帧**是否上线 | **20/20 都在总线上**（`on_bus=yes`）⇒ 本台架复现不出首帧丢失 |
| 之前那次「StartMotor 不在总线上」 | **我自己的 grep 错了**：29-bit id 带 `Seq[1:0]`，实测该帧是 `0D880406`（seq=2），我却按字面量 `0D880404` 找 ⇒ 假阴性 |
| 机制差异 | JointSDK 的 HAL **每次会话自己开串口**（Lawicel ASCII 帧）；motorbridge 走 **SocketCAN**（`slcand` 长期持有串口，会话只是新建 socket）⇒ 那头一两帧的窗口在本仓库不存在。本仓 transport 只有 socketcan/socketcanfd/pcan/dm-device/dm-serial |

因此**只借鉴了 JointSDK 里真正适用的那一半**：

* **幂等请求 + 重发**（`3801bc3` / `d782108`）—— 这才是丢帧问题的通用解，而丢帧在本台架
  真实存在（用户当初 ~1/10 的连接失败就是一个丢掉的 descriptor 分片造成的）。落地：
  * `CyberBeastMotor::warm_up(500ms)`：发 `QUERY_POS_VEL`（只读、幂等），每轮等
    `WARMUP_ATTEMPT(50ms)`，没答就重发；成功后再调是空操作；**只有新鲜回答**才算数
    （陈旧回答不算，`since` 检查）；失败返回 `Timeout`（**绝不报成功**）；除时间预算外
    另有**轮次上限**（`预算/50 + 2`），时钟不前进也不会死循环；重发次数记入 `tx_retries()`。
  * CLI：**只在 `--no-endpoint-map` 时**、在模式发第一帧之前预热（有端点表时不需要：descriptor
    传输本身就是会话的前几十帧、而且它自带按 offset 重发）；`--mode reset` **不预热**——它是
    用户在节点状态异常时才用的那一帧，必须立刻发出、不能依赖节点回答任何东西（与 JointSDK
    把急停排除在预热之外同一条理由）；`estop` 同理不预热。
  * `read_param_raw`：请求没答 → 在调用方 timeout 内重发（仅当**该 offset 完全没有回答**时
    才重发，避免把慢到的回答与重发的回答叠在一起——响应里不带 offset，无法区分重复分片；
    真出现重复时 `decode_value` 会按声明的宽度报 `Protocol` 错误，不会返回错值）。
* **观测**：`tx_retries()`（预热 + 读重发），`read-param` 在非 0 时打印一行，CLI 预热非 0 时
  打印「session warm-up: N probe(s) re-sent」。

验证（离线注入，`motor_vendors/cyberbeast/tests/endpoint_map_integration.rs` + 单元测试）：

* 仿真器新增 `drophead=N`（丢掉主站最前面 N 帧，JointSDK 同名故障注入）与
  「第 N 个 PARAM_READ 回答不上线」两种注入；
* 用例：丢 1..3 帧 → 预热重发次数 == N，且**之后那条真正重要的读仍然成功且值正确**；
  健康链路 1 次到位、重复调用零帧；只有心跳不算回答；陈旧回答不算回答；全丢 → 有界重发后
  `Timeout` 且报文含节点号；单次丢掉 **param 回答** → 重发后拿到正确值；丢掉分段读的
  **续读回答** → 同 offset 重发、拼出的值仍是设备给的值；节点彻底不答 → 报超时而不是编字节；
* 变异测试（`tools/mutate_warmup.sh`）：关掉重发、关掉读重试、去掉新鲜度检查、去掉
  「已预热即空操作」四处变异**都被用例检出**；另有一处（`record_response` 不过滤心跳）
  在**当前用法下等价**（读取侧也按 msg_type 匹配），脚本里如实标注为“测不出来”，
  没有把它当成测试的功劳。

真机复验（本轮）：`--mode status --no-endpoint-map` 首帧即预热探针 `0x15040404`、
20 轮 0 次丢帧/0 次重发；`read-param`（带表）输出与改动前逐字相同、无重发行；
`--mode reset --yes` 复位后标定/配置标志仍全为 true。

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
