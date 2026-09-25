# CAN 调试指南（PCAN + CANable candleLight/gs_usb）

本文档是本项目通道配置与链路排障的统一手册。

## 1. 范围与后端映射

- Linux 后端：SocketCAN（`can0`、`can1` 等）
- Windows 后端：PEAK PCAN（依赖 `PCANBasic.dll`，`can0/can1` 映射到 `PCAN_USBBUS1/2`）

规则：

- Linux：波特率在网卡初始化时设置，不要写进 `--channel`。
- Windows PCAN：`--channel` 支持可选 `@bitrate` 后缀（如 `can0@1000000`）。

## 2. Linux PCAN 与 CANable candleLight/gs_usb 初始化

### 2.1 识别适配器

```bash
lsusb
lsmod | grep -E 'peak_usb|gs_usb|can_raw|can_dev'
ip -details link show type can
```

期望适配器映射：

- PCAN-USB：内核驱动为 `peak_usb`；使用 `scripts/can_restart.sh can0`。
- CANable candleLight：内核驱动为 `gs_usb`；使用 `scripts/canable_restart.sh can0`。

### 2.2 初始化 SocketCAN 接口

PCAN：

```bash
scripts/can_restart.sh can0
```

CANable candleLight/gs_usb：

```bash
scripts/canable_restart.sh can0
```

然后确认 `can0` 为 `UP`，波特率为 `1000000`，并且 driver 行与当前适配器匹配。

#### 2.2.1 slcan 适配器（厂商 USB2CAN，非 gs_usb 驱动）

部分适配器（例如 CyberBeast/MCS 的 `16d0:117e`「CyberBeast USB2CAN」）在 CDC 串口上
跑 LAWICEL/slcan 固件，而不是 gs_usb 驱动。它们会枚举为 `/dev/ttyACM*`，用 `slcand`
桥接出 SocketCAN 接口，之后 SDK 与使用普通经典 CAN 通道完全一致，填接口名即可。

```bash
sudo modprobe slcan
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0   # -s8 = 1 Mbit/s（-s6 = 500k，-s5 = 250k）
sleep 1
sudo ip link set slcan0 up
ip -details -s link show slcan0             # 期望 link/can、UP、RX/TX errors 为 0
candump -t d slcan0                         # 普通用户即可看到帧
```

注意事项（已在 1 Mbit/s 的 CyberBeast 节点上验证）：

- 设备心跳稳定在 10.00 Hz、丢帧 0，`slcan0` 的 `RX errors / dropped` 保持 0。
- 只有 `slcand` 与 `ip link set up` 需要 root；`candump`、`motor_cli` 不需要。
- 适配器的 CAN 波特率必须与 `-sX` 一致；USB-CDC 适配器可忽略串口波特率
  （只有带真实 UART 的适配器才需要 `-S`）。
- 收尾：`sudo pkill slcand`（或 `sudo slcand -k slcan0`）。
- slcan 是把 CAN 帧当文本行经 USB 串口传输，吞吐需实测（见第 3 节）后再决定能否跑高速控制环。
- `slcand` 不校验适配器协议：`slcan0` 起来了但 `candump` 没帧时，先查接线与波特率，
  再用 `cansend slcan0 15000404#0000000000000000`（CyberBeast 的 QUERY_STATUS）看是否有回帧。

### 2.3 链路最小自检

```bash
candump can0
```

如果扫描或控制时没有帧，优先检查接线、终端电阻、地线参考、供电和电机端波特率一致性。

### 2.4 在 `can0` 上跑 `motor_cli`

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --mode scan --start-id 1 --end-id 16
```

## 3. Linux SocketCAN（`can0`）快检

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

重点观察计数器：

- `RX errors`、`TX errors`、`bus-off`、`re-started`

若 `bus-off` 持续增长，先处理物理层：终端电阻、地线参考、电机波特率一致性。

## 4. Windows PCAN 初始化与验收

### 4.1 前置条件

- 已安装 PEAK 驱动
- 已安装 PCAN-Basic 运行时（`PCANBasic.dll` 可加载）
- 在 PEAK 工具中可见对应 USB 通道

### 4.2 本项目通道约定

- `can0` -> `PCAN_USBBUS1`
- `can1` -> `PCAN_USBBUS2`
- 可选波特率后缀：`can0@1000000`

### 4.3 验证命令

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
```

若报 `load PCANBasic.dll failed`，先解决运行时/DLL 搜索路径问题。

## 5. 报错到动作（Error-to-Action）

### Linux SocketCAN 路径

- `if_nametoindex failed ...`：
  - 通道名错误，或网卡未创建/未拉起
  - 动作：`ip link show`，拉起 `can0` 或选择正确的 SocketCAN 接口
- `socketcan write failed` / `socketcan read failed` 且提示 `interface is down`：
  - 动作：`ip -details link show <ifname>`，再执行 `ip link set <ifname> up`
- `... unavailable` / `interface not found`：
  - 动作：检查 USB-CAN 连接与网卡命名

### Windows PCAN 路径

- `load PCANBasic.dll failed`：
  - 动作：安装 PCAN-Basic，重开终端/IDE 使 DLL 可被加载
- `PCAN initialize failed: status=...`：
  - 动作：核对 `can0/can1` 映射、`@bitrate`、适配器占用状态
- 持续重连失败：
  - 动作：检查线缆、终端电阻、供电和 PEAK 通道占用

## 6. 跨平台最小验收清单

- Linux `can0`：扫描能返回预期电机 ID
- Windows `can0@1000000`：扫描能成功
- 各环境至少执行 1 条控制命令（`mit` 或 `pos-vel`）成功

Linux SocketCAN 与 Windows PCAN 扫描通过，并且至少一条控制命令成功，即可判定通道支持已对齐。

