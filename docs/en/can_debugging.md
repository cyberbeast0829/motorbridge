# CAN Debugging Guide (PCAN + CANable candleLight/gs_usb)

This guide is the canonical troubleshooting playbook for channel setup and link-level diagnostics.

## 1. Scope and Backend Mapping

- Linux backend: SocketCAN (`can0`, `can1`, ...)
- Windows backend: PEAK PCAN via `PCANBasic.dll` (`can0/can1` mapping to `PCAN_USBBUS1/2`)

Rules:

- Linux: configure bitrate at interface bring-up time; do not put `@bitrate` in `--channel`.
- Windows PCAN: `@bitrate` suffix is allowed in `--channel` (for example `can0@1000000`).

## 2. Linux PCAN and CANable candleLight/gs_usb Bring-up

### 2.1 Identify the adapter

```bash
lsusb
lsmod | grep -E 'peak_usb|gs_usb|can_raw|can_dev'
ip -details link show type can
```

Expected adapter mapping:

- PCAN-USB: kernel driver `peak_usb`; use `scripts/can_restart.sh can0`.
- CANable candleLight: kernel driver `gs_usb`; use `scripts/canable_restart.sh can0`.

### 2.2 Initialize the SocketCAN interface

PCAN:

```bash
scripts/can_restart.sh can0
```

CANable candleLight/gs_usb:

```bash
scripts/canable_restart.sh can0
```

Then confirm `can0` is `UP`, bitrate is `1000000`, and the driver line matches the adapter.

#### 2.2.1 slcan adapters (vendor USB2CAN, no gs_usb driver)

Some adapters — for example the CyberBeast/MCS `16d0:117e` "CyberBeast USB2CAN" —
run LAWICEL/slcan firmware over a CDC serial port instead of the gs_usb driver.
They appear as `/dev/ttyACM*` and are bridged to SocketCAN by `slcand`; afterwards
the SDK uses the resulting interface name like any other classic-CAN channel.

```bash
sudo modprobe slcan
sudo slcand -o -c -s8 /dev/ttyACM0 slcan0   # -s8 = 1 Mbit/s (-s6 = 500k, -s5 = 250k)
sleep 1
sudo ip link set slcan0 up
ip -details -s link show slcan0             # expect link/can, UP, RX/TX errors 0
candump -t d slcan0                         # frames appear without root
```

Notes (verified with a CyberBeast node at 1 Mbit/s):

- The device heartbeat arrives at 10.00 Hz with 0 dropped frames, and
  `RX errors / dropped` stay at 0 on `slcan0`.
- Only `slcand` and `ip link set up` need root; `candump` and `motor_cli` do not.
- The adapter's CAN bitrate must match `-sX`. The serial baud rate is irrelevant for
  USB-CDC adapters (use `-S` only for adapters with a real UART).
- Cleanup: `sudo pkill slcand` (or `sudo slcand -k slcan0`).
- slcan carries CAN frames as text lines over USB serial, so treat its throughput
  as a budget to measure (section 3) before running fast control loops.
- `slcand` does not verify the adapter protocol: if `slcan0` comes up but `candump`
  stays silent, check the wiring and bitrate first, then try `cansend slcan0
  15000404#0000000000000000` (a CyberBeast QUERY_STATUS) and watch for a reply.

### 2.3 Quick traffic sanity checks

```bash
candump can0
```

If no frame appears during scan/control, check wiring, termination, ground reference, power, and bitrate consistency with motor firmware.

### 2.4 Run `motor_cli` on `can0`

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --mode scan --start-id 1 --end-id 16
```

## 3. Linux SocketCAN (`can0`) Quick Checks

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Counters to watch:

- `RX errors`, `TX errors`, `bus-off`, `re-started`

If `bus-off` grows, fix physical layer first (termination, ground reference, bitrate mismatch).

## 4. Windows PCAN Bring-up and Verification

### 4.1 Preconditions

- Install PEAK driver
- Install PCAN-Basic runtime (`PCANBasic.dll`) and ensure it is loadable
- Confirm adapter channel availability in PEAK tools

### 4.2 Channel conventions used by this project

- `can0` -> `PCAN_USBBUS1`
- `can1` -> `PCAN_USBBUS2`
- Optional bitrate suffix: `can0@1000000`

### 4.3 Validation commands

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
```

If startup fails with `load PCANBasic.dll failed`, fix PATH/runtime installation first.

## 5. Error-to-Action Map

### Linux SocketCAN path

- `if_nametoindex failed ...`:
  - Interface name is wrong or interface not created/up
  - Action: `ip link show`, bring up `can0` or select the correct SocketCAN interface
- `socketcan write failed` / `socketcan read failed` with hint `interface is down`:
  - Action: `ip -details link show <ifname>`, then bring link up
- `... unavailable` / `interface not found`:
  - Action: check USB adapter presence and interface naming

### Windows PCAN path

- `load PCANBasic.dll failed`:
  - Action: install PCAN-Basic runtime and restart shell/IDE so DLL is discoverable
- `PCAN initialize failed: status=...`:
  - Action: verify channel mapping (`can0/can1`), bitrate suffix, adapter availability
- Repeated reconnect failures:
  - Action: check cable/termination/power and channel occupancy in PEAK tools

## 6. Minimal Cross-Platform Acceptance Checklist

- Linux `can0`: scan command returns expected device IDs
- Windows `can0@1000000`: scan succeeds with PEAK adapter
- One control command (`mit` or `pos-vel`) succeeds in each environment

If Linux SocketCAN and Windows PCAN scans plus one control command pass, channel support is considered aligned.

