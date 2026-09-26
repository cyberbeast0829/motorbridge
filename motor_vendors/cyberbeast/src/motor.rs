use crate::endpoint_map::{decode_value, EndpointMap, ParamReadout, ParamValue, ValueType};
use crate::protocol::{
    self, can_id_parts, encode_clear_errors, encode_config_save, encode_current_control,
    encode_json_desc_read, encode_param_read_at, encode_pos_control, encode_set_zero,
    encode_torque_control, encode_vel_control, make_can_id, pack_mit_command, seq_next,
    unpack_mit_response, CyberBeastCanId, MitCommandParams, MsgType, Priority, ADDR_BROADCAST,
    DEFAULT_MASTER_ID, MAX_BROADCAST_DEVICES,
};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ============================================================================
// Model catalog
// ============================================================================

/// Common ODrive-based motor configurations for CyberBeast protocol.
///
/// These are typical setups. Users may need to adjust P/V/T limits for
/// their specific mechanical configuration (gear ratio, etc.).
const CYBERBEAST_MODELS: &[MotorModelSpec] = &[
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-default",
        pmax: 4.0 * std::f32::consts::PI, // ±4π rad
        vmax: 100.0,                      // ±100 rad/s (output)
        tmax: 10.0,                       // ±10 Nm
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-pro",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 150.0,
        tmax: 20.0,
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-high-torque",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 60.0,
        tmax: 50.0,
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-high-speed",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 300.0,
        tmax: 5.0,
    },
];

const CYBERBEAST_CATALOG: StaticModelCatalog = StaticModelCatalog {
    vendor_name: "cyberbeast",
    models: CYBERBEAST_MODELS,
};

pub fn model_limits(model: &str) -> Option<(f32, f32, f32)> {
    CYBERBEAST_CATALOG
        .get(model)
        .map(|spec| (spec.pmax, spec.vmax, spec.tmax))
}

// ============================================================================
// Control mode
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3,
    Current = 4,
}

// ============================================================================
// Parameter cache for SDO endpoint read/write
// ============================================================================

const DEFAULT_PARAM_TIMEOUT_MS: u64 = 200;
const PARAM_POLL_INTERVAL_MS: u64 = 2;

#[derive(Debug, Clone)]
struct ParamCache {
    /// Cached float values keyed by endpoint_id.
    values: HashMap<u16, f32>,
    /// Timestamp of last response for each endpoint_id.
    reply_time: HashMap<u16, Instant>,
    /// Write acknowledgment: endpoint_id → time of last ack.
    write_ack_time: HashMap<u16, Instant>,
    /// Last response shape per endpoint: (data_len, more, when).
    ///
    /// Used to report "this endpoint is not a 4-byte float32" instead of a bare
    /// timeout, and to follow the More flag of segmented values.
    shapes: HashMap<u16, (u8, bool, Instant)>,
    /// Assembled little-endian value bytes of the last (possibly segmented) read.
    raw: HashMap<u16, Vec<u8>>,
    /// Pending read endpoint (if any).
    pending_read: Option<u16>,
    /// Pending write endpoint (if any).
    pending_write: Option<u16>,
}

impl ParamCache {
    fn new() -> Self {
        Self {
            values: HashMap::new(),
            reply_time: HashMap::new(),
            write_ack_time: HashMap::new(),
            shapes: HashMap::new(),
            raw: HashMap::new(),
            pending_read: None,
            pending_write: None,
        }
    }

    /// Record a PARAM_READ response, assembling segmented values chunk by chunk.
    fn record_read_frame(&mut self, response: &protocol::ParamReadResponse) {
        let now = Instant::now();
        self.shapes.insert(
            response.endpoint_id,
            (response.data_len, response.more, now),
        );
        let entry = self.raw.entry(response.endpoint_id).or_default();
        entry.extend_from_slice(&response.raw[..response.data_len as usize]);
        if !response.more {
            self.reply_time.insert(response.endpoint_id, now);
            if let Some(value) = response.as_f32() {
                self.values.insert(response.endpoint_id, value);
            }
            self.pending_read = None;
        }
    }

    fn record_write_ack(&mut self, endpoint_id: u16) {
        self.write_ack_time.insert(endpoint_id, Instant::now());
        self.pending_write = None;
    }
}

/// Accumulates a JSON endpoint descriptor transfer (protocol 4.8).
#[derive(Debug, Default)]
struct JsonDescCache {
    /// `TotalLength` from the metadata frame, once seen.
    total_len: Option<u32>,
    /// `VersionCRC` from the metadata frame, once seen.
    version_crc: Option<u16>,
    /// Descriptor bytes assembled so far, positioned by `ChunkOffset`.
    bytes: Vec<u8>,
}

impl JsonDescCache {
    /// Start a new transfer.
    fn reset(&mut self) {
        self.total_len = None;
        self.version_crc = None;
        self.bytes.clear();
    }

    /// Record one response frame; returns `true` when the frame was consumed.
    ///
    /// The device repeats the metadata frame after every continuation request, so
    /// metadata is filtered out at any point instead of only at the beginning.
    fn record(&mut self, data: &[u8]) -> bool {
        if let Some((total_len, version_crc)) = protocol::decode_json_desc_meta(data) {
            if self.total_len.is_none() {
                self.total_len = Some(total_len);
                self.version_crc = Some(version_crc);
            }
            return true;
        }
        let Some(chunk) = protocol::decode_json_desc_chunk(data) else {
            return false;
        };
        let start = chunk.offset as usize;
        if start + chunk.data.len() > protocol::JSON_DESC_MAX_BYTES {
            return false;
        }
        if self.bytes.len() < start + chunk.data.len() {
            self.bytes.resize(start + chunk.data.len(), 0);
        }
        self.bytes[start..start + chunk.data.len()].copy_from_slice(chunk.data);
        true
    }
}

// ============================================================================
// Default MIT limits (ODrive CyberBeast defaults, per protocol v2.4)
// ============================================================================

const DEFAULT_MIT_POS_LIMIT: f32 = 4.0 * std::f32::consts::PI; // ±4π rad ≈ 12.566
const DEFAULT_MIT_VEL_LIMIT: f32 = 30.0; // ±30 rad/s
const DEFAULT_MIT_KP_LIMIT: f32 = 500.0; // max Kp (N·m/rad)
const DEFAULT_MIT_KD_LIMIT: f32 = 100.0; // max Kd (N·m·s/rad)
const DEFAULT_MIT_TORQUE_LIMIT: f32 = 18.0; // ±18 N·m
const DEFAULT_MIT_CURRENT_LIMIT: f32 = 40.0; // ± A (for response decoding)

/// Ranges that scale the MIT command and response bit fields.
///
/// The 16-bit/12-bit MIT fields are relative to the **device's own** maxima, which may
/// differ from the protocol defaults: the tested node declares `mit_max_pos` 12.5,
/// `mit_max_vel` 65, `mit_max_torque` 50, `mit_max_kp` 500 and `mit_max_kd` 5. Encoding
/// with the defaults would send a requested 0.05 N·m as 0.139 N·m and shrink Kd by 20x,
/// so [`CyberBeastMotor::probe_mit_ranges`] replaces them with the declared values and
/// [`CyberBeastMotor::send_mit_command`] uses whatever is in force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MitRanges {
    pub pos: f32,
    pub vel: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque: f32,
}

impl Default for MitRanges {
    fn default() -> Self {
        Self {
            pos: DEFAULT_MIT_POS_LIMIT,
            vel: DEFAULT_MIT_VEL_LIMIT,
            kp: DEFAULT_MIT_KP_LIMIT,
            kd: DEFAULT_MIT_KD_LIMIT,
            torque: DEFAULT_MIT_TORQUE_LIMIT,
        }
    }
}

impl MitRanges {
    /// Endpoint names the device uses to declare these maxima, in the order of the
    /// `MitRanges` fields (pos, vel, kp, kd, torque).
    const ENDPOINT_NAMES: [&'static str; 5] = [
        "mit_max_pos",
        "mit_max_vel",
        "mit_max_kp",
        "mit_max_kd",
        "mit_max_torque",
    ];
}

/// Protocol 4.1.2 clamps the MIT response current range at 80 A.
const MAX_MIT_CURRENT_RANGE_A: f32 = 80.0;

/// Endpoint ids verified against the device descriptor; see [`crate::registers::REGISTER_TABLE`].
const ENDPOINT_MIT_MAX_TORQUE: u16 = 0x0151;
const ENDPOINT_MOTOR_TORQUE_CONSTANT: u16 = 0x00F7;

/// Upper bound on continuation requests for one descriptor transfer.
const MAX_JSON_DESC_REQUESTS: u32 = 64;

/// Timeout for each MIT-range read when a motor is connected.
///
/// Short on purpose: a device that answers does so in milliseconds, and one that does
/// not must not add full response timeouts to every connect. The probe is best effort --
/// the documented defaults stay in force and `mit_ranges_note()` explains why -- so the
/// worst case here is 5 x 150 ms on a device that declares nothing.
pub(crate) const DEFAULT_MIT_RANGE_PROBE_TIMEOUT: Duration = Duration::from_millis(150);

/// Prefix an error message while keeping the error kind.
fn with_context(err: MotorError, prefix: &str) -> MotorError {
    match err {
        MotorError::InvalidArgument(m) => MotorError::InvalidArgument(format!("{prefix}{m}")),
        MotorError::Io(m) => MotorError::Io(format!("{prefix}{m}")),
        MotorError::Timeout(m) => MotorError::Timeout(format!("{prefix}{m}")),
        MotorError::Protocol(m) => MotorError::Protocol(format!("{prefix}{m}")),
        MotorError::Unsupported(m) => MotorError::Unsupported(format!("{prefix}{m}")),
    }
}

/// MIT response current range: `mit_max_torque / torque_constant`, clamped to 80 A.
///
/// Returns `None` when the device configuration cannot be used (non-finite or
/// non-positive values); the caller then keeps the documented 40 A default.
fn mit_current_range_from(mit_max_torque: f32, torque_constant: f32) -> Option<f32> {
    if !mit_max_torque.is_finite() || !torque_constant.is_finite() {
        return None;
    }
    if mit_max_torque <= 0.0 || torque_constant <= 0.0 {
        return None;
    }
    Some((mit_max_torque / torque_constant).min(MAX_MIT_CURRENT_RANGE_A))
}

/// Motor-side turns → radians.
///
/// Feedback frames (HEARTBEAT 4.6, QUERY_POS_VEL 4.10.2) report **motor-side**
/// turns / turns per second, while `CyberBeastMotorState` stores radians. Both
/// feedback paths must use this constant so the cached state never changes unit
/// depending on which frame arrived last.
///
/// Note: MIT/POS/VEL/TORQUE commands carry **output-side** units (the firmware
/// applies `× gear_ratio / 2π`), so this motor-side value is only directly
/// comparable with a command target after dividing by the gear ratio. That ratio
/// *is* readable: `axis0.motor.config.gear_ratio` (0x00F2, float rw; 7.75 on the
/// tested node). This crate deliberately does not apply it, so every value it
/// reports stays motor-side.
const MOTOR_TURNS_TO_RAD: f32 = 2.0 * std::f32::consts::PI;

// ============================================================================
// 命令侧电流限兜底值
// ============================================================================

/// `send_pos_control` / `send_vel_control` 的 `cur_limit_a` 兜底值（A）。
///
/// 调用者传入 `<= 0.0`（含 NaN）时视为「未指定」，改用本值。**不要传 0.0 表示「不限制」。**
///
/// # 为什么必须有这个兜底
///
/// 固件把这一参数**直接写进一个跨模式生效的全局配置**：
///
/// ```text
/// axis.motor_.config_.torque_lim = cur_limit_a * torque_constant
///     // can_cyberbeast.cpp:456 (cmd_pos_control) / :492 (cmd_vel_control)
/// ```
///
/// 而 `torque_lim` 是**所有控制模式的公共闸门**：
///
/// ```text
/// controller.cpp:330-331   Tlim = max_available_torque();
///                          torque_setpoint_ = clamp(torque_setpoint_, -Tlim, Tlim);
/// motor.cpp:400-401        max_torque = clamp(max_torque, 0.0, config_.torque_lim);
/// ```
///
/// ⇒ 传入 0 会把闸门拧死成 `clamp(x, 0, 0) == 0`，**连带废掉 MIT / 力矩等所有模式**，
/// 且报错码仍为 `0x00`（无错误），**必须断电重启才能恢复**（该值驻留 RAM，不写 Flash）。
///
/// 已在 2026-09-20 用四阶段对照实验实测复现，详见
/// `docs/cyberbeast-三模式实测报告.md` 第九节。
///
/// # 取值依据
///
/// `200.0` 与 `motor_cli` 的 `--cur-limit` 默认值一致（该值已实测跑通）。
/// 效果上等于「不额外加扭矩限制」—— 固件的 `current_lim`（板级默认 60A，
/// `paras.h:48`）仍会先兜住，因此不会放大过流风险。
pub const DEFAULT_CMD_CURRENT_LIMIT: f32 = 200.0;

/// 把「未指定」的电流限解析成可用值。
///
/// `<= 0.0`（含 NaN）→ [`DEFAULT_CMD_CURRENT_LIMIT`]；否则原样返回。
#[inline]
fn resolve_cur_limit(cur_limit_a: f32) -> f32 {
    if cur_limit_a > 0.0 {
        cur_limit_a
    } else {
        DEFAULT_CMD_CURRENT_LIMIT
    }
}

// ============================================================================
// Motor state
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct CyberBeastMotorState {
    /// The CAN arbitration ID this state was decoded from.
    pub arbitration_id: u32,
    /// Parsed CAN ID fields.
    pub can_id_parts: CyberBeastCanId,
    /// Position in motor-side radians (motor turns × 2π).
    ///
    /// Feedback is reported by the device in motor-side turns (HEARTBEAT,
    /// QUERY_POS_VEL); this struct converts it to radians. Control commands use
    /// output-side units, so divide by the device's `gear_ratio`
    /// (`axis0.motor.config.gear_ratio`, 0x00F2) before comparing with a command
    /// target; this SDK does not apply that conversion itself.
    pub pos: f32,
    /// Velocity in motor-side rad/s (motor turns/s × 2π). See `pos`.
    pub vel: f32,
    /// Motor current in Amps.
    pub current: f32,
    /// Error code from MIT response.
    pub error_code: u8,
    /// Mode/state from MIT response.
    pub mode_state: u8,
    /// Motor temperature in °C.
    pub motor_temp: f32,
    /// MOSFET temperature in °C.
    pub mos_temp: f32,
    /// Heartbeat error flags bitmask (0 if no heartbeat received).
    pub error_flags: u8,
    /// Life counter from last heartbeat frame.
    pub heartbeat_life: u8,
    /// Hardware version (raw uint32) from QUERY_DEVICE_INFO, if queried.
    pub hw_version: Option<u32>,
    /// Firmware version (raw uint32) from QUERY_DEVICE_INFO, if queried.
    pub fw_version: Option<u32>,
    /// Subsystem error value (raw uint32) from QUERY_ERROR, if queried.
    pub error_value: Option<u32>,
}

impl Default for CyberBeastMotorState {
    fn default() -> Self {
        Self {
            arbitration_id: 0,
            can_id_parts: CyberBeastCanId {
                priority: 0,
                msg_type: 0,
                dest: 0,
                source: 0,
                seq: 0,
                is_broadcast: false,
            },
            pos: 0.0,
            vel: 0.0,
            current: 0.0,
            error_code: 0,
            mode_state: 0,
            motor_temp: 0.0,
            mos_temp: 0.0,
            error_flags: 0,
            heartbeat_life: 0,
            hw_version: None,
            fw_version: None,
            error_value: None,
        }
    }
}

// ============================================================================
// CyberBeastMotor
// ============================================================================

pub struct CyberBeastMotor {
    /// CAN node ID of this motor device (destination for commands, source in responses).
    pub motor_id: u16,
    /// Master (host) CAN node ID. Used as source in outgoing frames.
    pub master_id: u8,
    /// Motor model string (must match a catalog entry).
    pub model: String,
    /// Shared CAN bus handle.
    bus: Arc<dyn CanBus>,
    /// Cached latest state from feedback/heartbeat frames.
    state: Mutex<Option<CyberBeastMotorState>>,
    /// Sequence number for outgoing commands (mod 4).
    tx_seq: AtomicU8,
    /// P/V/T limits derived from model catalog.
    #[allow(dead_code)]
    limits: PvTLimits,
    /// MIT encode/decode ranges.
    ///
    /// Protocol defaults until [`Self::probe_mit_ranges`] replaces them with the
    /// device's declared values (that happens when the motor is connected).
    mit_ranges: Mutex<MitRanges>,
    /// Whether `mit_ranges` came from the device (false = protocol defaults).
    mit_ranges_from_device: AtomicBool,
    /// Why the device's MIT ranges are not in force, when that happened.
    mit_ranges_note: Mutex<Option<String>>,
    /// Gear ratio between motor and output shaft (motor-side = output-side x this).
    ///
    /// MIT commands and responses are output-side while every other feedback path is
    /// motor-side, so this ratio is what keeps [`CyberBeastMotorState`] in one unit.
    /// 1.0 means "not declared".
    gear_ratio: AtomicU32,
    /// Whether `gear_ratio` came from the device.
    gear_ratio_from_device: AtomicBool,
    /// Current range used to decode the MIT response current field (A).
    ///
    /// Derived from the device (`mit_max_torque / torque_constant`, clamped to
    /// 80 A) by [`Self::probe_mit_current_range`]; stays at
    /// [`DEFAULT_MIT_CURRENT_LIMIT`] until that is called or when it fails.
    mit_current_limit: AtomicU32,
    /// Parameter cache for SDO endpoint read/write operations.
    param_cache: Mutex<ParamCache>,
    /// JSON endpoint descriptor transfer state (protocol 4.8).
    json_desc: Mutex<JsonDescCache>,
    /// Parsed endpoint map of this node, loaded once when the handle is created.
    ///
    /// Kept in an `Arc` so readers can use the table without holding the lock.
    endpoint_map: Mutex<Option<Arc<EndpointMap>>>,
}

impl CyberBeastMotor {
    pub fn new(
        motor_id: u16,
        _feedback_id: u16,
        model: &str,
        bus: Arc<dyn CanBus>,
    ) -> Result<Self> {
        let spec = CYBERBEAST_CATALOG.get(model).ok_or_else(|| {
            MotorError::InvalidArgument(format!("unknown cyberbeast model: {model}"))
        })?;

        Ok(Self {
            motor_id,
            master_id: DEFAULT_MASTER_ID,
            model: model.to_string(),
            bus,
            state: Mutex::new(None),
            tx_seq: AtomicU8::new(0),
            limits: PvTLimits::from_spec(spec),
            mit_ranges: Mutex::new(MitRanges::default()),
            mit_ranges_from_device: AtomicBool::new(false),
            mit_ranges_note: Mutex::new(None),
            gear_ratio: AtomicU32::new(1.0f32.to_bits()),
            gear_ratio_from_device: AtomicBool::new(false),
            mit_current_limit: AtomicU32::new(DEFAULT_MIT_CURRENT_LIMIT.to_bits()),
            param_cache: Mutex::new(ParamCache::new()),
            json_desc: Mutex::new(JsonDescCache::default()),
            endpoint_map: Mutex::new(None),
        })
    }

    pub fn with_master_id(mut self, master_id: u8) -> Self {
        self.master_id = master_id;
        self
    }

    pub fn set_master_id(&mut self, master_id: u8) {
        self.master_id = master_id;
    }

    pub fn latest_state(&self) -> Option<CyberBeastMotorState> {
        self.state.lock().ok().and_then(|s| *s)
    }

    /// Get or compute the next transmit sequence number.
    fn next_seq(&self) -> u8 {
        let seq = self.tx_seq.load(Ordering::Relaxed);
        self.tx_seq.store(seq_next(seq), Ordering::Relaxed);
        seq
    }

    /// Build a CAN ID for a command to this motor.
    fn cmd_can_id(&self, priority: Priority, msg_type: MsgType) -> u32 {
        make_can_id(
            priority as u8,
            msg_type as u8,
            self.motor_id as u8,
            self.master_id,
            self.next_seq(),
        )
    }

    /// Build a broadcast CAN ID: `Dest=0xFF` means a global broadcast.
    fn bcast_can_id(&self, priority: Priority, msg_type: MsgType) -> u32 {
        // For broadcast, dest encodes a bitmask of target devices.
        // For full broadcast, use 0xFF.
        make_can_id(
            priority as u8,
            msg_type as u8,
            0xFF,
            self.master_id,
            self.next_seq(),
        )
    }

    /// Send a raw CAN frame with extended ID.
    fn send_ext(&self, arbitration_id: u32, data: [u8; 8]) -> Result<()> {
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: 8,
            is_extended: true,
            is_rx: false,
        })
    }

    // ========================================================================
    // Command methods
    // ========================================================================

    /// Send MIT (force-position-velocity) control command.
    ///
    /// Parameters are in output-side units:
    /// - `pos`: target position (rad)
    /// - `vel`: target velocity (rad/s)
    /// - `kp`: position gain
    /// - `kd`: velocity damping gain
    /// - `torque`: feed-forward torque (N·m)
    pub fn send_mit_command(
        &self,
        pos: f32,
        vel: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    ) -> Result<()> {
        let params = MitCommandParams {
            pos,
            vel,
            kp,
            kd,
            torque,
        };
        let ranges = self.mit_ranges();
        let data = pack_mit_command(
            &params,
            ranges.pos,
            ranges.vel,
            ranges.kp,
            ranges.kd,
            ranges.torque,
        );
        let can_id = self.cmd_can_id(Priority::HighCtrl, MsgType::MitControl);
        self.send_ext(can_id, data)
    }

    /// Send position control command (output-side units).
    ///
    /// - `target_pos_deg`: target position in degrees
    /// - `vel_limit_rpm`: velocity limit in RPM
    /// - `cur_limit_a`: current limit in Amps.
    ///   **`<= 0.0`（含 NaN）表示「未指定」，将使用 [`DEFAULT_CMD_CURRENT_LIMIT`]。**
    pub fn send_pos_control(
        &self,
        target_pos_deg: f32,
        vel_limit_rpm: f32,
        cur_limit_a: f32,
    ) -> Result<()> {
        let data = encode_pos_control(
            target_pos_deg,
            vel_limit_rpm,
            resolve_cur_limit(cur_limit_a),
        );
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::PosControl);
        self.send_ext(can_id, data)
    }

    /// Send velocity control command (output-side units).
    ///
    /// - `target_vel_rpm`: target velocity in RPM
    /// - `cur_limit_a`: current limit in Amps.
    ///   **`<= 0.0`（含 NaN）表示「未指定」，将使用 [`DEFAULT_CMD_CURRENT_LIMIT`]。**
    pub fn send_vel_control(&self, target_vel_rpm: f32, cur_limit_a: f32) -> Result<()> {
        let data = encode_vel_control(target_vel_rpm, resolve_cur_limit(cur_limit_a));
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::VelControl);
        self.send_ext(can_id, data)
    }

    /// Send torque control command.
    ///
    /// - `target_torque_nm`: target torque in N·m
    pub fn send_torque_control(&self, target_torque_nm: f32) -> Result<()> {
        let data = encode_torque_control(target_torque_nm);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::TorqueControl);
        self.send_ext(can_id, data)
    }

    /// Send direct current control command.
    ///
    /// - `target_current_a`: target current in Amps
    pub fn send_current_control(&self, target_current_a: f32) -> Result<()> {
        let data = encode_current_control(target_current_a);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::CurrentControl);
        self.send_ext(can_id, data)
    }

    /// Send start motor command (enter closed-loop control).
    pub fn send_start_motor(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::StartMotor);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send stop motor command (enter IDLE).
    pub fn send_stop_motor(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::StopMotor);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send set-zero command (set current position as zero).
    pub fn send_set_zero(&self) -> Result<()> {
        let data = encode_set_zero();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::SetZero);
        self.send_ext(can_id, data)
    }

    /// Send clear-errors command.
    pub fn send_clear_errors(&self) -> Result<()> {
        let data = encode_clear_errors();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ClearErrors);
        self.send_ext(can_id, data)
    }

    /// Send emergency stop.
    ///
    /// Protocol 4.9: ESTOP is a **global broadcast** — `Priority=0 (CRITICAL)`,
    /// `MsgType=0xC0`, `Dest=0xFF`. Every device on the bus enters IDLE and latches
    /// `ERROR_ESTOP_REQUESTED`, which needs CLEAR_ERRORS (0x65) or a reset to clear.
    pub fn send_estop(&self) -> Result<()> {
        let can_id = self.bcast_can_id(Priority::Critical, MsgType::Estop);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send status query (requests MIT response).
    pub fn send_query_status(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryStatus);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send position+velocity query.
    pub fn send_query_pos_vel(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryPosVel);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send current query (Iq + Id).
    pub fn send_query_current(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryCurrent);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send temperature query (motor + FET temp).
    pub fn send_query_temperature(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryTemperature);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send bus voltage/current query.
    pub fn send_query_bus(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryBus);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send device info query (hardware + firmware version).
    pub fn send_query_device_info(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryDeviceInfo);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send detailed error query for a subsystem.
    pub fn send_query_error(&self, err_type: u8) -> Result<()> {
        let data = protocol::encode_query_error(err_type);
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryError);
        self.send_ext(can_id, data)
    }

    /// Request an immediate status feedback (device responds with MIT frame).
    pub fn send_status_feedback(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::StatusFeedback);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send config reset command (erase config and reboot).
    pub fn send_config_reset(&self) -> Result<()> {
        let data = protocol::encode_config_reset();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ConfigReset);
        self.send_ext(can_id, data)
    }

    // ========================================================================
    // Parameter (SDO endpoint) access
    // ========================================================================

    /// Send a parameter read request for an ODrive SDO endpoint (value offset 0).
    pub fn send_param_read(&self, endpoint_id: u16) -> Result<()> {
        self.send_param_read_at(endpoint_id, 0)
    }

    /// Send a PARAM_READ request for a specific byte offset inside the value.
    ///
    /// Offset 0 starts a new read sequence and drops everything the previous read
    /// assembled. That matters when the same endpoint is read twice: the cached
    /// value/reply/shape of the earlier read must never be mistaken for this read's
    /// answer, otherwise a second read returns a stale value instantly instead of
    /// asking the device (and a segmented value would concat onto the previous one).
    /// Later offsets continue a segmented read without clearing.
    pub fn send_param_read_at(&self, endpoint_id: u16, offset: u32) -> Result<()> {
        {
            let mut cache = self
                .param_cache
                .lock()
                .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
            cache.pending_read = Some(endpoint_id);
            if offset == 0 {
                cache.raw.remove(&endpoint_id);
                cache.shapes.remove(&endpoint_id);
                cache.values.remove(&endpoint_id);
                cache.reply_time.remove(&endpoint_id);
            }
        }
        let data = encode_param_read_at(endpoint_id, offset);
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ParamRead);
        self.send_ext(can_id, data)
    }

    /// Send a parameter write request for an ODrive SDO endpoint (float32 value).
    pub fn send_param_write(&self, endpoint_id: u16, value: f32) -> Result<()> {
        self.send_param_write_bytes(endpoint_id, &value.to_le_bytes())
    }

    /// Write explicit value bytes to an endpoint.
    ///
    /// Escape hatch for firmware quirks: the caller decides the payload (little-endian, as
    /// the firmware stores it), while the device's table still decides whether the endpoint
    /// may be written. At most 4 value bytes fit in a classic-CAN frame.
    pub fn set_param_bytes(&self, endpoint_id: u16, raw: &[u8]) -> Result<()> {
        if raw.len() > 4 {
            return Err(MotorError::InvalidArgument(format!(
                "{} value bytes; a classic-CAN PARAM_WRITE carries at most 4",
                raw.len()
            )));
        }
        self.check_writable(endpoint_id, None)?;
        self.send_param_write_bytes(endpoint_id, raw)?;
        self.wait_for_write_ack(endpoint_id, Duration::from_millis(DEFAULT_PARAM_TIMEOUT_MS))
    }

    /// Send a PARAM_WRITE request with an explicit value width (big-endian, per protocol 4.7).
    fn send_param_write_bytes(&self, endpoint_id: u16, value: &[u8]) -> Result<()> {
        {
            let mut cache = self
                .param_cache
                .lock()
                .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
            cache.pending_write = Some(endpoint_id);
            // A fresh write must not be "acknowledged" by the previous write's ack.
            cache.write_ack_time.remove(&endpoint_id);
        }
        let data = protocol::encode_param_write_bytes(endpoint_id, value);
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ParamWrite);
        self.send_ext(can_id, data)
    }

    /// Wait for the value of an SDO endpoint that has already been requested.
    ///
    /// This does **not** transmit anything: call [`Self::send_param_read`] first (the
    /// ABI layer does exactly that). Responses are cached by the background polling
    /// thread, so calling this without a preceding request always times out.
    ///
    /// Only 4-byte float32 endpoints are supported here. When the device answers
    /// with a different value width (or with a segmented value), the error is
    /// reported as `Unsupported` with the observed `DataLen` instead of a timeout.
    pub fn get_param_f32(&self, endpoint_id: u16, timeout: Duration) -> Result<f32> {
        let since = Instant::now();
        match self.wait_for_param(endpoint_id, timeout) {
            Ok(value) => Ok(value),
            Err(err) => {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                match cache.shapes.get(&endpoint_id) {
                    Some((data_len, more, when)) if *when >= since => {
                        Err(MotorError::Unsupported(format!(
                            "endpoint 0x{endpoint_id:04X} answered with {data_len} byte(s){}; \
                             get_param_f32 handles 4-byte float32 endpoints only \
                             (use read_param_raw for other widths)",
                            if *more {
                                " and the More flag set (segmented value)"
                            } else {
                                ""
                            }
                        )))
                    }
                    _ => Err(err),
                }
            }
        }
    }

    /// Read an SDO endpoint as raw little-endian bytes, following the More flag.
    ///
    /// Unlike [`Self::get_param_f32`] this sends the first request itself, so it can
    /// read any value width (uint8/uint16/uint32/uint64/float32/float64).
    pub fn read_param_raw(&self, endpoint_id: u16, timeout: Duration) -> Result<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        let mut requested_offset = 0usize;
        self.send_param_read(endpoint_id)?;
        loop {
            let snapshot = {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                cache
                    .raw
                    .get(&endpoint_id)
                    .cloned()
                    .zip(cache.shapes.get(&endpoint_id).map(|shape| shape.1))
            };
            if let Some((bytes, more)) = snapshot {
                if !more {
                    return Ok(bytes);
                }
                if bytes.len() > requested_offset {
                    requested_offset = bytes.len();
                    self.send_param_read_at(endpoint_id, requested_offset as u32)?;
                }
            }
            if Instant::now() > deadline {
                return Err(MotorError::Timeout(format!(
                    "timeout waiting for param read 0x{endpoint_id:04X}",
                )));
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }
    }

    /// Read a float32 SDO endpoint, sending the request first.
    pub fn read_param_f32(&self, endpoint_id: u16, timeout: Duration) -> Result<f32> {
        let bytes = self.read_param_raw(endpoint_id, timeout)?;
        let raw: [u8; 4] = bytes.as_slice().try_into().map_err(|_| {
            MotorError::Unsupported(format!(
                "endpoint 0x{endpoint_id:04X} returned {} byte(s), expected 4 (float32)",
                bytes.len()
            ))
        })?;
        Ok(f32::from_le_bytes(raw))
    }

    /// Send a JSON_DESC_READ request for a descriptor byte offset (protocol 4.8).
    pub fn send_json_desc_read(&self, offset: u32) -> Result<()> {
        let data = encode_json_desc_read(offset);
        let can_id = self.cmd_can_id(Priority::Config, MsgType::JsonDescRead);
        self.send_ext(can_id, data)
    }

    /// Fetch the device's JSON endpoint descriptor and return it as text.
    ///
    /// Returns `(total_len, version_crc, json)`; see
    /// [`Self::read_endpoint_descriptor_raw`] for the raw bytes.
    pub fn read_endpoint_descriptor(&self, timeout: Duration) -> Result<(u32, u16, String)> {
        let (total_len, version_crc, bytes) = self.read_endpoint_descriptor_raw(timeout)?;
        let json = String::from_utf8(bytes).map_err(|err| {
            MotorError::Protocol(format!("endpoint descriptor is not valid UTF-8: {err}"))
        })?;
        Ok((total_len, version_crc, json))
    }

    /// Fetch the device's JSON endpoint descriptor as raw bytes.
    ///
    /// Sends `JSON_DESC_READ` (0x24) and reassembles the `JSON_DESC_DATA` (0x25)
    /// frames: one metadata frame plus chunks of 6 JSON bytes. The device streams a
    /// large burst for a single request, so a continuation request is only sent once
    /// the device has gone quiet for `timeout` and the descriptor is still incomplete.
    /// `timeout` therefore bounds the quiet window, not the whole transfer.
    pub fn read_endpoint_descriptor_raw(&self, timeout: Duration) -> Result<(u32, u16, Vec<u8>)> {
        {
            let mut cache = self
                .json_desc
                .lock()
                .map_err(|_| MotorError::Io("json descriptor lock poisoned".into()))?;
            cache.reset();
        }
        self.send_json_desc_read(0)?;

        let mut last_received = 0usize;
        let mut quiet_deadline = Instant::now() + timeout;
        let mut requests = 1u32;
        loop {
            let (received, total_len) = {
                let cache = self
                    .json_desc
                    .lock()
                    .map_err(|_| MotorError::Io("json descriptor lock poisoned".into()))?;
                (cache.bytes.len(), cache.total_len)
            };
            if let Some(total_len) = total_len {
                if received >= total_len as usize {
                    break;
                }
            }
            if received > last_received {
                // Still streaming: keep the window open.
                last_received = received;
                quiet_deadline = Instant::now() + timeout;
            } else if Instant::now() > quiet_deadline {
                if total_len.is_none() {
                    // Nothing at all came back: stop instead of retrying 64 times,
                    // which would turn a wrong node id into a ~30 s stall.
                    return Err(MotorError::Timeout(format!(
                        "no JSON descriptor metadata from node 0x{:02X} within {timeout:?} \
                         (wrong node id, silent node, or no access to the endpoint map?)",
                        self.motor_id
                    )));
                }
                if requests >= MAX_JSON_DESC_REQUESTS {
                    return Err(MotorError::Timeout(format!(
                        "endpoint descriptor transfer gave up after {requests} requests ({received} of {} bytes)",
                        total_len.unwrap_or(0)
                    )));
                }
                requests += 1;
                self.send_json_desc_read(received as u32)?;
                quiet_deadline = Instant::now() + timeout;
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }

        let (total_len, version_crc, bytes) = {
            let cache = self
                .json_desc
                .lock()
                .map_err(|_| MotorError::Io("json descriptor lock poisoned".into()))?;
            (
                cache.total_len.unwrap_or(0),
                cache.version_crc.unwrap_or(0),
                cache.bytes.clone(),
            )
        };
        let bytes = bytes[..total_len as usize].to_vec();
        Ok((total_len, version_crc, bytes))
    }

    /// Read the device's endpoint descriptor, parse it and cache the resulting table.
    ///
    /// Callers usually do not need this: adding a motor through
    /// [`crate::CyberBeastController::add_motor`] loads the map, so `read_param` and
    /// `write_param` always have the device's complete table available. Call it again
    /// to pick up a changed map, for example after a firmware update.
    pub fn load_endpoint_map(&self, timeout: Duration) -> Result<Arc<EndpointMap>> {
        let (total_len, version_crc, json) = self.read_endpoint_descriptor(timeout)?;
        let map = Arc::new(EndpointMap::parse(&json)?.with_metadata(total_len, version_crc));
        if map.is_empty() {
            return Err(MotorError::Protocol(format!(
                "endpoint descriptor ({total_len} bytes, VersionCRC=0x{version_crc:04X}) describes no endpoints"
            )));
        }
        let mut slot = self
            .endpoint_map
            .lock()
            .map_err(|_| MotorError::Io("endpoint map lock poisoned".into()))?;
        *slot = Some(Arc::clone(&map));
        Ok(map)
    }

    /// The cached endpoint map, loading it first when it has not been loaded yet.
    pub fn ensure_endpoint_map(&self, timeout: Duration) -> Result<Arc<EndpointMap>> {
        match self.endpoint_map() {
            Some(map) => Ok(map),
            None => self.load_endpoint_map(timeout),
        }
    }

    /// The cached endpoint map, if one has been loaded.
    pub fn endpoint_map(&self) -> Option<Arc<EndpointMap>> {
        self.endpoint_map
            .lock()
            .ok()
            .and_then(|slot| slot.as_ref().map(Arc::clone))
    }

    /// Read an endpoint with the type the device declares for it.
    ///
    /// The cached endpoint map supplies the value type and the dotted path, so the
    /// result is typed instead of guessed (`uint32`, `bool` and 8-byte endpoints
    /// included). An id the device does not have is `InvalidArgument`, and endpoints
    /// declared as functions or references are `Unsupported`.
    pub fn read_param_value(&self, endpoint_id: u16, timeout: Duration) -> Result<ParamReadout> {
        let map = self.endpoint_map().ok_or_else(|| {
            MotorError::Unsupported(
                "the endpoint map of this motor is not loaded; add it through \
                 CyberBeastController::add_motor (or call load_endpoint_map) before reading parameters"
                    .to_string(),
            )
        })?;
        let entry = map.get(endpoint_id).ok_or_else(|| {
            MotorError::InvalidArgument(format!(
                "endpoint 0x{endpoint_id:04X} ({endpoint_id}) is not in the device's endpoint map ({} entries, VersionCRC=0x{:04X})",
                map.len(),
                map.version_crc()
            ))
        })?;
        let value_type = entry.value_type().ok_or_else(|| {
            MotorError::Unsupported(format!(
                "endpoint 0x{endpoint_id:04X} ({}) is declared \"{}\" and cannot be read with PARAM_READ",
                entry.path, entry.raw_type
            ))
        })?;
        let raw = self.read_param_raw(endpoint_id, timeout)?;
        let value = decode_value(&raw, value_type)?;
        Ok(ParamReadout {
            endpoint_id,
            path: Some(entry.path.clone()),
            declared: entry.raw_type.clone(),
            access: entry.access,
            value,
            raw,
        })
    }

    /// Current range used to decode MIT response current values (A).
    pub fn mit_current_limit(&self) -> f32 {
        f32::from_bits(self.mit_current_limit.load(Ordering::Relaxed))
    }

    /// MIT ranges currently used to encode commands and decode responses.
    pub fn mit_ranges(&self) -> MitRanges {
        self.mit_ranges
            .lock()
            .map(|ranges| *ranges)
            .unwrap_or_default()
    }

    /// Whether [`Self::mit_ranges`] came from the device's own declarations.
    ///
    /// `false` means the protocol defaults are in force, in which case every MIT field
    /// is scaled with the wrong denominator; [`Self::mit_ranges_note`] says why.
    pub fn mit_ranges_from_device(&self) -> bool {
        self.mit_ranges_from_device.load(Ordering::Relaxed)
    }

    /// Why the device's MIT ranges are not in force, when the probe did not succeed.
    pub fn mit_ranges_note(&self) -> Option<String> {
        self.mit_ranges_note
            .lock()
            .ok()
            .and_then(|note| note.clone())
    }

    /// The device's torque maximum used to scale MIT commands (N·m).
    pub fn mit_torque_limit(&self) -> f32 {
        self.mit_ranges().torque
    }

    /// Gear ratio between the motor and the output shaft.
    ///
    /// Motor-side radians = output-side radians x this. 1.0 means the device did not
    /// declare one, in which case nothing is scaled (and the MIT response keeps its
    /// output-side units).
    pub fn gear_ratio(&self) -> f32 {
        f32::from_bits(self.gear_ratio.load(Ordering::Relaxed))
    }

    /// Whether [`Self::gear_ratio`] came from the device.
    pub fn gear_ratio_from_device(&self) -> bool {
        self.gear_ratio_from_device.load(Ordering::Relaxed)
    }

    /// Read the device's gear ratio and use it to convert MIT responses.
    ///
    /// Measured on hardware: commanding a 0.1 output rad step moved the motor 0.7506 rad
    /// while the MIT response settled at 0.0973 output rad, i.e. a ratio of 7.71 against the
    /// declared 7.75. The endpoint is resolved by name in the loaded map, so the id is not
    /// hardcoded.
    pub fn probe_gear_ratio(&self, timeout: Duration) -> Result<f32> {
        let map = self.endpoint_map().ok_or_else(|| {
            MotorError::Unsupported(
                "the endpoint map of this motor is not loaded, so the gear ratio cannot be 
                 resolved by name"
                    .to_string(),
            )
        })?;
        let entry = map
            .resolve("gear_ratio")
            .map_err(|err| MotorError::Unsupported(format!("gear ratio endpoint: {err}")))?;
        let readout = self
            .read_param_value(entry.endpoint_id, timeout)
            .map_err(|err| {
                with_context(
                    err,
                    &format!(
                        "gear ratio endpoint \"{}\" (0x{:04X}): ",
                        entry.path, entry.endpoint_id
                    ),
                )
            })?;
        let ratio = readout.value.as_f32().ok_or_else(|| {
            MotorError::Unsupported(format!(
                "endpoint {} is declared \"{}\", not a float",
                entry.path, readout.declared
            ))
        })?;
        if !ratio.is_finite() || ratio <= 0.0 {
            return Err(MotorError::Unsupported(format!(
                "endpoint {} declares {ratio}, which cannot be a gear ratio",
                entry.path
            )));
        }
        self.gear_ratio.store(ratio.to_bits(), Ordering::Relaxed);
        self.gear_ratio_from_device.store(true, Ordering::Relaxed);
        Ok(ratio)
    }

    /// Read the device's declared MIT ranges and use them for encoding and decoding.
    ///
    /// The MIT bit fields are relative to the device's maxima, so this is what makes a
    /// commanded torque, velocity or Kd mean what the caller asked for. The endpoints
    /// are resolved **by name** in the loaded endpoint map (no hardcoded ids) and all
    /// five have to be declared and usable; otherwise the documented defaults stay in
    /// force and this returns an error naming the endpoint that failed.
    pub fn probe_mit_ranges(&self, timeout: Duration) -> Result<MitRanges> {
        let map = self.endpoint_map().ok_or_else(|| {
            MotorError::Unsupported(
                "the endpoint map of this motor is not loaded, so the MIT ranges cannot be 
                 resolved by name"
                    .to_string(),
            )
        })?;
        let mut declared = [0f32; MitRanges::ENDPOINT_NAMES.len()];
        for (index, name) in MitRanges::ENDPOINT_NAMES.iter().enumerate() {
            let entry = map.resolve(name).map_err(|err| {
                MotorError::Unsupported(format!("MIT range endpoint \"{name}\": {err}"))
            })?;
            let readout = self
                .read_param_value(entry.endpoint_id, timeout)
                .map_err(|err| {
                    with_context(
                        err,
                        &format!(
                            "MIT range endpoint \"{name}\" (0x{:04X}): ",
                            entry.endpoint_id
                        ),
                    )
                })?;
            let value = readout.value.as_f32().ok_or_else(|| {
                MotorError::Unsupported(format!(
                    "endpoint {name} is declared \"{}\", not a float, so it cannot scale the MIT fields",
                    readout.declared
                ))
            })?;
            if !value.is_finite() || value <= 0.0 {
                return Err(MotorError::Unsupported(format!(
                    "endpoint {name} declares {value}, which cannot scale an MIT field"
                )));
            }
            declared[index] = value;
        }
        let ranges = MitRanges {
            pos: declared[0],
            vel: declared[1],
            kp: declared[2],
            kd: declared[3],
            torque: declared[4],
        };
        if let Ok(mut slot) = self.mit_ranges.lock() {
            *slot = ranges;
        }
        self.mit_ranges_from_device.store(true, Ordering::Relaxed);
        if let Ok(mut note) = self.mit_ranges_note.lock() {
            *note = None;
        }
        Ok(ranges)
    }

    /// Keep the documented MIT defaults and record why the device's were not used.
    pub(crate) fn note_mit_ranges_failure(&self, reason: String) {
        self.mit_ranges_from_device.store(false, Ordering::Relaxed);
        if let Ok(mut note) = self.mit_ranges_note.lock() {
            *note = Some(reason);
        }
    }

    /// Override the MIT response current range (A) without probing the device.
    pub fn set_mit_current_limit(&self, limit_a: f32) {
        self.mit_current_limit
            .store(limit_a.to_bits(), Ordering::Relaxed);
    }

    /// Derive the MIT response current range from the device configuration.
    ///
    /// Protocol 4.1.2: the MIT response encodes current over
    /// `max_current = mit_max_torque / torque_constant`, clamped to 80 A, while
    /// devices with an invalid torque constant use 40 A. Reads
    /// `axis0.controller.config.mit_max_torque` (0x0151) and
    /// `axis0.motor.config.torque_constant` (0x00F7) so `state.current` is scaled by
    /// the device's real range instead of the fallback default.
    ///
    /// Returns the applied range, or an error (keeping the fallback) when the
    /// endpoints cannot be read or their values are unusable.
    pub fn probe_mit_current_range(&self, timeout: Duration) -> Result<f32> {
        let mit_max_torque = self.read_param_f32(ENDPOINT_MIT_MAX_TORQUE, timeout)?;
        let torque_constant = self.read_param_f32(ENDPOINT_MOTOR_TORQUE_CONSTANT, timeout)?;
        match mit_current_range_from(mit_max_torque, torque_constant) {
            Some(range) => {
                self.set_mit_current_limit(range);
                Ok(range)
            }
            None => {
                self.set_mit_current_limit(DEFAULT_MIT_CURRENT_LIMIT);
                Err(MotorError::Unsupported(format!(
                    "cannot derive the MIT response current range from mit_max_torque={mit_max_torque} Nm and torque_constant={torque_constant} Nm/A; keeping the {DEFAULT_MIT_CURRENT_LIMIT} A fallback"
                )))
            }
        }
    }

    /// Write a float32 parameter value and wait for acknowledgment.
    ///
    /// When the endpoint map is loaded, the device's own declaration is enforced: a
    /// four-byte value is no longer sent to a narrower endpoint (use
    /// [`Self::set_param_value`] for those).
    pub fn set_param_f32(&self, endpoint_id: u16, value: f32) -> Result<()> {
        self.check_writable(endpoint_id, Some(ValueType::F32))?;
        self.send_param_write(endpoint_id, value)?;
        self.wait_for_write_ack(endpoint_id, Duration::from_millis(DEFAULT_PARAM_TIMEOUT_MS))
    }

    /// Write a value with the width the device declares for the endpoint.
    ///
    /// This is what makes non-float endpoints writable at all: `axis0.requested_state` is
    /// one byte wide, so it must be written with one value byte. The declared type has to
    /// match the value, and the endpoint map has to be loaded (connected motors have it),
    /// because the width comes from the device's own declaration.
    pub fn set_param_value(&self, endpoint_id: u16, value: ParamValue) -> Result<()> {
        let declared = self.check_writable(endpoint_id, None)?.ok_or_else(|| {
            MotorError::Unsupported(
                "the endpoint map of this motor is not loaded, so the width the device 
                 declares for this endpoint is unknown; connect through 
                 CyberBeastController::add_motor or use set_param_f32"
                    .to_string(),
            )
        })?;
        if !value.matches_type(declared) {
            return Err(MotorError::InvalidArgument(format!(
                "endpoint 0x{endpoint_id:04X} is declared \"{}\" but the value to write is {value:?}",
                declared.label()
            )));
        }
        if declared.byte_width() > 4 {
            return Err(MotorError::Unsupported(format!(
                "endpoint 0x{endpoint_id:04X} is declared {} ({} bytes); a classic-CAN 
                 PARAM_WRITE carries at most 4 value bytes",
                declared.label(),
                declared.byte_width()
            )));
        }
        self.send_param_write_bytes(endpoint_id, &value.to_write_bytes())?;
        self.wait_for_write_ack(endpoint_id, Duration::from_millis(DEFAULT_PARAM_TIMEOUT_MS))
    }

    /// Check the device's own table for a write when it is loaded.
    ///
    /// Returns the declared type (`None` when no map is loaded, which keeps the older
    /// permissive behaviour for probe handles). Enforces existence, writability and --
    /// when `expected` is given -- the declared type.
    fn check_writable(
        &self,
        endpoint_id: u16,
        expected: Option<ValueType>,
    ) -> Result<Option<ValueType>> {
        let Some(map) = self.endpoint_map() else {
            return Ok(None);
        };
        let entry = map.get(endpoint_id).ok_or_else(|| {
            MotorError::InvalidArgument(format!(
                "endpoint 0x{endpoint_id:04X} ({endpoint_id}) is not in the device's endpoint map ({} entries)",
                map.len()
            ))
        })?;
        if !entry.access.is_writable() {
            return Err(MotorError::InvalidArgument(format!(
                "endpoint 0x{endpoint_id:04X} ({}) is declared access=\"{}\"; refusing to write it",
                entry.path,
                entry.access.label()
            )));
        }
        let declared = entry.value_type().ok_or_else(|| {
            MotorError::Unsupported(format!(
                "endpoint 0x{endpoint_id:04X} ({}) is declared \"{}\" and cannot be written with PARAM_WRITE",
                entry.path, entry.raw_type
            ))
        })?;
        if let Some(expected) = expected {
            if declared != expected {
                return Err(MotorError::InvalidArgument(format!(
                    "endpoint 0x{endpoint_id:04X} ({}) is declared \"{}\", not \"{}\"",
                    entry.path,
                    declared.label(),
                    expected.label()
                )));
            }
        }
        Ok(Some(declared))
    }

    /// Store parameters to flash (CONFIG_SAVE).
    pub fn store_parameters(&self) -> Result<()> {
        let data = encode_config_save();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ConfigSave);
        self.send_ext(can_id, data)
    }

    /// Block until a param read response arrives or timeout.
    fn wait_for_param(&self, endpoint_id: u16, timeout: Duration) -> Result<f32> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                if let Some(ts) = cache.reply_time.get(&endpoint_id) {
                    if *ts > Instant::now() - timeout {
                        // Fresh value
                        if let Some(val) = cache.values.get(&endpoint_id) {
                            return Ok(*val);
                        }
                    }
                }
            }
            if Instant::now() > deadline {
                return Err(MotorError::Timeout(format!(
                    "timeout waiting for param read 0x{endpoint_id:04X}",
                )));
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }
    }

    /// Block until a param write ack arrives or timeout.
    fn wait_for_write_ack(&self, endpoint_id: u16, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                if let Some(ts) = cache.write_ack_time.get(&endpoint_id) {
                    if *ts > Instant::now() - timeout {
                        return Ok(());
                    }
                }
            }
            if Instant::now() > deadline {
                return Err(MotorError::Timeout(format!(
                    "timeout waiting for param write ack 0x{endpoint_id:04X}",
                )));
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }
    }

    // ========================================================================
    // Feedback processing
    // ========================================================================

    /// Process an incoming frame: decode MIT response, heartbeat, or other feedback.
    fn process_feedback_frame_impl(&self, frame: CanFrame) -> Result<()> {
        let parts = can_id_parts(frame.arbitration_id);

        match parts.msg_type {
            // MIT response: reused MIT control msg type from motor → host
            t if t == MsgType::MitControl as u8 => {
                let ranges = self.mit_ranges();
                let resp = unpack_mit_response(
                    &frame.data,
                    ranges.pos,
                    ranges.vel,
                    self.mit_current_limit(),
                );
                // MIT responses carry **output-side** units while the cached state is
                // motor-side like every other feedback path, so scale by the device's
                // gear ratio (measured: 0.0973 output rad matched 0.7506 motor rad).
                let gear = self.gear_ratio();

                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();

                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    pos: resp.pos * gear,
                    vel: resp.vel * gear,
                    current: resp.current,
                    error_code: resp.error_code,
                    mode_state: resp.mode_state,
                    motor_temp: resp.motor_temp,
                    mos_temp: resp.mos_temp,
                    error_flags: existing.error_flags,
                    heartbeat_life: existing.heartbeat_life,
                    hw_version: existing.hw_version,
                    fw_version: existing.fw_version,
                    error_value: existing.error_value,
                });
            }

            // Heartbeat
            t if t == MsgType::Heartbeat as u8 => {
                if let Some(hb) = protocol::decode_heartbeat(&frame.data) {
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();

                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        // Heartbeat now provides position, velocity, current,
                        // temperature, error flags, and life counter all in one frame.
                        // Position/velocity are motor-side turns → radians.
                        pos: hb.position_turns * MOTOR_TURNS_TO_RAD,
                        vel: hb.velocity_turns_per_s * MOTOR_TURNS_TO_RAD,
                        current: hb.iq_current,
                        error_flags: hb.error_flags,
                        motor_temp: hb.motor_temp,
                        mos_temp: existing.mos_temp,
                        heartbeat_life: hb.life_counter,
                        ..existing
                    });
                }
            }

            // QUERY_POS_VEL response
            t if t == MsgType::QueryPosVel as u8 => {
                // QUERY_POS_VEL answers with motor-side turns / turns per second
                // (protocol 4.10.2). Convert like the heartbeat path does, so the
                // cached state never depends on which frame arrived last.
                let (pos_turns, vel_turns_per_s) = protocol::decode_pos_vel_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    pos: pos_turns * MOTOR_TURNS_TO_RAD,
                    vel: vel_turns_per_s * MOTOR_TURNS_TO_RAD,
                    ..existing
                });
            }

            // QUERY_CURRENT response
            t if t == MsgType::QueryCurrent as u8 => {
                let (iq, _id) = protocol::decode_current_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    current: iq,
                    ..existing
                });
            }

            // QUERY_TEMPERATURE response
            t if t == MsgType::QueryTemperature as u8 => {
                let (motor_temp, mos_temp) = protocol::decode_temperature_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    motor_temp,
                    mos_temp,
                    ..existing
                });
            }

            // QUERY_BUS response
            t if t == MsgType::QueryBus as u8 => {
                // Bus voltage/current — stored in state for now (could add dedicated fields)
                let _ = protocol::decode_bus_response(&frame.data);
            }

            // QUERY_DEVICE_INFO response (Classic CAN: HW + FW version)
            t if t == MsgType::QueryDeviceInfo as u8 => {
                if let Some(info) = protocol::decode_device_info_response(&frame.data) {
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();
                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        hw_version: Some(info.hw_version),
                        fw_version: Some(info.fw_version),
                        ..existing
                    });
                }
            }

            // QUERY_ERROR response: Byte 0=type echo, Byte 4-7=error value (uint32 BE)
            t if t == MsgType::QueryError as u8 => {
                if frame.data.len() >= 8 {
                    let value = ((frame.data[4] as u32) << 24)
                        | ((frame.data[5] as u32) << 16)
                        | ((frame.data[6] as u32) << 8)
                        | (frame.data[7] as u32);
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();
                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        error_value: Some(value),
                        ..existing
                    });
                }
            }

            // PARAM_READ response
            t if t == MsgType::ParamRead as u8 => {
                if let Some(response) = protocol::decode_param_read_frame(&frame.data) {
                    let mut cache = self
                        .param_cache
                        .lock()
                        .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                    cache.record_read_frame(&response);
                }
            }

            // PARAM_WRITE acknowledgment
            t if t == MsgType::ParamWrite as u8 && frame.data.len() >= 3 => {
                let endpoint_id = ((frame.data[1] as u16) << 8) | (frame.data[2] as u16);
                if protocol::is_param_write_ack(&frame.data, endpoint_id) {
                    let mut cache = self
                        .param_cache
                        .lock()
                        .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                    cache.record_write_ack(endpoint_id);
                }
            }

            // JSON_DESC_DATA: descriptor metadata frame or one descriptor chunk
            t if t == MsgType::JsonDescData as u8 => {
                let mut cache = self
                    .json_desc
                    .lock()
                    .map_err(|_| MotorError::Io("json descriptor lock poisoned".into()))?;
                cache.record(&frame.data);
            }

            _ => {
                // Unknown response — ignore
            }
        }

        Ok(())
    }

    /// Check if a broadcast CAN frame targets this motor.
    ///
    /// Protocol v2.4 broadcast semantics:
    /// - `Dest=0xFF`: global broadcast → reaches ALL devices (no bitmap limit)
    /// - `Dest=bitmap`: multicast to Device#0~7 via 8-bit bitmap (bit0=Dev#0)
    /// - In Classic CAN, MIT broadcast applies the same 8-byte command to every
    ///   bitmap-matched device (no per-device slots, which require CAN FD).
    fn is_broadcast_slot_for_me(&self, parts: &CyberBeastCanId) -> bool {
        let device_id = self.motor_id as u8;
        if device_id == 0 {
            return false;
        }
        // Dest=0xFF is global broadcast — every device responds/applies.
        if parts.dest == ADDR_BROADCAST {
            return true;
        }
        // Bitmap multicast only addresses Device#0~7.
        if device_id >= MAX_BROADCAST_DEVICES {
            return false;
        }
        (parts.dest & (1 << device_id)) != 0
    }
}

// ============================================================================
// MotorDevice trait impl
// ============================================================================

impl MotorDevice for CyberBeastMotor {
    fn vendor(&self) -> &'static str {
        "cyberbeast"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        // In CyberBeast protocol, feedback frames use the motor's node_id
        // as the source address. Same as motor_id.
        self.motor_id
    }

    fn enable(&self) -> Result<()> {
        self.send_start_motor()
    }

    fn disable(&self) -> Result<()> {
        self.send_stop_motor()
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_rx {
            return false;
        }
        let parts = can_id_parts(frame.arbitration_id);

        // Point-to-point: source matches our motor_id
        if !parts.is_broadcast {
            return parts.source == self.motor_id as u8;
        }

        // Broadcast: check bitmask in dest field
        self.is_broadcast_slot_for_me(&parts)
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        self.process_feedback_frame_impl(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mit_current_range_follows_the_device_formula() {
        // Observed on hardware: mit_max_torque 50 Nm / torque_constant 0.0824 Nm/A
        // = 606 A, clamped to the documented 80 A ceiling.
        assert_eq!(mit_current_range_from(50.0, 0.0824), Some(80.0));
        assert_eq!(mit_current_range_from(5.0, 0.1), Some(50.0));
        assert_eq!(mit_current_range_from(5.0, 0.0), None);
        assert_eq!(mit_current_range_from(0.0, 0.1), None);
        assert_eq!(mit_current_range_from(f32::NAN, 1.0), None);
        assert_eq!(mit_current_range_from(1.0, f32::INFINITY), None);
    }

    #[test]
    fn json_desc_cache_assembles_metadata_and_chunks() {
        // Frames captured on hardware: metadata first, then 6-byte JSON chunks.
        let mut cache = JsonDescCache::default();
        cache.reset();
        let meta = [0x00, 0x00, 0x21, 0x96, 0x00, 0x00, 0x82, 0x3F];
        assert!(cache.record(&meta));
        assert_eq!(cache.total_len, Some(38433));
        assert_eq!(cache.version_crc, Some(0x3F82));

        assert!(cache.record(&[0x00, 0x00, b'{', b'"', b'a', b'b', b'c', b'd']));
        assert!(cache.record(&[0x06, 0x00, b'e', b'f', b'g', b'h', b'i', b'j']));
        assert_eq!(&cache.bytes, b"{\"abcdefghij");

        // The device repeats the metadata frame after a continuation request; it
        // must never be stored as descriptor bytes (that would overwrite the start).
        assert!(cache.record(&meta));
        assert_eq!(&cache.bytes, b"{\"abcdefghij");
        assert_eq!(cache.total_len, Some(38433));
    }
    use motor_core::bus::{CanBus, CanFrame};
    use motor_core::test_support::MockBus;
    use std::sync::Arc;

    fn make_motor() -> CyberBeastMotor {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        CyberBeastMotor::new(0x01, 0x01, "odrive-default", bus).unwrap()
    }

    #[test]
    fn test_model_catalog() {
        let spec = CYBERBEAST_CATALOG.get("odrive-default").unwrap();
        assert_eq!(spec.vendor, "cyberbeast");
        assert_eq!(spec.model, "odrive-default");
    }

    #[test]
    fn test_unknown_model_rejected() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let result = CyberBeastMotor::new(0x01, 0x01, "nonexistent", bus);
        assert!(result.is_err());
    }

    #[test]
    fn test_can_id_roundtrip() {
        let id = make_can_id(2, 0x00, 0x01, 0x0A, 1);
        let parts = can_id_parts(id);
        assert_eq!(parts.priority, 2);
        assert_eq!(parts.msg_type, 0x00); // MIT_CONTROL
        assert_eq!(parts.dest, 0x01);
        assert_eq!(parts.source, 0x0A);
        assert_eq!(parts.seq, 1);
        assert!(!parts.is_broadcast);
    }

    #[test]
    fn test_broadcast_detection() {
        let id = make_can_id(2, 0x80, 0xFF, 0x0A, 0);
        let parts = can_id_parts(id);
        assert!(parts.is_broadcast);
    }

    #[test]
    fn test_mit_pack_unpack_roundtrip() {
        let params = MitCommandParams {
            pos: 1.5,
            vel: 10.0,
            kp: 100.0,
            kd: 5.0,
            torque: 0.5,
        };
        let packed = pack_mit_command(&params, 12.5, 50.0, 500.0, 100.0, 10.0);
        let unpacked = protocol::unpack_mit_command(&packed, 12.5, 50.0, 500.0, 100.0, 10.0);

        // Due to quantization, compare with tolerance
        assert!((unpacked.pos - 1.5).abs() < 0.01);
        assert!((unpacked.vel - 10.0).abs() < 0.1);
        assert!((unpacked.kp - 100.0).abs() < 1.0);
        assert!((unpacked.kd - 5.0).abs() < 0.1);
        assert!((unpacked.torque - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_mit_response_decode() {
        // Verify decode of zero-filled MIT response doesn't panic and returns defaults
        let resp = unpack_mit_response(&[0u8; 8], 12.5, 50.0, 40.0);
        assert_eq!(resp.error_code, 0);
        assert_eq!(resp.mode_state, 0);
    }

    #[test]
    fn test_accepts_frame() {
        let motor = make_motor();
        // Frame from our motor (source == motor_id)
        let id = make_can_id(2, 0x00, 0x0A, 0x01, 0); // source=0x01 matches motor_id=1
        let frame = CanFrame {
            arbitration_id: id,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&frame));

        // Frame from a different motor
        let id2 = make_can_id(2, 0x00, 0x0A, 0x02, 0); // source=0x02
        let frame2 = CanFrame {
            arbitration_id: id2,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&frame2));

        // TX frame should not be accepted
        let frame3 = CanFrame {
            arbitration_id: id,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: false,
        };
        assert!(!motor.accepts_frame(&frame3));
    }

    #[test]
    fn test_send_mit_command_encodes_correct_frame() {
        let mock_bus: Arc<MockBus> = Arc::new(MockBus::new());
        let bus: Arc<dyn CanBus> = Arc::clone(&mock_bus) as Arc<dyn CanBus>;
        let motor = CyberBeastMotor::new(0x01, 0x01, "odrive-default", bus).unwrap();

        motor.send_mit_command(1.0, 0.5, 100.0, 10.0, 0.2).unwrap();

        let sent: Vec<CanFrame> = mock_bus.sent.lock().unwrap().drain(..).collect();
        assert_eq!(sent.len(), 1);
        let frame = &sent[0];
        assert!(frame.is_extended);
        assert!(!frame.is_rx);

        // Verify CAN ID: Priority=2(HighCtrl), MsgType=0x00(MIT), dest=0x01, source=0x01
        let parts = can_id_parts(frame.arbitration_id);
        assert_eq!(parts.priority, Priority::HighCtrl as u8);
        assert_eq!(parts.msg_type, MsgType::MitControl as u8);
        assert_eq!(parts.dest, 0x01);
        assert_eq!(parts.source, DEFAULT_MASTER_ID);
    }

    #[test]
    fn test_mit_response_process_updates_state() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x02, 0x02, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a mock MIT response frame
        // pos=1.0 rad, vel=2.0 rad/s, current=3.0 A, error=0, mode=4(MIT)
        let params = MitCommandParams {
            pos: 1.0,
            vel: 2.0,
            kp: 0.0,
            kd: 0.0,
            torque: 3.0, // Using torque value to represent current in packed form
        };
        let packed = pack_mit_command(&params, 12.5, 50.0, 500.0, 100.0, 40.0);

        let can_id = make_can_id(2, MsgType::MitControl as u8, 0x0A, 0x02, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data: packed,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        // With packed MIT command layout mapped to response decode, values won't match
        // exactly, but state should be populated
        assert!(state.pos != 0.0 || state.vel != 0.0 || state.current != 0.0);
        assert_eq!(state.can_id_parts.source, 0x02);
    }

    #[test]
    fn test_heartbeat_decode_updates_state() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x03, 0x03, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a mock heartbeat frame (new v8.1 format)
        // Life=2 (0b010xxxxx), ErrorFlags=0x03 (AXIS|MOTOR)
        // State=0x3 (CLOSED_LOOP), Mode=0x1 (POSITION)
        // Motor Temp=80 (raw) → actual 30°C
        // Position=5000 (raw int16) → 50.0 turns
        // Velocity=200 (raw int16) → 2.0 turns/s
        // Iq=20 (raw int8) → 10.0 A
        let mut data = [0u8; 8];
        data[0] = (2 << 5) | 0x03; // Life=2, ErrorFlags=AXIS|MOTOR
        data[1] = (0x3 << 4) | 0x1; // State=3, ControlMode=1
        data[2] = 80; // Motor Temp = 30°C
        let pos_raw: i16 = 5000;
        data[3] = (pos_raw >> 8) as u8;
        data[4] = pos_raw as u8;
        let vel_raw: i16 = 200;
        data[5] = (vel_raw >> 8) as u8;
        data[6] = vel_raw as u8;
        data[7] = 20; // Iq = 10.0 A

        let can_id = make_can_id(6, MsgType::Heartbeat as u8, 0x01, 0x03, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        assert_eq!(state.heartbeat_life, 2);
        assert_eq!(state.error_flags, 0x03); // AXIS|MOTOR
        assert!((state.motor_temp - 30.0).abs() < 1.0);
        // Position: 50.0 turns → output rad = 50.0 * 2π
        assert!((state.pos - 50.0 * 2.0 * std::f32::consts::PI).abs() < 1.0);
        // Velocity: 2.0 turns/s → output rad/s = 2.0 * 2π
        assert!((state.vel - 2.0 * 2.0 * std::f32::consts::PI).abs() < 1.0);
        assert!((state.current - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_process_mit_response_updates_core_fields() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x04, 0x04, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a proper MIT response (different layout from MIT command)
        // Response: pos(16-bit), vel(12-bit)+error(4-bit), current(12-bit)+mode(4-bit), motor_temp, mos_temp
        // Use mid-range values: pos=0x8000, vel=0x0800, error=0, current=0x0800, mode=3
        let buf: [u8; 8] = [
            0x80, 0x00, // pos mid-range
            0x80, 0x00, // vel mid-range + error=0
            0x80, 0x03, // current mid-range + mode=3 (CLOSED_LOOP)
            0x6E, // motor_temp=110 → actual = 60°C
            0x6E, // mos_temp=110 → actual = 60°C
        ];

        let can_id = make_can_id(2, MsgType::MitControl as u8, 0x01, 0x04, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data: buf,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        assert_eq!(state.mode_state, 3); // CLOSED_LOOP
        assert_eq!(state.error_code, 0);
        assert!((state.motor_temp - 60.0).abs() < 1.0);
        assert!((state.mos_temp - 60.0).abs() < 1.0);
    }
}
