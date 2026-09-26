use crate::args::{get_f32, get_opt_u16_hex_or_dec, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_can_bus, CanBus, CanFrame};
use motor_core::error::Result as MotorResult;
use motor_vendor_cyberbeast::{
    big_endian_bytes_to_f32, can_id_parts, decode_heartbeat, CyberBeastController, CyberBeastMotor,
    CyberBeastMotorState, ModeState, MsgType, REGISTER_TABLE,
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

const QUERY_TIMEOUT_MS: u64 = 200;
const PARAM_TIMEOUT_MS: u64 = 200;

// ---------------------------------------------------------------------------
// Ctrl+C / SIGTERM handling for long-running loops
// ---------------------------------------------------------------------------

#[cfg(unix)]
mod sigint {
    use std::sync::atomic::{AtomicBool, Ordering};

    static STOP_REQUESTED: AtomicBool = AtomicBool::new(false);

    extern "C" fn handle_stop_signal(_sig: i32) {
        STOP_REQUESTED.store(true, Ordering::SeqCst);
    }

    unsafe extern "C" {
        fn signal(sig: i32, handler: usize) -> usize;
    }

    const SIGINT: i32 = 2;
    const SIGTERM: i32 = 15;

    /// Install handlers so an interrupted loop can send StopMotor before exiting.
    pub fn install() {
        // `function_casts_as_integer` requires going through a pointer first.
        let handler = handle_stop_signal as *const () as usize;
        unsafe {
            signal(SIGINT, handler);
            signal(SIGTERM, handler);
        }
    }

    pub fn stop_requested() -> bool {
        STOP_REQUESTED.load(Ordering::SeqCst)
    }
}

#[cfg(not(unix))]
mod sigint {
    /// No handler on this platform: Ctrl+C terminates the process directly.
    pub fn install() {}

    pub fn stop_requested() -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Optional frame trace (--trace)
// ---------------------------------------------------------------------------

struct TracingBus {
    inner: Arc<dyn CanBus>,
}

impl CanBus for TracingBus {
    fn send(&self, frame: CanFrame) -> MotorResult<()> {
        eprintln!(
            "[cb TX] id=0x{:08X} {}",
            frame.arbitration_id,
            frame_text(&frame)
        );
        self.inner.send(frame)
    }

    fn recv(&self, timeout: Duration) -> MotorResult<Option<CanFrame>> {
        let frame = self.inner.recv(timeout)?;
        if let Some(f) = frame.as_ref() {
            eprintln!(
                "[cb RX] id=0x{:08X} {}{}",
                f.arbitration_id,
                frame_text(f),
                payload_suffix(f)
            );
        }
        Ok(frame)
    }

    fn shutdown(&self) -> MotorResult<()> {
        self.inner.shutdown()
    }
}

fn msg_type_name(msg_type: u8) -> &'static str {
    match msg_type {
        0x00 => "mit",
        0x01 => "pos",
        0x02 => "vel",
        0x03 => "torque",
        0x04 => "current",
        0x20 => "param-read",
        0x21 => "param-write",
        0x22 => "config-save",
        0x23 => "config-reset",
        0x24 => "json-desc-read",
        0x25 => "json-desc-data",
        0x40 => "query-status",
        0x41 => "query-posvel",
        0x42 => "query-current",
        0x43 => "query-temp",
        0x44 => "query-bus",
        0x45 => "query-error",
        0x46 => "query-devinfo",
        0x47 => "query-power",
        0x48 => "heartbeat",
        0x49 => "status-feedback",
        0x60 => "set-node-id",
        0x61 => "set-zero",
        0x62 => "start-motor",
        0x63 => "stop-motor",
        0x64 => "reset-device",
        0x65 => "clear-errors",
        0x80 => "mit-broadcast",
        0x81 => "pos-broadcast",
        0x82 => "vel-broadcast",
        0x83 => "torque-broadcast",
        0xC0 => "estop",
        0xC1 => "fault-alert",
        _ => "unknown",
    }
}

fn frame_text(frame: &CanFrame) -> String {
    let parts = can_id_parts(frame.arbitration_id);
    let data: Vec<String> = frame
        .data
        .iter()
        .take(frame.dlc as usize)
        .map(|b| format!("{b:02X}"))
        .collect();
    format!(
        "pri={} mt=0x{:02X}({}) dest={} src={} seq={} dlc={} data=[{}]",
        parts.priority,
        parts.msg_type,
        msg_type_name(parts.msg_type),
        parts.dest,
        parts.source,
        parts.seq,
        frame.dlc,
        data.join(" ")
    )
}

/// Extra decoding appended to received-frame trace lines.
fn payload_suffix(frame: &CanFrame) -> String {
    let msg_type = can_id_parts(frame.arbitration_id).msg_type;
    if msg_type == MsgType::Heartbeat as u8 {
        return match decode_heartbeat(&frame.data) {
            Some(hb) => format!(
                "  => life={} errflags=0x{:02X} state={} mode={} temp={:.1}C pos={:.3}turns vel={:.3}turns/s iq={:.1}A",
                hb.life_counter,
                hb.error_flags,
                hb.motor_state,
                hb.control_mode,
                hb.motor_temp,
                hb.position_turns,
                hb.velocity_turns_per_s,
                hb.iq_current
            ),
            None => String::new(),
        };
    }
    // 0x41/0x42/0x43/0x44 all answer with two big-endian float32 values.
    let pair_label = match msg_type {
        0x41 => "pos,vel",
        0x42 => "iq,id",
        0x43 => "motor_temp,fet_temp",
        0x44 => "vbus,ibus",
        _ => return String::new(),
    };
    format!(
        "  => {pair_label} = {:.6}, {:.6} (float32 BE)",
        big_endian_bytes_to_f32(&frame.data, 0),
        big_endian_bytes_to_f32(&frame.data, 4)
    )
}

/// The mode/error nibbles only exist in MIT response frames. A heartbeat replaces the
/// cached state without touching them, so never report the leftover value as current.
fn mode_text(state: &CyberBeastMotorState) -> String {
    if state.can_id_parts.msg_type == MsgType::Heartbeat as u8 {
        "n/a(hb)".to_string()
    } else {
        format!(
            "{}({})",
            state.mode_state,
            ModeState::name(state.mode_state)
        )
    }
}

fn error_text(state: &CyberBeastMotorState) -> String {
    if state.can_id_parts.msg_type == MsgType::Heartbeat as u8 {
        "n/a(hb)".to_string()
    } else {
        format!("0x{:X}", state.error_code)
    }
}

fn progress_line(state: &CyberBeastMotorState) -> String {
    format!(
        "pos={:.4} vel={:.4} cur={:.3}A err={} mode={} temp={:.1}C life={} errflags=0x{:02X}",
        state.pos,
        state.vel,
        state.current,
        error_text(state),
        mode_text(state),
        state.motor_temp,
        state.heartbeat_life,
        state.error_flags
    )
}

/// Newest view of each frame type collected during a read window.
#[derive(Default)]
struct Snapshot {
    heartbeat: Option<String>,
    mit: Option<(u8, u8, f32, f32, f32)>,
    pos_vel: Option<(f32, f32)>,
    device_info: Option<(u32, u32)>,
}

/// Poll the bus for a fixed window, latching the newest view of each frame type.
fn pump(
    ctrl: &CyberBeastController,
    motor: &Arc<CyberBeastMotor>,
    window_ms: u64,
    snapshot: &mut Snapshot,
) {
    let deadline = Instant::now() + Duration::from_millis(window_ms);
    while Instant::now() < deadline {
        let _ = ctrl.poll_feedback_once();
        if let Some(state) = motor.latest_state() {
            let msg_type = state.can_id_parts.msg_type;
            if msg_type == MsgType::Heartbeat as u8 {
                snapshot.heartbeat = Some(progress_line(&state));
            } else if msg_type == MsgType::MitControl as u8 {
                snapshot.mit = Some((
                    state.mode_state,
                    state.error_code,
                    state.current,
                    state.motor_temp,
                    state.mos_temp,
                ));
            } else if msg_type == MsgType::QueryPosVel as u8 {
                snapshot.pos_vel = Some((state.pos, state.vel));
            } else if msg_type == MsgType::QueryDeviceInfo as u8 {
                if let (Some(hw), Some(fw)) = (state.hw_version, state.fw_version) {
                    snapshot.device_info = Some((hw, fw));
                }
            }
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Report (and apply) the device's MIT response current range so telemetry is scaled correctly.
fn apply_mit_current_range(motor: &CyberBeastMotor, timeout_ms: u64) {
    match motor.probe_mit_current_range(Duration::from_millis(timeout_ms)) {
        Ok(range) => println!("  MIT response current range derived from the device: +/-{range} A"),
        Err(err) => eprintln!("  warning: {err}"),
    }
}

/// `--tau` is accepted as an alias of `--torque` for consistency with other vendors.
fn get_torque(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("tau") {
        get_f32(args, "tau", 0.0)
    } else {
        get_f32(args, "torque", 0.0)
    }
}

/// `--endpoint` is the canonical spelling; `--param-id` (Python CLI spelling) is accepted too.
fn get_endpoint(args: &HashMap<String, String>) -> Result<u16, String> {
    if let Some(value) = get_opt_u16_hex_or_dec(args, "endpoint")? {
        return Ok(value);
    }
    if let Some(value) = get_opt_u16_hex_or_dec(args, "param-id")? {
        return Ok(value);
    }
    Err(
        "--endpoint <hex|dec> is required for read-param/write-param (alias: --param-id)"
            .to_string(),
    )
}

/// Human-readable name for a documented SDO endpoint, if the static table knows it.
fn endpoint_name(endpoint: u16) -> Option<&'static str> {
    REGISTER_TABLE
        .iter()
        .find(|info| info.endpoint_id == endpoint)
        .map(|info| info.variable)
}

/// Declared value type of a documented SDO endpoint, as reported by the device descriptor.
fn endpoint_type(endpoint: u16) -> &'static str {
    REGISTER_TABLE
        .iter()
        .find(|info| info.endpoint_id == endpoint)
        .map(|info| info.value_type)
        .unwrap_or("unknown")
}

fn le_array<const N: usize>(bytes: &[u8]) -> Option<[u8; N]> {
    bytes.get(..N).and_then(|slice| slice.try_into().ok())
}

/// Decode little-endian value bytes using the type declared by the device descriptor.
///
/// Firmware 0.6.9 answers PARAM_READ with **little-endian** values (verified on
/// hardware: vbus 23.09 V, torque_constant 0.0824, cpr 16384), even though
/// protocol 4.7 documents big-endian.
fn decode_param_value(declared: &str, bytes: &[u8]) -> String {
    let undecodable = || unknown_value(bytes);
    match declared.split_whitespace().next().unwrap_or("") {
        "float" => {
            le_array::<4>(bytes).map_or_else(undecodable, |b| f32::from_le_bytes(b).to_string())
        }
        "uint8" => bytes.first().map_or_else(undecodable, |b| b.to_string()),
        "int8" => bytes
            .first()
            .map_or_else(undecodable, |b| (*b as i8).to_string()),
        "bool" => bytes
            .first()
            .map_or_else(undecodable, |b| (*b != 0).to_string()),
        "uint16" => {
            le_array::<2>(bytes).map_or_else(undecodable, |b| u16::from_le_bytes(b).to_string())
        }
        "int16" => {
            le_array::<2>(bytes).map_or_else(undecodable, |b| i16::from_le_bytes(b).to_string())
        }
        "uint32" => {
            le_array::<4>(bytes).map_or_else(undecodable, |b| u32::from_le_bytes(b).to_string())
        }
        "int32" => {
            le_array::<4>(bytes).map_or_else(undecodable, |b| i32::from_le_bytes(b).to_string())
        }
        "uint64" => {
            le_array::<8>(bytes).map_or_else(undecodable, |b| u64::from_le_bytes(b).to_string())
        }
        _ => {
            let as_f32 = le_array::<4>(bytes).map(f32::from_le_bytes);
            let as_i32 = le_array::<4>(bytes).map(i32::from_le_bytes);
            match (as_f32, as_i32) {
                (Some(f), Some(i)) => format!(
                    "{f} (if float32 LE) / {i} (if int32 LE); type unknown for this endpoint"
                ),
                _ => unknown_value(bytes),
            }
        }
    }
}

fn unknown_value(bytes: &[u8]) -> String {
    format!(
        "(no decoder for {} byte(s): {})",
        bytes.len(),
        bytes
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    )
}

/// Walk the descriptor tree and collect `(name, id, type, access)` for nested objects too.
fn collect_endpoint_entries(
    value: &serde_json::Value,
    prefix: &str,
    out: &mut Vec<(String, u64, String, String)>,
) {
    match value {
        serde_json::Value::Object(map) => {
            let name = map.get("name").and_then(|v| v.as_str()).unwrap_or("");
            let path = if prefix.is_empty() || name.is_empty() {
                format!("{prefix}{name}")
            } else {
                format!("{prefix}.{name}")
            };
            if let Some(id) = map.get("id").and_then(|v| v.as_u64()) {
                let value_type = map
                    .get("type")
                    .and_then(|v| v.as_str())
                    .unwrap_or("unknown");
                let access = map.get("access").and_then(|v| v.as_str()).unwrap_or("-");
                out.push((path.clone(), id, value_type.to_string(), access.to_string()));
            }
            for (key, child) in map {
                if matches!(key.as_str(), "name" | "id" | "type" | "access") {
                    continue;
                }
                if child.is_object() || child.is_array() {
                    collect_endpoint_entries(child, &path, out);
                }
            }
        }
        serde_json::Value::Array(items) => {
            for item in items {
                collect_endpoint_entries(item, prefix, out);
            }
        }
        _ => {}
    }
}

pub fn run_cyberbeast(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    _feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let ctrl = if args.contains_key("trace") {
        CyberBeastController::new(Arc::new(TracingBus {
            inner: open_can_bus(channel)?,
        }))
    } else {
        CyberBeastController::new_socketcan(channel)?
    };

    match mode.as_str() {
        "scan" => {
            let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
            let end_id = get_u16_hex_or_dec(args, "end-id", 32)?;
            if end_id < start_id {
                return Err(format!(
                    "--end-id 0x{end_id:02X} is below --start-id 0x{start_id:02X}"
                )
                .into());
            }
            println!(
                "scanning CyberBeast motors on {channel} (IDs {start_id}..{end_id}); query-only: no StartMotor/StopMotor frame is sent"
            );

            let mut responders = 0u32;
            for id in start_id..=end_id {
                let motor = match ctrl.add_motor(id, id, model) {
                    Ok(m) => m,
                    Err(_) => continue,
                };
                let _ = motor.send_query_status();
                std::thread::sleep(Duration::from_millis(QUERY_TIMEOUT_MS));
                let _ = ctrl.poll_feedback_once();

                if let Some(state) = motor.latest_state() {
                    responders += 1;
                    println!(
                        "  [found] id=0x{id:02X} pos={:.4} vel={:.4} [source mt=0x{:02X} ({})]",
                        state.pos,
                        state.vel,
                        state.can_id_parts.msg_type,
                        msg_type_name(state.can_id_parts.msg_type)
                    );
                    println!(
                        "          current={:.3} A err={} mode={} motor_temp={:.1}C mos_temp={:.1}C life={} errflags=0x{:02X}",
                        state.current,
                        error_text(&state),
                        mode_text(&state),
                        state.motor_temp,
                        state.mos_temp,
                        state.heartbeat_life,
                        state.error_flags
                    );
                }
            }
            println!("scan done: {responders} device(s) responded");
            ctrl.shutdown()?;
        }

        "status" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!(
                "status for motor 0x{motor_id:02X} on {channel} (query-only: no StartMotor/StopMotor frame is sent)"
            );
            let mut snapshot = Snapshot::default();
            let _ = motor.send_query_status();
            pump(&ctrl, &motor, QUERY_TIMEOUT_MS + 100, &mut snapshot);
            let _ = motor.send_query_pos_vel();
            pump(&ctrl, &motor, QUERY_TIMEOUT_MS + 100, &mut snapshot);
            let _ = motor.send_query_device_info();
            pump(&ctrl, &motor, QUERY_TIMEOUT_MS + 100, &mut snapshot);

            println!(
                "  heartbeat   : {}",
                snapshot
                    .heartbeat
                    .as_deref()
                    .unwrap_or("no heartbeat received (is the device powered?)")
            );
            match snapshot.mit {
                Some((mode, err, current, motor_temp, mos_temp)) => println!(
                    "  query-status: mode={mode}({}) err=0x{err:X} current={current:.3}A motor_temp={motor_temp:.1}C mos_temp={mos_temp:.1}C",
                    ModeState::name(mode)
                ),
                None => println!("  query-status: no response to QueryStatus (0x40)"),
            }
            match snapshot.pos_vel {
                Some((pos, vel)) => println!(
                    "  query-posvel: pos={pos:.6} rad vel={vel:.6} rad/s (motor-side turns reported by the device, converted to rad)"
                ),
                None => println!("  query-posvel: no response to QueryPosVel (0x41)"),
            }
            match snapshot.device_info {
                Some((hw, fw)) => println!(
                    "  device-info : hw=0x{hw:08X} ({}.{}.{}) fw=0x{fw:08X} ({}.{}.{})",
                    (hw >> 16) & 0xFF,
                    (hw >> 8) & 0xFF,
                    hw & 0xFF,
                    (fw >> 16) & 0xFF,
                    (fw >> 8) & 0xFF,
                    fw & 0xFF
                ),
                None => println!("  device-info : no response to QueryDeviceInfo (0x46)"),
            }
            match motor.probe_mit_current_range(Duration::from_millis(PARAM_TIMEOUT_MS)) {
                Ok(range) => println!(
                    "  mit-current : +/-{range} A (MIT response current range derived from the device)"
                ),
                Err(err) => println!("  mit-current : {err}"),
            }
            ctrl.shutdown()?;
        }

        "mit" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            apply_mit_current_range(&motor, PARAM_TIMEOUT_MS);
            let kp = get_f32(args, "kp", 100.0)?;
            let kd = get_f32(args, "kd", 10.0)?;
            let target_pos = get_f32(args, "pos", 0.0)?;
            let target_vel = get_f32(args, "vel", 0.0)?;
            let target_torque = get_torque(args)?;
            let loop_ms = get_u64(args, "loop-ms", 5)?;
            sigint::install();
            println!(
                "starting MIT control for motor 0x{motor_id:02X}: pos={target_pos} vel={target_vel} kp={kp} kd={kd} tau={target_torque} loop={loop_ms}ms"
            );
            println!(
                "  Ctrl+C sends StopMotor (0x63) then exits; pos/vel units follow the last received frame"
            );
            let _ = motor.send_start_motor();
            let mut cycles = 0u64;

            while !sigint::stop_requested() {
                motor.send_mit_command(target_pos, target_vel, kp, kd, target_torque)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!("\r{}", progress_line(&state));
                }
                cycles += 1;
                std::thread::sleep(Duration::from_millis(loop_ms));
            }

            println!("\nstop requested after {cycles} cycles");
            match motor.send_stop_motor() {
                Ok(()) => println!("  StopMotor (0x63) sent; the axis returns to IDLE"),
                Err(err) => eprintln!("  StopMotor failed: {err}"),
            }
            ctrl.shutdown()?;
        }

        "pos" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            apply_mit_current_range(&motor, PARAM_TIMEOUT_MS);
            let target_pos = get_f32(args, "pos", 0.0)?;
            let vel_limit = get_f32(args, "vel-limit", 100.0)?;
            // Current limit [A]. Default exceeds hardware max so the firmware torque_lim clamp is inert.
            let cur_limit = get_f32(args, "cur-limit", 200.0)?;
            let loop_ms = get_u64(args, "loop-ms", 10)?;
            sigint::install();
            println!(
                "starting POS control for motor 0x{motor_id:02X}: pos={target_pos} vel_limit={vel_limit} cur_limit={cur_limit} loop={loop_ms}ms"
            );
            println!("  Ctrl+C sends StopMotor (0x63) then exits");
            let _ = motor.send_start_motor();
            let mut cycles = 0u64;

            while !sigint::stop_requested() {
                motor.send_pos_control(target_pos, vel_limit, cur_limit)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!("\r{}", progress_line(&state));
                }
                cycles += 1;
                std::thread::sleep(Duration::from_millis(loop_ms));
            }

            println!("\nstop requested after {cycles} cycles");
            match motor.send_stop_motor() {
                Ok(()) => println!("  StopMotor (0x63) sent; the axis returns to IDLE"),
                Err(err) => eprintln!("  StopMotor failed: {err}"),
            }
            ctrl.shutdown()?;
        }

        "vel" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            apply_mit_current_range(&motor, PARAM_TIMEOUT_MS);
            let target_vel = get_f32(args, "vel", 0.0)?;
            // Current limit [A]. Default exceeds hardware max so the firmware torque_lim clamp is inert.
            let cur_limit = get_f32(args, "cur-limit", 200.0)?;
            let loop_ms = get_u64(args, "loop-ms", 10)?;
            sigint::install();
            println!(
                "starting VEL control for motor 0x{motor_id:02X}: vel={target_vel} rpm cur_limit={cur_limit} loop={loop_ms}ms"
            );
            println!("  Ctrl+C sends StopMotor (0x63) then exits");
            let _ = motor.send_start_motor();
            let mut cycles = 0u64;

            while !sigint::stop_requested() {
                motor.send_vel_control(target_vel, cur_limit)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!("\r{}", progress_line(&state));
                }
                cycles += 1;
                std::thread::sleep(Duration::from_millis(loop_ms));
            }

            println!("\nstop requested after {cycles} cycles");
            match motor.send_stop_motor() {
                Ok(()) => println!("  StopMotor (0x63) sent; the axis returns to IDLE"),
                Err(err) => eprintln!("  StopMotor failed: {err}"),
            }
            ctrl.shutdown()?;
        }

        "torque" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            apply_mit_current_range(&motor, PARAM_TIMEOUT_MS);
            let target_torque = get_torque(args)?;
            let loop_ms = get_u64(args, "loop-ms", 5)?;
            sigint::install();
            println!(
                "starting TORQUE control for motor 0x{motor_id:02X}: tau={target_torque} loop={loop_ms}ms"
            );
            println!("  Ctrl+C sends StopMotor (0x63) then exits");
            let _ = motor.send_start_motor();
            let mut cycles = 0u64;

            while !sigint::stop_requested() {
                motor.send_torque_control(target_torque)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!("\r{}", progress_line(&state));
                }
                cycles += 1;
                std::thread::sleep(Duration::from_millis(loop_ms));
            }

            println!("\nstop requested after {cycles} cycles");
            match motor.send_stop_motor() {
                Ok(()) => println!("  StopMotor (0x63) sent; the axis returns to IDLE"),
                Err(err) => eprintln!("  StopMotor failed: {err}"),
            }
            ctrl.shutdown()?;
        }

        "enable" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_start_motor()?;
            println!(
                "enabled motor 0x{motor_id:02X} (StartMotor 0x62 sent; the device stays enabled after this CLI exits)"
            );
            ctrl.shutdown()?;
        }

        "disable" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_stop_motor()?;
            println!("disabled motor 0x{motor_id:02X} (StopMotor 0x63 sent)");
            ctrl.shutdown()?;
        }

        "estop" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!(
                "warning: ESTOP is a global broadcast (Priority=0, MsgType=0xC0, Dest=0xFF per protocol 4.9): every device on {channel} stops and latches an ESTOP error, cleared only by clear-error or a reset"
            );
            motor.send_estop()?;
            println!("estop broadcast sent (Priority=0, mt=0xC0, dest=0xFF)");
            ctrl.shutdown()?;
        }

        "clear-error" | "clear-fault" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_clear_errors()?;
            println!("clear-errors (0x65) sent to 0x{motor_id:02X}; re-read status to verify");
            ctrl.shutdown()?;
        }

        "set-zero" => {
            if !args.contains_key("yes") {
                return Err(
                    "set-zero changes the mechanical zero reference; re-run with --yes to confirm"
                        .into(),
                );
            }
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_set_zero()?;
            println!(
                "set-zero (0x61) sent to 0x{motor_id:02X}; re-read status to confirm the new zero"
            );
            ctrl.shutdown()?;
        }

        // SDO endpoint read/write (fibre endpoint system, protocol 4.7 / 4.8).
        "read-param" => {
            let endpoint = get_endpoint(args)?;
            let timeout_ms = get_u64(args, "timeout-ms", PARAM_TIMEOUT_MS)?;
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            // read_param_raw sends the request itself and follows the More flag, so
            // it works for every value width, not just 4-byte float32.
            let bytes = motor.read_param_raw(endpoint, Duration::from_millis(timeout_ms))?;
            let name = endpoint_name(endpoint).unwrap_or("not in the verified endpoint table");
            let declared = endpoint_type(endpoint);
            println!(
                "endpoint 0x{endpoint:04X} ({name}) declared={declared} value={}",
                decode_param_value(declared, &bytes)
            );
            println!(
                "  raw little-endian bytes: [{}]",
                bytes
                    .iter()
                    .map(|b| format!("{b:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
            ctrl.shutdown()?;
        }

        "write-param" => {
            let endpoint = get_endpoint(args)?;
            if !args.contains_key("value") {
                return Err("--value <float> is required for write-param".into());
            }
            if !args.contains_key("yes") {
                return Err(
                    "write-param changes device configuration; re-run with --yes to confirm".into(),
                );
            }
            let requested = get_f32(args, "value", 0.0)?;
            let timeout_ms = get_u64(args, "timeout-ms", PARAM_TIMEOUT_MS)?;
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_param_read(endpoint)?;
            let before = motor
                .get_param_f32(endpoint, Duration::from_millis(timeout_ms))
                .ok();
            motor.set_param_f32(endpoint, requested)?;
            motor.send_param_read(endpoint)?;
            let read_back = motor
                .get_param_f32(endpoint, Duration::from_millis(timeout_ms))
                .ok();
            let name = endpoint_name(endpoint).unwrap_or("?");
            println!(
                "endpoint 0x{endpoint:04X} ({name}): requested={requested} before={before:?} read_back={read_back:?}"
            );
            match read_back {
                Some(value) => {
                    let delta = (value - requested).abs();
                    if delta <= f32::EPSILON * requested.abs().max(1.0) {
                        println!("  verified: the device reports the requested value");
                    } else {
                        return Err(format!(
                            "read-back mismatch on endpoint 0x{endpoint:04X}: requested {requested}, device reports {value} (delta {delta})"
                        )
                        .into());
                    }
                }
                None => {
                    return Err(format!(
                        "endpoint 0x{endpoint:04X} write was acknowledged but could not be read back; treat it as unverified"
                    )
                    .into());
                }
            }
            ctrl.shutdown()?;
        }

        // Read-only: look an endpoint up by name in the device's own descriptor.
        "find-endpoint" => {
            let Some(name_filter) = args.get("name") else {
                return Err("--name <substring> is required for find-endpoint".into());
            };
            let filter = name_filter.to_lowercase();
            let timeout_ms = get_u64(args, "timeout-ms", 500)?;
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            let (total_len, version_crc, json) =
                motor.read_endpoint_descriptor(Duration::from_millis(timeout_ms))?;
            let root: serde_json::Value = serde_json::from_str(&json)
                .map_err(|err| format!("descriptor is not valid JSON: {err}"))?;
            let mut entries = Vec::new();
            collect_endpoint_entries(&root, "", &mut entries);
            let mut shown = 0usize;
            for (name, id, value_type, access) in &entries {
                if name.to_lowercase().contains(&filter) {
                    shown += 1;
                    println!("  id=0x{id:04X} ({id:5}) {value_type:<9} {access:<4} {name}");
                }
            }
            println!(
                "  {shown} of {} endpoints match \"{name_filter}\" (descriptor {total_len} bytes, VersionCRC=0x{version_crc:04X})",
                entries.len()
            );
            ctrl.shutdown()?;
        }

        // Read-only: fetch the device's own endpoint descriptor (protocol 4.8).
        "endpoint-map" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            let timeout_ms = get_u64(args, "timeout-ms", 500)?;
            println!(
                "fetching the JSON endpoint descriptor from 0x{motor_id:02X} (JSON_DESC_READ 0x24 / JSON_DESC_DATA 0x25), stall window {timeout_ms} ms"
            );
            let (total_len, version_crc, json) =
                motor.read_endpoint_descriptor(Duration::from_millis(timeout_ms))?;
            println!(
                "  descriptor: {total_len} bytes, VersionCRC=0x{version_crc:04X}, {} \"id\" occurrences",
                json.matches("\"id\"").count()
            );
            match args.get("out") {
                Some(path) => {
                    std::fs::write(path, json.as_bytes())
                        .map_err(|err| format!("cannot write {path}: {err}"))?;
                    println!("  wrote {path}");
                }
                None => {
                    println!("  (use --out <path> to save the JSON, or --dump to print it here)")
                }
            }
            if args.contains_key("dump") {
                print!("{json}");
            }
            ctrl.shutdown()?;
        }

        // Passive link/state monitor: never transmits a frame.
        "monitor" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            let duration_s = get_u64(args, "duration-s", 0)?;
            sigint::install();
            println!(
                "monitor: passive (no frame is transmitted), duration {}",
                if duration_s == 0 {
                    "until Ctrl+C".to_string()
                } else {
                    format!("{duration_s}s")
                }
            );
            let started = Instant::now();
            let mut heartbeats = 0u64;
            let mut lost = 0u64;
            let mut prev_life: Option<u8> = None;
            let mut prev_print = Instant::now();

            while !sigint::stop_requested() {
                if duration_s > 0 && started.elapsed() >= Duration::from_secs(duration_s) {
                    break;
                }
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    if state.can_id_parts.msg_type == MsgType::Heartbeat as u8 {
                        match prev_life {
                            Some(prev) => {
                                let delta = state.heartbeat_life.wrapping_sub(prev) & 0x07;
                                if delta != 0 {
                                    heartbeats += u64::from(delta);
                                    lost += u64::from(delta - 1);
                                    prev_life = Some(state.heartbeat_life);
                                }
                            }
                            None => {
                                heartbeats += 1;
                                prev_life = Some(state.heartbeat_life);
                            }
                        }
                    }
                    if prev_print.elapsed() >= Duration::from_millis(500) {
                        println!(
                            "  t={:>6.1}s heartbeats={heartbeats} lost={lost} | {}",
                            started.elapsed().as_secs_f32(),
                            progress_line(&state)
                        );
                        prev_print = Instant::now();
                    }
                }
                std::thread::sleep(Duration::from_millis(2));
            }

            let elapsed = started.elapsed().as_secs_f32();
            let total = heartbeats + lost;
            println!(
                "monitor done: {elapsed:.1}s heartbeats={heartbeats} lost={lost} ({:.2}% loss), rate={:.2} Hz",
                if total > 0 {
                    100.0 * lost as f32 / total as f32
                } else {
                    0.0
                },
                heartbeats as f32 / elapsed.max(0.001)
            );
            ctrl.shutdown()?;
        }

        // Query keep-alive: sends QueryStatus / QueryPosVel only, never enable or control frames.
        "keep-alive" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            let period_ms = get_u64(args, "keep-alive-ms", 500)?;
            let duration_s = get_u64(args, "duration-s", 0)?;
            if period_ms == 0 {
                return Err("--keep-alive-ms must be greater than 0".into());
            }
            sigint::install();
            println!(
                "keep-alive: QueryStatus + QueryPosVel every {period_ms} ms, duration {}; no StartMotor or control frame is sent",
                if duration_s == 0 {
                    "until Ctrl+C".to_string()
                } else {
                    format!("{duration_s}s")
                }
            );
            let started = Instant::now();
            let mut sent = 0u64;
            let mut next_print = Instant::now();

            while !sigint::stop_requested() {
                if duration_s > 0 && started.elapsed() >= Duration::from_secs(duration_s) {
                    break;
                }
                motor.send_query_status()?;
                motor.send_query_pos_vel()?;
                sent += 2;
                let deadline = Instant::now() + Duration::from_millis(period_ms);
                while Instant::now() < deadline {
                    let _ = ctrl.poll_feedback_once();
                    std::thread::sleep(Duration::from_millis(2));
                }
                if next_print.elapsed() >= Duration::from_secs(1) {
                    match motor.latest_state() {
                        Some(state) => println!(
                            "  t={:>6.1}s sent={sent} | {}",
                            started.elapsed().as_secs_f32(),
                            progress_line(&state)
                        ),
                        None => println!(
                            "  t={:>6.1}s sent={sent} | no response yet",
                            started.elapsed().as_secs_f32()
                        ),
                    }
                    next_print = Instant::now();
                }
            }
            println!("keep-alive done: sent={sent} frame(s)");
            ctrl.shutdown()?;
        }

        other => {
            eprintln!(
                "unknown mode: {other}. Supported: scan, status, mit, pos, vel, torque, enable, disable, estop, clear-error, set-zero, read-param, write-param, endpoint-map, find-endpoint, monitor, keep-alive"
            );
            ctrl.shutdown()?;
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collect_endpoint_entries_walks_nested_objects() {
        // Shape mirrors the device descriptor: nested objects carry their own id,
        // so `find-endpoint` must report the dotted path, not just the leaf name.
        let json = r#"[
            {"name":"axis0","id":14,"type":"object","children":[
                {"name":"config","id":21,"type":"object","children":[
                    {"name":"node_id","id":180,"type":"uint32","access":"rw"}]},
                {"name":"current_state","id":142,"type":"uint8","access":"r"}]}]"#;
        let root: serde_json::Value = serde_json::from_str(json).expect("fixture is valid json");
        let mut entries = Vec::new();

        collect_endpoint_entries(&root, "", &mut entries);

        assert_eq!(
            entries,
            vec![
                (
                    "axis0".to_string(),
                    14,
                    "object".to_string(),
                    "-".to_string()
                ),
                (
                    "axis0.config".to_string(),
                    21,
                    "object".to_string(),
                    "-".to_string()
                ),
                (
                    "axis0.config.node_id".to_string(),
                    180,
                    "uint32".to_string(),
                    "rw".to_string()
                ),
                (
                    "axis0.current_state".to_string(),
                    142,
                    "uint8".to_string(),
                    "r".to_string()
                ),
            ]
        );
    }

    #[test]
    fn collect_endpoint_entries_ignores_objects_without_id() {
        let root: serde_json::Value =
            serde_json::from_str(r#"{"name":"meta","children":[{"name":"x"}]}"#).expect("json");
        let mut entries = Vec::new();

        collect_endpoint_entries(&root, "", &mut entries);

        assert!(entries.is_empty());
    }
}
