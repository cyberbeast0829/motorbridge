use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_can_bus, CanBus, CanFrame};
use motor_core::error::Result as MotorResult;
use motor_vendor_cyberbeast::{
    big_endian_bytes_to_f32, can_id_parts, decode_heartbeat, CyberBeastController, CyberBeastMotor,
    CyberBeastMotorState, ModeState, MsgType, ParamReadout, ParamValue, ValueType,
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

/// Report (and apply) the device's own MIT scaling so commands and telemetry mean what they say.
///
/// Two independent things are needed: the ranges that scale the MIT bit fields (without
/// them a commanded torque or Kd is scaled by the protocol defaults instead of the
/// device's own maxima) and the current range used to decode the MIT response.
fn apply_mit_scaling(motor: &CyberBeastMotor, timeout_ms: u64) {
    let timeout = Duration::from_millis(timeout_ms);
    match motor.probe_mit_ranges(timeout) {
        Ok(ranges) => println!(
            "  MIT encode ranges from the device: pos=+/-{} rad, vel=+/-{} rad/s, kp<={}, kd<={}, tau=+/-{} Nm",
            ranges.pos, ranges.vel, ranges.kp, ranges.kd, ranges.torque
        ),
        Err(err) => eprintln!(
            "  warning: {err}\n           keeping the protocol defaults (pos 12.566, vel 30, kp 500, kd 100, tau 18), \
             so commanded values will not mean what they say"
        ),
    }
    match motor.probe_mit_current_range(timeout) {
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
///
/// The value is returned as written: it may be an id (`0x00F2`) or a name/path that
/// the device's own endpoint table resolves (`gear_ratio`).
fn get_endpoint_spec(args: &HashMap<String, String>) -> Result<String, String> {
    for key in ["endpoint", "param-id"] {
        if let Some(value) = args.get(key) {
            return Ok(value.clone());
        }
    }
    Err(
        "--endpoint <id|name> is required for read-param/write-param (alias: --param-id)"
            .to_string(),
    )
}

/// Resolve `--endpoint` into an id: a number, or a name/path from the device's table.
fn resolve_endpoint(
    motor: &CyberBeastMotor,
    spec: &str,
) -> Result<u16, Box<dyn std::error::Error>> {
    match motor.endpoint_map() {
        Some(map) => Ok(map.resolve(spec)?.endpoint_id),
        None => crate::args::parse_u16_hex_or_dec(spec, "--endpoint").map_err(|_| {
            format!(
                "\"{spec}\" is not a numeric endpoint id and the device's endpoint table is not \
                 loaded (--no-endpoint-map?); use an id such as 0x00F2"
            )
            .into()
        }),
    }
}

/// Adds motors through the controller.
///
/// Connecting loads the node's endpoint map (protocol 4.8) so every later
/// `read-param` / `write-param` uses the device's own table. `--no-endpoint-map`
/// switches to a probe connection that sends nothing, for bring-up on a node whose
/// descriptor transfer is broken.
struct MotorAdder<'a> {
    ctrl: &'a CyberBeastController,
    load_map: bool,
}

impl MotorAdder<'_> {
    fn add(
        &self,
        motor_id: u16,
        model: &str,
    ) -> Result<Arc<CyberBeastMotor>, Box<dyn std::error::Error>> {
        if self.load_map {
            Ok(self.ctrl.add_motor(motor_id, motor_id, model)?)
        } else {
            Ok(self.ctrl.add_motor_probe(motor_id, motor_id, model)?)
        }
    }
}

fn hex_bytes(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// Parse `--raw-bytes` (hex byte pairs, separated by spaces, commas or nothing).
fn parse_hex_bytes(text: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let digits: String = text
        .chars()
        .filter(|c| !c.is_whitespace() && *c != ',' && *c != '_')
        .collect();
    if digits.is_empty() || !digits.len().is_multiple_of(2) {
        return Err(format!("--raw-bytes {text} must be whole bytes, e.g. 08 00 00 00").into());
    }
    let mut bytes = Vec::with_capacity(digits.len() / 2);
    for pair in digits.as_bytes().chunks(2) {
        let pair = std::str::from_utf8(pair)?;
        bytes.push(
            u8::from_str_radix(pair, 16)
                .map_err(|_| format!("--raw-bytes {text}: {pair} is not a hex byte"))?,
        );
    }
    Ok(bytes)
}

/// Parse `--value` into the type the device declares for the endpoint.
///
/// The declaration decides the width: `axis0.requested_state` is a `uint8`, so `8` becomes
/// one value byte instead of a four-byte float.
fn parse_param_value(
    text: &str,
    declared: ValueType,
) -> Result<ParamValue, Box<dyn std::error::Error>> {
    let text = text.trim();
    let invalid = |what: &str| -> Box<dyn std::error::Error> {
        format!("--value {text} is not a valid {what}").into()
    };
    Ok(match declared {
        ValueType::F32 => ParamValue::F32(text.parse().map_err(|_| invalid("float"))?),
        ValueType::F64 => ParamValue::F64(text.parse().map_err(|_| invalid("float64"))?),
        ValueType::U8 => ParamValue::U8(text.parse().map_err(|_| invalid("uint8"))?),
        ValueType::U16 => ParamValue::U16(text.parse().map_err(|_| invalid("uint16"))?),
        ValueType::U32 => ParamValue::U32(text.parse().map_err(|_| invalid("uint32"))?),
        ValueType::U64 => ParamValue::U64(text.parse().map_err(|_| invalid("uint64"))?),
        ValueType::I8 => ParamValue::I8(text.parse().map_err(|_| invalid("int8"))?),
        ValueType::I16 => ParamValue::I16(text.parse().map_err(|_| invalid("int16"))?),
        ValueType::I32 => ParamValue::I32(text.parse().map_err(|_| invalid("int32"))?),
        ValueType::I64 => ParamValue::I64(text.parse().map_err(|_| invalid("int64"))?),
        ValueType::Bool => match text.to_ascii_lowercase().as_str() {
            "true" | "1" => ParamValue::Bool(true),
            "false" | "0" => ParamValue::Bool(false),
            _ => return Err(invalid("bool (true/false/1/0)")),
        },
    })
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
    // Connecting loads the device's endpoint table, so read-param/write-param never
    // have to guess a value type or fetch a table on first use.
    let adder = MotorAdder {
        ctrl: &ctrl,
        load_map: !args.contains_key("no-endpoint-map"),
    };
    if !adder.load_map {
        eprintln!(
            "warning: --no-endpoint-map: the device's endpoint table is not loaded, so parameter \
             reads will fail and --endpoint only accepts a numeric id"
        );
    }

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
                // A scan probes ids that are expected to be silent, so it must not load
                // every candidate's endpoint descriptor.
                let motor = match ctrl.add_motor_probe(id, id, model) {
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
            let motor = adder.add(motor_id, model)?;
            println!(
                "status for motor 0x{motor_id:02X} on {channel} (query-only: no StartMotor/StopMotor frame is sent)"
            );
            match motor.endpoint_map() {
                Some(map) => println!("  endpoint map: {}", map.summary()),
                None => println!("  endpoint map: not loaded (--no-endpoint-map)"),
            }
            let ranges = motor.mit_ranges();
            println!(
                "  mit-ranges  : pos=+/-{} rad, vel=+/-{} rad/s, kp<={}, kd<={}, tau=+/-{} Nm ({})",
                ranges.pos,
                ranges.vel,
                ranges.kp,
                ranges.kd,
                ranges.torque,
                if motor.mit_ranges_from_device() {
                    "from the device"
                } else {
                    "protocol defaults"
                }
            );
            println!(
                "  gear-ratio  : {} ({})",
                motor.gear_ratio(),
                if motor.gear_ratio_from_device() {
                    "motor-side rad = output-side rad x this"
                } else {
                    "not declared; motor-side and output-side left equal"
                }
            );
            if let Some(note) = motor.mit_ranges_note() {
                println!("                device ranges unavailable: {note}");
            }
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
            let motor = adder.add(motor_id, model)?;
            apply_mit_scaling(&motor, PARAM_TIMEOUT_MS);
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
            let motor = adder.add(motor_id, model)?;
            apply_mit_scaling(&motor, PARAM_TIMEOUT_MS);
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
            let motor = adder.add(motor_id, model)?;
            apply_mit_scaling(&motor, PARAM_TIMEOUT_MS);
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
            let motor = adder.add(motor_id, model)?;
            apply_mit_scaling(&motor, PARAM_TIMEOUT_MS);
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
            let motor = adder.add(motor_id, model)?;
            motor.send_start_motor()?;
            // Detach without stopping: `shutdown()` disables every motor, which undid the
            // StartMotor that was just sent (that is why this mode never left the axis in
            // closed loop). Every other mode keeps stopping the axis on exit.
            ctrl.close_bus()?;
            println!(
                "enabled motor 0x{motor_id:02X} (StartMotor 0x62 sent, priority 3); the axis stays in closed loop after this CLI exits"
            );
        }

        "disable" => {
            let motor = adder.add(motor_id, model)?;
            motor.send_stop_motor()?;
            println!("disabled motor 0x{motor_id:02X} (StopMotor 0x63 sent)");
            ctrl.shutdown()?;
        }

        "estop" => {
            let motor = adder.add(motor_id, model)?;
            println!(
                "warning: ESTOP is a global broadcast (Priority=0, MsgType=0xC0, Dest=0xFF per protocol 4.9): every device on {channel} stops and latches an ESTOP error, cleared only by clear-error or a reset"
            );
            motor.send_estop()?;
            println!("estop broadcast sent (Priority=0, mt=0xC0, dest=0xFF)");
            ctrl.shutdown()?;
        }

        "clear-error" | "clear-fault" => {
            let motor = adder.add(motor_id, model)?;
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
            let motor = adder.add(motor_id, model)?;
            motor.send_set_zero()?;
            println!(
                "set-zero (0x61) sent to 0x{motor_id:02X}; re-read status to confirm the new zero"
            );
            ctrl.shutdown()?;
        }

        // SDO endpoint read/write (fibre endpoint system, protocol 4.7 / 4.8).
        "read-param" => {
            let spec = get_endpoint_spec(args)?;
            let timeout_ms = get_u64(args, "timeout-ms", PARAM_TIMEOUT_MS)?;
            let motor = adder.add(motor_id, model)?;
            let endpoint = resolve_endpoint(&motor, &spec)?;
            // The device's own table says how wide this endpoint is, so the value is
            // decoded with its declared type instead of a guess.
            let readout = motor.read_param_value(endpoint, Duration::from_millis(timeout_ms))?;
            println!(
                "endpoint 0x{:04X} ({}) declared={} access={} value={}",
                readout.endpoint_id,
                readout.path.as_deref().unwrap_or("?"),
                readout.declared,
                readout.access.label(),
                readout.value
            );
            println!("  raw little-endian bytes: [{}]", hex_bytes(&readout.raw));
            ctrl.shutdown()?;
        }

        "write-param" => {
            let spec = get_endpoint_spec(args)?;
            // `--raw-bytes` sends an explicit payload, so it needs no --value.
            let raw_value = args.get("value").cloned();
            if raw_value.is_none() && !args.contains_key("raw-bytes") {
                return Err("--value <number|true|false> (or --raw-bytes <hex>) is required for write-param".into());
            }
            if !args.contains_key("yes") {
                return Err(
                    "write-param changes device configuration; re-run with --yes to confirm".into(),
                );
            }
            let timeout_ms = get_u64(args, "timeout-ms", PARAM_TIMEOUT_MS)?;
            let motor = adder.add(motor_id, model)?;
            let endpoint = resolve_endpoint(&motor, &spec)?;
            let timeout = Duration::from_millis(timeout_ms);
            // The width and the access flag come from the device's own table: a uint8
            // endpoint is written with one value byte, and a read-only one is refused.
            let map = motor.endpoint_map().ok_or(
                "write-param needs the device's endpoint table: drop --no-endpoint-map so the \
                 declared value width is known",
            )?;
            let entry = map.get(endpoint).ok_or_else(|| {
                format!("endpoint 0x{endpoint:04X} is not in the device's endpoint table")
            })?;
            let declared = entry.value_type().ok_or_else(|| {
                format!(
                    "endpoint 0x{endpoint:04X} ({}) is declared \"{}\" and cannot be written",
                    entry.path, entry.raw_type
                )
            })?;
            // Escape hatch: send exactly these value bytes (little-endian) instead of the
            // payload the declared type would build. Used to probe firmware quirks such as
            // whether a narrow endpoint accepts a 1-byte or only a 4-byte value.
            if let Some(hex) = args.get("raw-bytes") {
                let bytes = parse_hex_bytes(hex)?;
                let before = motor.read_param_value(endpoint, timeout).ok();
                motor.set_param_bytes(endpoint, &bytes)?;
                let read_back = motor.read_param_value(endpoint, timeout).ok();
                println!(
                    "endpoint {}: raw=[{}] before={:?} read_back={:?}",
                    entry.path,
                    hex_bytes(&bytes),
                    before.map(|readout| readout.value.to_string()),
                    read_back.map(|readout| readout.value.to_string())
                );
                println!("  note: explicit byte payload, so compare the read-back yourself");
                ctrl.shutdown()?;
                return Ok(());
            }

            let requested =
                parse_param_value(raw_value.as_deref().ok_or("--value is required")?, declared)?;
            let before = motor.read_param_value(endpoint, timeout).ok();
            motor.set_param_value(endpoint, requested)?;
            let read_back = motor.read_param_value(endpoint, timeout).ok();
            let show = |value: &Option<ParamReadout>| {
                value
                    .as_ref()
                    .map(|readout| format!("{} ({})", readout.value, readout.declared))
                    .unwrap_or_else(|| "unreadable".to_string())
            };
            let path = read_back
                .as_ref()
                .and_then(|readout| readout.path.clone())
                .unwrap_or_else(|| format!("0x{endpoint:04X}"));
            println!(
                "endpoint {path}: requested={requested} ({}) before={} read_back={}",
                declared.label(),
                show(&before),
                show(&read_back)
            );
            match read_back.map(|readout| readout.value) {
                Some(actual) if actual == requested => {
                    println!("  verified: the device reports the requested value");
                }
                Some(actual) => {
                    // `requested_state` is consumed by the state machine the moment it is
                    // written, so it can never read back as itself: say that instead of
                    // reporting a mismatch, and point at the endpoint that does settle.
                    if entry.path.ends_with("requested_state") {
                        println!(
                            "  note: the device consumed this write before it could be read back \
                             (it reports {actual}); check its effect through axis0.current_state"
                        );
                    } else {
                        return Err(format!(
                            "read-back mismatch on endpoint 0x{endpoint:04X}: requested {requested}, device reports {actual}"
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

        // Read-only: look an endpoint up by name in the device's own table.
        "find-endpoint" => {
            let Some(needle) = args.get("name") else {
                return Err("--name <substring> is required for find-endpoint".into());
            };
            let timeout_ms = get_u64(args, "timeout-ms", 500)?;
            let motor = adder.add(motor_id, model)?;
            let map = motor.ensure_endpoint_map(Duration::from_millis(timeout_ms))?;
            let mut shown = 0usize;
            for entry in map.find(needle) {
                shown += 1;
                println!(
                    "  id=0x{:04X} ({:5}) {:<9} {:<4} {}",
                    entry.endpoint_id,
                    entry.endpoint_id,
                    entry.raw_type,
                    entry.access.label(),
                    entry.path
                );
            }
            println!(
                "  {shown} of {} endpoints match \"{needle}\" ({})",
                map.len(),
                map.summary()
            );
            ctrl.shutdown()?;
        }

        // Read-only: the device's own endpoint descriptor (protocol 4.8).
        "endpoint-map" => {
            let timeout_ms = get_u64(args, "timeout-ms", 500)?;
            let motor = adder.add(motor_id, model)?;
            let timeout = Duration::from_millis(timeout_ms);
            let map = if args.contains_key("refresh") {
                println!(
                    "re-reading the endpoint descriptor from 0x{motor_id:02X} (JSON_DESC_READ 0x24), stall window {timeout_ms} ms"
                );
                motor.load_endpoint_map(timeout)?
            } else {
                // Already loaded when this motor was connected.
                motor.ensure_endpoint_map(timeout)?
            };
            println!("  descriptor: {}", map.summary());
            match args.get("out") {
                Some(path) => {
                    std::fs::write(path, map.json_text().as_bytes())
                        .map_err(|err| format!("cannot write {path}: {err}"))?;
                    println!("  wrote {path}");
                }
                None => {
                    println!("  (use --out <path> to save the JSON, or --dump to print it here)")
                }
            }
            if args.contains_key("dump") {
                print!("{}", map.json_text());
            }
            ctrl.shutdown()?;
        }

        // Passive link/state monitor: never transmits a frame.
        "monitor" => {
            // Passive by contract: connect without loading the endpoint map.
            let motor = ctrl.add_motor_probe(motor_id, motor_id, model)?;
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
            let motor = adder.add(motor_id, model)?;
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

    fn args_of(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(key, value)| ((*key).to_string(), (*value).to_string()))
            .collect()
    }

    #[test]
    fn get_endpoint_spec_accepts_ids_and_names() {
        // The spec stays a string: the device's table decides whether it is a name.
        assert_eq!(
            get_endpoint_spec(&args_of(&[("endpoint", "0x00F2")])).expect("id"),
            "0x00F2"
        );
        // `--param-id` is the Python CLI spelling of the same option.
        assert_eq!(
            get_endpoint_spec(&args_of(&[("param-id", "gear_ratio")])).expect("name"),
            "gear_ratio"
        );

        let err = get_endpoint_spec(&args_of(&[])).expect_err("missing endpoint");
        assert!(err.contains("--endpoint <id|name>"), "{err}");
    }
}
