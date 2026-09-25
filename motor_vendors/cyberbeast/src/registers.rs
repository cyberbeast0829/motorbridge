//! CyberBeast register table.
//!
//! The CyberBeast protocol uses ODrive's SDO endpoint system for parameter
//! access (via `MSG_PARAM_READ` / `MSG_PARAM_WRITE` with 16-bit endpoint IDs).
//! This is different from traditional fixed-register protocols like Damiao or
//! RobStride — the full parameter map is discovered at runtime via
//! `MSG_JSON_DESC_READ` (endpoint descriptor JSON).
//!
//! The ids are firmware-assigned hashes: they do **not** follow a "nice" scheme.
//! An earlier revision of this file guessed ids such as 0x0001 = `requested_state`
//! and 0x0019 = `torque_constant`; those guesses are wrong and were removed after
//! reading the device's own descriptor (see `REGISTER_TABLE`).
//!
//! This module provides a minimal static table of **verified** endpoint ids for
//! documentation and tooling. For complete parameter access, use the dynamic JSON
//! descriptor mechanism (protocol 4.8: `JSON_DESC_READ` 0x24 / `JSON_DESC_DATA` 0x25;
//! `tools/cb_json_probe.py` fetches and dumps it).

#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    /// SDO endpoint ID (16-bit).
    pub endpoint_id: u16,
    /// Human-readable variable name.
    pub variable: &'static str,
    /// Description.
    pub description: &'static str,
    /// Declared value type and access, exactly as reported by the device descriptor.
    ///
    /// Pick the matching reader: `float` → `get_param_f32` / `read_param_raw`,
    /// `uint8`/`uint16`/`uint32`/`int32`/`bool` → `read_param_raw` plus the matching
    /// `ParamReadResponse::as_*` accessor (`get_param_f32` rejects other widths).
    pub value_type: &'static str,
}

/// Verified SDO endpoints of a CyberBeast node.
///
/// Every id, name, type and access below was read from the device's own JSON
/// endpoint descriptor on firmware 0.6.9 (descriptor length 38433, VersionCRC
/// 0x3F82) with `tools/cb_json_probe.py`. The complete map holds 554 entries;
/// fetch it at runtime instead of extending this table by hand.
pub static REGISTER_TABLE: &[RegisterInfo] = &[
    // ── Board (odrv) ──
    RegisterInfo {
        endpoint_id: 0x0001,
        variable: "odrv.error",
        description: "Board (odrv) error flags",
        value_type: "uint8 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0002,
        variable: "odrv.vbus_voltage",
        description: "DC bus voltage (V)",
        value_type: "float r",
    },
    RegisterInfo {
        endpoint_id: 0x0003,
        variable: "odrv.ibus",
        description: "DC bus current (A)",
        value_type: "float r",
    },
    RegisterInfo {
        endpoint_id: 0x0005,
        variable: "odrv.serial_number",
        description: "MCU unique serial number (needs a segmented read)",
        value_type: "uint64 r",
    },
    // ── CAN controller ──
    RegisterInfo {
        endpoint_id: 0x0044,
        variable: "can.error",
        description: "CAN error flags",
        value_type: "uint8 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0045,
        variable: "can.config.baud_rate",
        description: "CAN bitrate (bit/s)",
        value_type: "uint32 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0046,
        variable: "can.config.protocol",
        description: "CAN protocol variant",
        value_type: "uint16 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0049,
        variable: "can.config.break_timeout",
        description: "CAN break timeout (ms)",
        value_type: "uint16 rw",
    },
    RegisterInfo {
        endpoint_id: 0x004A,
        variable: "can.config.auto_bus_off",
        description: "Enable automatic bus-off recovery",
        value_type: "bool rw",
    },
    RegisterInfo {
        endpoint_id: 0x004B,
        variable: "can.config.auto_retransmission",
        description: "Enable CAN automatic retransmission",
        value_type: "bool rw",
    },
    // ── Axis state and watchdog ──
    RegisterInfo {
        endpoint_id: 0x008E,
        variable: "axis0.current_state",
        description: "Current axis state (AXIS_STATE_*)",
        value_type: "uint8 r",
    },
    RegisterInfo {
        endpoint_id: 0x008F,
        variable: "axis0.requested_state",
        description: "Requested axis state (8 = CLOSED_LOOP_CONTROL)",
        value_type: "uint8 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0099,
        variable: "axis0.config.watchdog_timeout",
        description: "Axis watchdog timeout (s)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x009A,
        variable: "axis0.config.enable_watchdog",
        description: "Enable the axis watchdog",
        value_type: "bool rw",
    },
    RegisterInfo {
        endpoint_id: 0x00B4,
        variable: "axis0.config.can.node_id",
        description: "CAN node id of this axis",
        value_type: "uint32 rw",
    },
    RegisterInfo {
        endpoint_id: 0x00B5,
        variable: "axis0.config.can.is_extended",
        description: "Use 29-bit extended CAN ids",
        value_type: "bool rw",
    },
    RegisterInfo {
        endpoint_id: 0x00B6,
        variable: "axis0.config.can.heartbeat_rate_ms",
        description: "Heartbeat period (ms)",
        value_type: "uint32 rw",
    },
    // ── Motor ──
    RegisterInfo {
        endpoint_id: 0x00CC,
        variable: "axis0.motor.effective_current_lim",
        description: "Effective current limit after clamping (A)",
        value_type: "float r",
    },
    RegisterInfo {
        endpoint_id: 0x00F1,
        variable: "axis0.motor.config.pole_pairs",
        description: "Motor pole pairs",
        value_type: "int32 rw",
    },
    RegisterInfo {
        endpoint_id: 0x00F3,
        variable: "axis0.motor.config.calibration_current",
        description: "Calibration current (A)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x00F7,
        variable: "axis0.motor.config.torque_constant",
        description: "Torque constant (Nm/A)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x00F9,
        variable: "axis0.motor.config.current_lim",
        description: "Configured current limit (A)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x00FB,
        variable: "axis0.motor.config.torque_lim",
        description: "Torque limit (Nm)",
        value_type: "float rw",
    },
    // ── Controller ──
    RegisterInfo {
        endpoint_id: 0x011F,
        variable: "axis0.controller.config.control_mode",
        description: "Control mode (see firmware CONTROL_MODE enum)",
        value_type: "uint8 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0120,
        variable: "axis0.controller.config.input_mode",
        description: "Input mode (see firmware INPUT_MODE enum)",
        value_type: "uint8 rw",
    },
    RegisterInfo {
        endpoint_id: 0x0123,
        variable: "axis0.controller.config.pos_gain",
        description: "Position gain (turns/s per turn)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x0127,
        variable: "axis0.controller.config.vel_gain",
        description: "Velocity gain (Nm per turn/s)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x012D,
        variable: "axis0.controller.config.vel_limit",
        description: "Velocity limit (turns/s)",
        value_type: "float rw",
    },
    // ── MIT limits ──
    RegisterInfo {
        endpoint_id: 0x014F,
        variable: "axis0.controller.config.mit_max_pos",
        description: "MIT max position (rad)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x0150,
        variable: "axis0.controller.config.mit_max_vel",
        description: "MIT max velocity (rad/s)",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x0151,
        variable: "axis0.controller.config.mit_max_torque",
        description:
            "MIT max torque (Nm); with torque_constant it sets the MIT response current range",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x0152,
        variable: "axis0.controller.config.mit_max_kp",
        description: "MIT max Kp",
        value_type: "float rw",
    },
    RegisterInfo {
        endpoint_id: 0x0153,
        variable: "axis0.controller.config.mit_max_kd",
        description: "MIT max Kd",
        value_type: "float rw",
    },
    // ── Encoder ──
    RegisterInfo {
        endpoint_id: 0x0186,
        variable: "axis0.encoder.config.cpr",
        description: "Encoder counts per revolution",
        value_type: "int32 rw",
    },
];
