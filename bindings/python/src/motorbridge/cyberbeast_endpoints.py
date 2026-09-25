from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EndpointSpec:
    endpoint_id: int
    variable: str
    description: str
    value_type: str = ""


# Verified CyberBeast SDO endpoint IDs (ODrive/"fibre" endpoint system).
#
# Mirrors `motor_vendors/cyberbeast/src/registers.rs` (`REGISTER_TABLE`). Every
# id, name, description and type below was read from the device's own JSON
# endpoint descriptor (`JSON_DESC_READ` 0x24 / `JSON_DESC_DATA` 0x25) on firmware
# 0.6.9 (descriptor length 38433, VersionCRC 0x3F82). The full map holds 554
# entries; fetch it at runtime instead of extending this table by hand.
#
# Note: the id-to-name mapping is firmware-assigned and does not follow a "nice"
# scheme. Earlier revisions of this mirror claimed 0x0001 = requested_state and
# 0x0019 = torque_constant; both were wrong (0x0001 is `odrv.error`, the real
# `torque_constant` is 0x00F7).
CYBERBEAST_ENDPOINTS: dict[int, EndpointSpec] = {
    0x0001: EndpointSpec(0x0001, "odrv.error", "Board (odrv) error flags", "uint8 rw"),
    0x0002: EndpointSpec(0x0002, "odrv.vbus_voltage", "DC bus voltage (V)", "float r"),
    0x0003: EndpointSpec(0x0003, "odrv.ibus", "DC bus current (A)", "float r"),
    0x0005: EndpointSpec(
        0x0005,
        "odrv.serial_number",
        "MCU unique serial number (needs a segmented read)",
        "uint64 r",
    ),
    0x0044: EndpointSpec(0x0044, "can.error", "CAN error flags", "uint8 rw"),
    0x0045: EndpointSpec(0x0045, "can.config.baud_rate", "CAN bitrate (bit/s)", "uint32 rw"),
    0x0046: EndpointSpec(0x0046, "can.config.protocol", "CAN protocol variant", "uint16 rw"),
    0x0049: EndpointSpec(0x0049, "can.config.break_timeout", "CAN break timeout (ms)", "uint16 rw"),
    0x004A: EndpointSpec(
        0x004A, "can.config.auto_bus_off", "Enable automatic bus-off recovery", "bool rw"
    ),
    0x004B: EndpointSpec(
        0x004B, "can.config.auto_retransmission", "Enable CAN automatic retransmission", "bool rw"
    ),
    0x008E: EndpointSpec(0x008E, "axis0.current_state", "Current axis state (AXIS_STATE_*)", "uint8 r"),
    0x008F: EndpointSpec(
        0x008F,
        "axis0.requested_state",
        "Requested axis state (8 = CLOSED_LOOP_CONTROL)",
        "uint8 rw",
    ),
    0x0099: EndpointSpec(
        0x0099, "axis0.config.watchdog_timeout", "Axis watchdog timeout (s)", "float rw"
    ),
    0x009A: EndpointSpec(0x009A, "axis0.config.enable_watchdog", "Enable the axis watchdog", "bool rw"),
    0x00B4: EndpointSpec(0x00B4, "axis0.config.can.node_id", "CAN node id of this axis", "uint32 rw"),
    0x00B5: EndpointSpec(
        0x00B5, "axis0.config.can.is_extended", "Use 29-bit extended CAN ids", "bool rw"
    ),
    0x00B6: EndpointSpec(
        0x00B6, "axis0.config.can.heartbeat_rate_ms", "Heartbeat period (ms)", "uint32 rw"
    ),
    0x00CC: EndpointSpec(
        0x00CC,
        "axis0.motor.effective_current_lim",
        "Effective current limit after clamping (A)",
        "float r",
    ),
    0x00F1: EndpointSpec(0x00F1, "axis0.motor.config.pole_pairs", "Motor pole pairs", "int32 rw"),
    0x00F3: EndpointSpec(
        0x00F3, "axis0.motor.config.calibration_current", "Calibration current (A)", "float rw"
    ),
    0x00F7: EndpointSpec(
        0x00F7, "axis0.motor.config.torque_constant", "Torque constant (Nm/A)", "float rw"
    ),
    0x00F9: EndpointSpec(
        0x00F9, "axis0.motor.config.current_lim", "Configured current limit (A)", "float rw"
    ),
    0x00FB: EndpointSpec(0x00FB, "axis0.motor.config.torque_lim", "Torque limit (Nm)", "float rw"),
    0x011F: EndpointSpec(
        0x011F,
        "axis0.controller.config.control_mode",
        "Control mode (see firmware CONTROL_MODE enum)",
        "uint8 rw",
    ),
    0x0120: EndpointSpec(
        0x0120,
        "axis0.controller.config.input_mode",
        "Input mode (see firmware INPUT_MODE enum)",
        "uint8 rw",
    ),
    0x0123: EndpointSpec(
        0x0123,
        "axis0.controller.config.pos_gain",
        "Position gain (turns/s per turn)",
        "float rw",
    ),
    0x0127: EndpointSpec(
        0x0127, "axis0.controller.config.vel_gain", "Velocity gain (Nm per turn/s)", "float rw"
    ),
    0x012D: EndpointSpec(
        0x012D, "axis0.controller.config.vel_limit", "Velocity limit (turns/s)", "float rw"
    ),
    0x014F: EndpointSpec(
        0x014F, "axis0.controller.config.mit_max_pos", "MIT max position (rad)", "float rw"
    ),
    0x0150: EndpointSpec(
        0x0150, "axis0.controller.config.mit_max_vel", "MIT max velocity (rad/s)", "float rw"
    ),
    0x0151: EndpointSpec(
        0x0151,
        "axis0.controller.config.mit_max_torque",
        "MIT max torque (Nm); with torque_constant it sets the MIT response current range",
        "float rw",
    ),
    0x0152: EndpointSpec(0x0152, "axis0.controller.config.mit_max_kp", "MIT max Kp", "float rw"),
    0x0153: EndpointSpec(0x0153, "axis0.controller.config.mit_max_kd", "MIT max Kd", "float rw"),
    0x0186: EndpointSpec(
        0x0186, "axis0.encoder.config.cpr", "Encoder counts per revolution", "int32 rw"
    ),
}

# Names for the endpoints that come up most often in scripts and CLI examples.
EP_ODRV_ERROR = 0x0001
EP_ODRV_VBUS_VOLTAGE = 0x0002
EP_ODRV_IBUS = 0x0003
EP_ODRV_SERIAL_NUMBER = 0x0005
EP_CAN_ERROR = 0x0044
EP_CAN_BAUD_RATE = 0x0045
EP_CAN_PROTOCOL = 0x0046
EP_CAN_BREAK_TIMEOUT = 0x0049
EP_CAN_AUTO_BUS_OFF = 0x004A
EP_CAN_AUTO_RETRANSMISSION = 0x004B
EP_AXIS_CURRENT_STATE = 0x008E
EP_AXIS_REQUESTED_STATE = 0x008F
EP_AXIS_WATCHDOG_TIMEOUT = 0x0099
EP_AXIS_ENABLE_WATCHDOG = 0x009A
EP_AXIS_CAN_NODE_ID = 0x00B4
EP_AXIS_CAN_IS_EXTENDED = 0x00B5
EP_AXIS_CAN_HEARTBEAT_RATE_MS = 0x00B6
EP_MOTOR_EFFECTIVE_CURRENT_LIM = 0x00CC
EP_MOTOR_POLE_PAIRS = 0x00F1
EP_MOTOR_CALIBRATION_CURRENT = 0x00F3
EP_MOTOR_TORQUE_CONSTANT = 0x00F7
EP_MOTOR_CURRENT_LIM = 0x00F9
EP_MOTOR_TORQUE_LIM = 0x00FB
EP_CONTROLLER_CONTROL_MODE = 0x011F
EP_CONTROLLER_INPUT_MODE = 0x0120
EP_CONTROLLER_POS_GAIN = 0x0123
EP_CONTROLLER_VEL_GAIN = 0x0127
EP_CONTROLLER_VEL_LIMIT = 0x012D
EP_MIT_MAX_POS = 0x014F
EP_MIT_MAX_VEL = 0x0150
EP_MIT_MAX_TORQUE = 0x0151
EP_MIT_MAX_KP = 0x0152
EP_MIT_MAX_KD = 0x0153
EP_ENCODER_CPR = 0x0186


def get_cyberbeast_endpoint(endpoint_id: int) -> EndpointSpec | None:
    return CYBERBEAST_ENDPOINTS.get(endpoint_id)
