from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EndpointSpec:
    endpoint_id: int
    variable: str
    description: str


# Commonly used ODrive SDO endpoint IDs for the CyberBeast protocol.
#
# Mirrors `motor_vendors/cyberbeast/src/registers.rs` (`REGISTER_TABLE`). The
# full endpoint map (hundreds of entries, with their real value types) is
# discovered at runtime via `MSG_JSON_DESC_READ`; the ABI binding exposes the
# float32 read/write path only, so keep this table to float32 endpoints.
CYBERBEAST_ENDPOINTS: dict[int, EndpointSpec] = {
    0x0000: EndpointSpec(0x0000, "axis.current_state", "Current axis state"),
    0x0001: EndpointSpec(0x0001, "axis.requested_state", "Requested axis state (AXIS_STATE_*)"),
    0x0012: EndpointSpec(0x0012, "axis.error", "Axis error flags"),
    0x0019: EndpointSpec(0x0019, "motor.config.torque_constant", "Motor torque constant (Nm/A)"),
    0x001C: EndpointSpec(0x001C, "motor.config.current_lim", "Motor current limit (A)"),
    0x001D: EndpointSpec(0x001D, "motor.error", "Motor error flags"),
    0x002C: EndpointSpec(0x002C, "encoder.error", "Encoder error flags"),
    0x0030: EndpointSpec(0x0030, "controller.config.control_mode", "Control mode (POSITION/VELOCITY/TORQUE)"),
    0x0031: EndpointSpec(
        0x0031,
        "controller.config.input_mode",
        "Input mode (PASSTHROUGH/POS_FILTER/VEL_RAMP/TORQUE_RAMP/MIT)",
    ),
    0x0035: EndpointSpec(0x0035, "controller.config.pos_gain", "Position gain (turns/s per turn)"),
    0x0036: EndpointSpec(0x0036, "controller.config.vel_gain", "Velocity gain (Nm per turn/s)"),
    0x0037: EndpointSpec(
        0x0037,
        "controller.config.vel_integrator_gain",
        "Velocity integrator gain (Nm per turn)",
    ),
    0x0039: EndpointSpec(0x0039, "controller.config.vel_limit", "Velocity limit (turns/s)"),
    0x003D: EndpointSpec(0x003D, "controller.error", "Controller error flags"),
    0x0100: EndpointSpec(0x0100, "can.config.node_id", "CAN node ID"),
    0x0101: EndpointSpec(0x0101, "can.config.break_timeout", "CAN break timeout (ms, 0 = default 100ms)"),
    0x0300: EndpointSpec(0x0300, "controller.config.mit_max_pos", "MIT max position (rad)"),
    0x0301: EndpointSpec(0x0301, "controller.config.mit_max_vel", "MIT max velocity (rad/s)"),
    0x0302: EndpointSpec(0x0302, "controller.config.mit_max_kp", "MIT max Kp"),
    0x0303: EndpointSpec(0x0303, "controller.config.mit_max_kd", "MIT max Kd"),
    0x0304: EndpointSpec(0x0304, "controller.config.mit_max_torque", "MIT max torque (Nm)"),
}

# Names for the endpoints that come up most often in scripts and CLI examples.
EP_AXIS_CURRENT_STATE = 0x0000
EP_AXIS_REQUESTED_STATE = 0x0001
EP_AXIS_ERROR = 0x0012
EP_MOTOR_TORQUE_CONSTANT = 0x0019
EP_MOTOR_CURRENT_LIM = 0x001C
EP_MOTOR_ERROR = 0x001D
EP_ENCODER_ERROR = 0x002C
EP_CONTROLLER_CONTROL_MODE = 0x0030
EP_CONTROLLER_INPUT_MODE = 0x0031
EP_CONTROLLER_POS_GAIN = 0x0035
EP_CONTROLLER_VEL_GAIN = 0x0036
EP_CONTROLLER_VEL_LIMIT = 0x0039
EP_CONTROLLER_ERROR = 0x003D
EP_CAN_NODE_ID = 0x0100
EP_CAN_BREAK_TIMEOUT = 0x0101
EP_MIT_MAX_POS = 0x0300
EP_MIT_MAX_VEL = 0x0301
EP_MIT_MAX_KP = 0x0302
EP_MIT_MAX_KD = 0x0303
EP_MIT_MAX_TORQUE = 0x0304


def get_cyberbeast_endpoint(endpoint_id: int) -> EndpointSpec | None:
    return CYBERBEAST_ENDPOINTS.get(endpoint_id)
