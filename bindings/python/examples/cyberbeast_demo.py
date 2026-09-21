#!/usr/bin/env python3
"""CyberBeast (ODrive SDO) demo on the classic-CAN path.

CyberBeast uses big-endian extended CAN IDs and ODrive SDO endpoints instead of
a fixed register table, so parameter access goes through
``cyberbeast_get_param_f32`` / ``cyberbeast_write_param_f32``.

Usage:
    PYTHONPATH=bindings/python/src python3 bindings/python/examples/cyberbeast_demo.py \
        --channel can0 --model odrive-default --motor-id 0x01 --mode mit --loop 20 --dt-ms 20

Read-only endpoint probe (no control frames besides the query):
    PYTHONPATH=bindings/python/src python3 bindings/python/examples/cyberbeast_demo.py \
        --channel can0 --motor-id 0x01 --mode probe
"""
from __future__ import annotations

import argparse
import time

from motorbridge import Controller, Mode, get_cyberbeast_endpoint

# Endpoints probed by --mode probe; they are documented in
# motorbridge.cyberbeast_endpoints (mirrors registers.rs REGISTER_TABLE).
PROBE_ENDPOINTS = (0x0030, 0x0304, 0x0019, 0x0012)


def _parse_id(text: str) -> int:
    return int(text, 0)


def _describe(endpoint_id: int) -> str:
    spec = get_cyberbeast_endpoint(endpoint_id)
    return spec.variable if spec else "unknown"


def main() -> None:
    p = argparse.ArgumentParser(description="CyberBeast classic-CAN demo (ODrive SDO endpoints)")
    p.add_argument("--channel", default="can0", help="SocketCAN/PCAN channel (classic CAN, not CAN-FD)")
    p.add_argument("--model", default="odrive-default", help="catalog model: odrive-default|odrive-pro|odrive-high-torque|odrive-high-speed")
    p.add_argument("--motor-id", default="0x01", help="CyberBeast CAN node ID, hex or decimal (0..255)")
    p.add_argument("--feedback-id", default="0x01", help="accepted for signature parity; CyberBeast feedback is addressed by motor-id")
    p.add_argument("--mode", default="mit", choices=["probe", "mit", "pos-vel", "vel", "force-pos"])
    p.add_argument("--loop", type=int, default=20, help="control cycles")
    p.add_argument("--dt-ms", type=int, default=20, help="period between control frames in ms")
    p.add_argument("--ensure-timeout-ms", type=int, default=1000, help="mode guard timeout")
    p.add_argument("--pos", type=float, default=0.0, help="target position in rad")
    p.add_argument("--vel", type=float, default=0.0, help="target velocity in rad/s")
    p.add_argument("--vlim", type=float, default=1.0, help="velocity limit for pos-vel in rad/s")
    p.add_argument("--kp", type=float, default=30.0, help="MIT kp")
    p.add_argument("--kd", type=float, default=1.0, help="MIT kd")
    p.add_argument("--tau", type=float, default=0.0, help="MIT feed-forward torque in Nm")
    p.add_argument(
        "--ratio",
        type=float,
        default=0.0,
        help="force-pos torque ratio; torque = ratio * model MIT torque limit (18 Nm)",
    )
    p.add_argument("--read-endpoint", default="", help="extra SDO endpoint to read, hex or decimal")
    p.add_argument("--write-endpoint", default="", help="SDO endpoint to write before the control loop")
    p.add_argument("--write-value", type=float, default=0.0, help="value for --write-endpoint")
    p.add_argument("--save", type=int, default=0, help="send CONFIG_SAVE after a verified write, 1/0")
    p.add_argument("--timeout-ms", type=int, default=1000, help="endpoint read/write timeout in ms (ABI floor 200)")
    args = p.parse_args()

    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)

    print(
        f"vendor=cyberbeast transport=socketcan channel={args.channel} model={args.model} "
        f"motor_id=0x{motor_id:02X} feedback_id=0x{feedback_id:02X} mode={args.mode}"
    )

    with Controller(args.channel) as ctrl:
        motor = ctrl.add_cyberbeast_motor(motor_id, feedback_id, args.model)
        try:
            if args.mode == "probe" or args.read_endpoint:
                endpoints = list(PROBE_ENDPOINTS)
                if args.read_endpoint:
                    endpoints.append(_parse_id(args.read_endpoint))
                for endpoint_id in endpoints:
                    try:
                        value = motor.cyberbeast_get_param_f32(endpoint_id, args.timeout_ms)
                    except Exception as e:
                        print(f"endpoint 0x{endpoint_id:04X} {_describe(endpoint_id)}: read failed ({e})")
                        continue
                    print(f"endpoint 0x{endpoint_id:04X} {_describe(endpoint_id)} = {value}")

            if args.write_endpoint:
                endpoint_id = _parse_id(args.write_endpoint)
                requested = float(args.write_value)
                motor.cyberbeast_write_param_f32(endpoint_id, requested)
                time.sleep(0.05)
                value = motor.cyberbeast_get_param_f32(endpoint_id, args.timeout_ms)
                verified = abs(value - requested) <= max(1e-6, abs(requested) * 1e-6)
                print(
                    f"write endpoint 0x{endpoint_id:04X} {_describe(endpoint_id)}: "
                    f"requested={requested} value={value} verified={verified}"
                )
                if not verified:
                    print(
                        "[warn] read back differs from the requested value; the device clamped it or "
                        "consumed it immediately (axis.requested_state is consumed by the state machine)"
                    )
                if args.save:
                    motor.store_parameters()
                    print("[ok] CONFIG_SAVE requested")

            if args.mode == "probe":
                return

            ctrl.enable_all()  # CyberBeast enable = AXIS_STATE_CLOSED_LOOP start
            mode = {
                "mit": Mode.MIT,
                "pos-vel": Mode.POS_VEL,
                "vel": Mode.VEL,
                "force-pos": Mode.FORCE_POS,
            }[args.mode]
            motor.ensure_mode(mode, args.ensure_timeout_ms)

            for i in range(args.loop):
                if args.mode == "mit":
                    motor.send_mit(args.pos, args.vel, args.kp, args.kd, args.tau)
                elif args.mode == "pos-vel":
                    motor.send_pos_vel(args.pos, args.vlim)
                elif args.mode == "vel":
                    motor.send_vel(args.vel)
                else:
                    motor.send_force_pos(args.pos, args.vlim, args.ratio)

                st = motor.get_state()
                if st is None:
                    print(f"#{i} no feedback yet")
                else:
                    # The unified state maps CyberBeast current to torq and the two
                    # device temperatures to t_mos/t_rotor.
                    print(
                        f"#{i} pos={st.pos:+.4f} vel={st.vel:+.4f} current={st.torq:+.3f}A "
                        f"err=0x{st.status_code:X} mos_temp={st.t_mos:.1f}C motor_temp={st.t_rotor:.1f}C"
                    )
                if args.dt_ms > 0:
                    time.sleep(args.dt_ms / 1000.0)
        finally:
            motor.close()


if __name__ == "__main__":
    main()
