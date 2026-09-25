"""CyberBeast Python binding coverage.

The tests intentionally avoid a real CAN device:

* the ABI is replaced by a recorder so call arguments and error handling are
  checked without the shared library, and
* the endpoint table is cross-checked against the Rust source of truth
  (`motor_vendors/cyberbeast/src/registers.rs`) so it cannot silently drift.
"""
from __future__ import annotations

import argparse
import inspect
import json
import re
from pathlib import Path

import pytest

from motorbridge import Controller, Motor, cyberbeast_endpoints
from motorbridge.cli import common as cli_common
from motorbridge.cli import run as cli_run
from motorbridge.cli import scan as cli_scan
from motorbridge.errors import CallError
from motorbridge.models import Mode

REPO_ROOT = Path(__file__).resolve().parents[3]


# ---------------------------------------------------------------------------
# fake ABI plumbing
# ---------------------------------------------------------------------------
class _Recorder:
    def __init__(self, lib: "FakeLib", name: str) -> None:
        self._lib = lib
        self._name = name

    def __call__(self, *args):
        self._lib.calls.append((self._name, args))
        handler = self._lib.handlers.get(self._name)
        if handler is not None:
            return handler(*args)
        return self._lib.returns.get(self._name, 0)


class FakeLib:
    def __init__(self, returns: dict | None = None, handlers: dict | None = None) -> None:
        self.returns = dict(returns or {})
        self.handlers = dict(handlers or {})
        self.calls: list[tuple[str, tuple]] = []
        self._funcs: dict[str, _Recorder] = {}

    def __getattr__(self, name: str) -> _Recorder:
        if name.startswith("_"):
            raise AttributeError(name)
        recorder = self._funcs.get(name)
        if recorder is None:
            recorder = _Recorder(self, name)
            self._funcs[name] = recorder
        return recorder

    def args_of(self, name: str) -> tuple:
        matches = [args for called, args in self.calls if called == name]
        assert matches, f"{name} was never called; calls={[c[0] for c in self.calls]}"
        return matches[-1]


class FakeAbi:
    def __init__(self, lib: FakeLib) -> None:
        self.lib = lib


@pytest.fixture()
def fake_abi(monkeypatch):
    def _install(returns: dict | None = None, handlers: dict | None = None) -> tuple[FakeAbi, object]:
        lib = FakeLib(returns=returns, handlers=handlers)
        abi = FakeAbi(lib)
        monkeypatch.setattr("motorbridge.core.get_abi", lambda: abi)
        return abi, lib

    return _install


def _fake_controller(lib: FakeLib, ptr: int = 0xDEAD) -> Controller:
    ctrl = Controller.__new__(Controller)
    ctrl._abi = FakeAbi(lib)
    ctrl._ptr = ptr
    return ctrl


def _fake_motor(lib: FakeLib, ptr: int = 0xBEEF) -> Motor:
    motor = Motor.__new__(Motor)
    motor._abi = FakeAbi(lib)
    motor._ptr = ptr
    return motor


# ---------------------------------------------------------------------------
# binding: controller / motor surface
# ---------------------------------------------------------------------------
def test_add_cyberbeast_motor_forwards_arguments(fake_abi) -> None:
    _, lib = fake_abi(returns={"motor_controller_add_cyberbeast_motor": 0x1234})
    ctrl = _fake_controller(lib)

    # Distinct values so a swapped motor_id/feedback_id cannot pass unnoticed.
    motor = ctrl.add_cyberbeast_motor(0x03, 0x07, "odrive-pro")

    assert lib.args_of("motor_controller_add_cyberbeast_motor") == (
        0xDEAD,
        0x03,
        0x07,
        b"odrive-pro",
    )
    assert isinstance(motor, Motor)
    assert motor._ptr == 0x1234


def test_add_cyberbeast_motor_reports_abi_failure(fake_abi) -> None:
    _, lib = fake_abi(
        returns={
            "motor_controller_add_cyberbeast_motor": 0,
            "motor_last_error_message": b"unknown cyberbeast model: nope",
        }
    )
    ctrl = _fake_controller(lib)

    with pytest.raises(CallError) as excinfo:
        ctrl.add_cyberbeast_motor(1, 1, "nope")

    assert "add_cyberbeast_motor failed" in str(excinfo.value)
    assert "unknown cyberbeast model: nope" in str(excinfo.value)


@pytest.mark.parametrize(
    "motor_id,feedback_id",
    [(256, 1), (-1, 1), (1, 256), (1, -1)],
)
def test_add_cyberbeast_motor_rejects_out_of_range_ids(fake_abi, motor_id, feedback_id) -> None:
    _, lib = fake_abi()
    ctrl = _fake_controller(lib)

    with pytest.raises(ValueError):
        ctrl.add_cyberbeast_motor(motor_id, feedback_id, "odrive-default")

    assert lib.calls == []


def test_cyberbeast_get_param_f32_plumbs_value_and_timeout(fake_abi) -> None:
    def handler(_motor_ptr, param_id, timeout_ms, out) -> int:
        out._obj.value = 12.5
        return 0

    _, lib = fake_abi(handlers={"motor_handle_cyberbeast_get_param_f32": handler})
    motor = _fake_motor(lib)

    value = motor.cyberbeast_get_param_f32(0x0304)

    assert value == pytest.approx(12.5)
    assert lib.args_of("motor_handle_cyberbeast_get_param_f32")[:3] == (0xBEEF, 0x0304, 1000)

    motor.cyberbeast_get_param_f32(0x0019, 250)
    assert lib.args_of("motor_handle_cyberbeast_get_param_f32")[:3] == (0xBEEF, 0x0019, 250)


def test_cyberbeast_write_param_f32_forwards_value(fake_abi) -> None:
    _, lib = fake_abi()
    motor = _fake_motor(lib)

    motor.cyberbeast_write_param_f32(0x0030, 2.0)

    assert lib.args_of("motor_handle_cyberbeast_write_param_f32") == (0xBEEF, 0x0030, 2.0)


def test_cyberbeast_param_helpers_surface_foreign_handle_error(fake_abi) -> None:
    """A CyberBeast helper called on another vendor's handle must fail loudly."""
    _, lib = fake_abi(
        returns={
            "motor_handle_cyberbeast_get_param_f32": -1,
            "motor_last_error_message": b"motor is not a CyberBeast motor",
        }
    )
    motor = _fake_motor(lib)

    with pytest.raises(CallError) as excinfo:
        motor.cyberbeast_get_param_f32(0x0304)

    assert "motor is not a CyberBeast motor" in str(excinfo.value)

    abi_source = (REPO_ROOT / "motor_abi" / "src" / "vendor_params" / "cyberbeast.rs").read_text(
        encoding="utf-8"
    )
    assert "motor is not a CyberBeast motor" in abi_source


# ---------------------------------------------------------------------------
# endpoint table parity with the Rust register table
# ---------------------------------------------------------------------------
_ENTRY_RE = re.compile(
    r"endpoint_id:\s*(0x[0-9A-Fa-f]+),\s*"
    r'variable:\s*"([^"]+)",\s*'
    r'description:\s*"([^"]+)",\s*'
    r'value_type:\s*"([^"]+)",',
    re.S,
)


def test_endpoint_table_matches_rust_register_table() -> None:
    rust = (REPO_ROOT / "motor_vendors" / "cyberbeast" / "src" / "registers.rs").read_text(
        encoding="utf-8"
    )
    rust_entries = {
        int(endpoint_id, 16): (variable, description, value_type)
        for endpoint_id, variable, description, value_type in _ENTRY_RE.findall(rust)
    }
    assert rust_entries, "failed to parse REGISTER_TABLE from registers.rs"

    python_entries = {
        endpoint_id: (spec.variable, spec.description, spec.value_type)
        for endpoint_id, spec in cyberbeast_endpoints.CYBERBEAST_ENDPOINTS.items()
    }
    assert python_entries == rust_entries


def test_endpoint_names_match_table() -> None:
    table = cyberbeast_endpoints.CYBERBEAST_ENDPOINTS
    for name in dir(cyberbeast_endpoints):
        if not name.startswith("EP_"):
            continue
        endpoint_id = getattr(cyberbeast_endpoints, name)
        assert endpoint_id in table, f"{name} points at unknown endpoint 0x{endpoint_id:04X}"

    assert cyberbeast_endpoints.get_cyberbeast_endpoint(0x0153) is table[0x0153]
    assert cyberbeast_endpoints.get_cyberbeast_endpoint(0xFFFF) is None


# ---------------------------------------------------------------------------
# API surface parity
# ---------------------------------------------------------------------------
def _surface() -> dict:
    return json.loads((REPO_ROOT / "bindings" / "api_surface.json").read_text(encoding="utf-8"))


def test_api_surface_lists_cyberbeast_entries() -> None:
    surface = _surface()

    assert "motor_controller_add_cyberbeast_motor" in surface["abi"]["controller"]
    assert surface["abi"]["cyberbeast"] == [
        "motor_handle_cyberbeast_get_param_f32",
        "motor_handle_cyberbeast_write_param_f32",
    ]
    assert "cyberbeast" in surface["vendors"]
    assert "Controller.add_cyberbeast_motor(motor_id, feedback_id, model)" in surface["bindings"]["controller_methods"]
    assert "Motor.cyberbeast_get_param_f32(param_id, timeout_ms)" in surface["bindings"]["motor_methods"]
    assert "Motor.cyberbeast_write_param_f32(param_id, value)" in surface["bindings"]["motor_methods"]


def test_api_surface_and_abi_bindings_are_bidirectionally_covered() -> None:
    """Every symbol listed in api_surface.json is bound, and every bound
    symbol is listed -- otherwise a new ABI entry point can be added to the
    binding without being tracked (or vice versa)."""
    surface = _surface()
    listed = {
        name
        for names in surface["abi"].values()
        for name in names
        if name.startswith("motor_")
    }
    abi_source = (REPO_ROOT / "bindings" / "python" / "src" / "motorbridge" / "abi.py").read_text(
        encoding="utf-8"
    )
    bound = set(re.findall(r"\blib\.(motor_\w+)", abi_source))

    assert bound - listed == set()
    assert listed - bound == set()


def test_api_surface_python_methods_exist_with_listed_parameters() -> None:
    """Guards against renamed/removed parameters in the public Python API."""
    surface = _surface()
    classes = {"Controller": Controller, "Motor": Motor}
    entries = surface["bindings"]["controller_methods"] + surface["bindings"]["motor_methods"]
    checked = 0
    for entry in entries:
        match = re.fullmatch(r"(Controller|Motor)\.(\w+)\((.*)\)", entry)
        if match is None:
            continue
        cls = classes[match.group(1)]
        raw = inspect.getattr_static(cls, match.group(2))
        func = raw.__func__ if isinstance(raw, (classmethod, staticmethod)) else raw
        listed = [part.strip() for part in match.group(3).split(",") if part.strip()]
        actual = list(inspect.signature(func).parameters)
        if actual and actual[0] in ("self", "cls"):
            actual = actual[1:]
        assert actual[: len(listed)] == listed, entry
        checked += 1
    assert checked >= 30


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def test_cli_vendor_defaults_for_cyberbeast() -> None:
    assert cli_common._vendor_defaults("cyberbeast", "4340", "0x11") == ("odrive-default", "0x01")
    assert cli_common._vendor_defaults("cyberbeast", "odrive-pro", "0x02") == ("odrive-pro", "0x02")


def test_cli_add_motor_dispatches_cyberbeast() -> None:
    calls: list[tuple] = []

    class FakeCtrl:
        def add_cyberbeast_motor(self, motor_id: int, feedback_id: int, model: str):
            calls.append((motor_id, feedback_id, model))
            return "motor"

    assert cli_common._add_motor(FakeCtrl(), "cyberbeast", 3, 3, "odrive-default") == "motor"
    assert calls == [(3, 3, "odrive-default")]


def test_cli_rejects_canfd_transport_for_cyberbeast() -> None:
    args = argparse.Namespace(transport="socketcanfd", channel="can0")

    with pytest.raises(ValueError) as excinfo:
        cli_common._open_controller(args, "cyberbeast")

    assert "no CAN-FD path" in str(excinfo.value)

    # The binding documents the same limit (classic CAN only) without quoting an
    # ABI message, so the binding stays correct across ABI wording changes.
    core_py = (
        REPO_ROOT / "bindings" / "python" / "src" / "motorbridge" / "core.py"
    ).read_text(encoding="utf-8")
    assert "has no CAN-FD transport" in core_py


def test_cli_run_mode_enum_mapping_is_cyberbeast_compatible() -> None:
    """ABI mode codes: 1=MIT, 2=POSITION, 3=VELOCITY, 4=TORQUE."""
    assert cli_common._mode_to_enum("mit") == Mode.MIT
    assert cli_common._mode_to_enum("pos-vel") == Mode.POS_VEL
    assert cli_common._mode_to_enum("vel") == Mode.VEL
    assert cli_common._mode_to_enum("force-pos") == Mode.FORCE_POS


def test_cli_write_param_reports_readback_value(monkeypatch, capsys) -> None:
    class FakeMotor:
        def __init__(self) -> None:
            self.writes: list[tuple[int, float]] = []
            self.stored = 0
            self.values = {0x0304: 18.0}

        def cyberbeast_write_param_f32(self, param_id: int, value: float) -> None:
            self.writes.append((param_id, value))

        def cyberbeast_get_param_f32(self, param_id: int, timeout_ms: int) -> float:
            return self.values[param_id]

        def store_parameters(self) -> None:
            self.stored += 1

        def close(self) -> None:
            pass

    motor = FakeMotor()

    class FakeCtrl:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    monkeypatch.setattr(cli_run, "_open_controller", lambda args, vendor: FakeCtrl())
    monkeypatch.setattr(cli_run, "_add_motor", lambda ctrl, vendor, mid, fid, model: motor)

    args = argparse.Namespace(
        vendor="cyberbeast",
        model="4340",
        feedback_id="0x11",
        transport="auto",
        channel="can0",
        mode="write-param",
        set_motor_id="",
        set_feedback_id="",
        motor_id="0x01",
        param_id="0x0304",
        param_value="18.0",
        param_type="",
        timeout_ms=500,
        store=1,
    )
    cli_run._run_command(args)

    assert motor.writes == [(0x0304, 18.0)]
    assert motor.stored == 1
    out = capsys.readouterr().out
    assert "vendor=cyberbeast param_id=0x0304 type=f32 requested=18.0 value=18.0 verified=1" in out


def test_cli_scan_cyberbeast_uses_node_id_for_feedback(monkeypatch) -> None:
    events: list[tuple] = []

    class FakeMotor:
        def __init__(self, mid: int) -> None:
            self.mid = mid

        def enable(self) -> None:
            events.append(("enable", self.mid))

        def disable(self) -> None:
            events.append(("disable", self.mid))

        def request_feedback(self) -> None:
            events.append(("feedback", self.mid))

        def get_state(self):
            if self.mid != 2:
                raise RuntimeError("no feedback")
            return argparse.Namespace(
                pos=0.5, vel=0.1, torq=1.25, status_code=0, t_mos=41.0, t_rotor=39.0
            )

        def close(self) -> None:
            events.append(("close", self.mid))

    class FakeController:
        def __init__(self, channel: str) -> None:
            events.append(("open", channel))

        def add_cyberbeast_motor(self, mid: int, fid: int, model: str) -> FakeMotor:
            events.append(("add", mid, fid, model))
            return FakeMotor(mid)

        def poll_feedback_once(self) -> None:
            pass

        def close_bus(self) -> None:
            events.append(("close_bus",))

        def close(self) -> None:
            events.append(("close_ctrl",))

    monkeypatch.setattr(cli_common, "Controller", FakeController)

    args = argparse.Namespace(
        channel="can0",
        model="odrive-default",
        transport="auto",
        timeout_ms=10,
    )
    found = cli_scan._scan_cyberbeast(args, 1, 2)

    assert [mid for mid, _ in found] == [2]
    assert "vendor=cyberbeast node_id=0x02" in found[0][1]
    # feedback_id mirrors the node id: CyberBeast feedback is addressed by motor_id.
    assert [e for e in events if e[0] == "add"] == [
        ("add", 1, 1, "odrive-default"),
        ("add", 2, 2, "odrive-default"),
    ]
    # The probe leaves the axis stopped again.
    assert ("disable", 2) in events
    assert events[-1] == ("close_ctrl",)
