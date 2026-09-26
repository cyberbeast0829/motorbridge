//! End-to-end checks of the endpoint-map workflow, without a CAN device.
//!
//! These cover the guarantee that a motor handle never works with half a table:
//! connecting loads the device's own descriptor and caches it, parameter reads use
//! the type the device declared, and a node that does not describe itself makes the
//! connect step fail instead of handing out a handle whose table is missing.

use motor_core::bus::{CanBus, CanFrame};
use motor_core::test_support::MockBus;
use motor_vendor_cyberbeast::{
    can_id_parts, make_can_id, CyberBeastController, CyberBeastMotor, EndpointKind, MsgType,
    ParamValue, ValueType,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

const MOTOR_ID: u16 = 0x01;

/// Descriptor fixture in the device's own shape: nested `members`, a function under
/// `outputs`, and a `uint64` that needs a segmented read. Its length is above the
/// 64-byte minimum a metadata frame may declare.
const DESCRIPTOR: &str = r#"[{"name":"axis0","id":14,"type":"object","members":[
  {"name":"current_state","id":142,"type":"uint8","access":"r"},
  {"name":"motor","id":20,"type":"object","members":[{"name":"config","id":21,"type":"object","members":[
    {"name":"gear_ratio","id":242,"type":"float","access":"rw"},
    {"name":"serial_number","id":5,"type":"uint64","access":"r"}]}]}],
  "outputs":[{"name":"save_configuration","id":63,"type":"function"}]}]"#;

fn frame(msg_type: MsgType, data: [u8; 8], dlc: u8) -> CanFrame {
    CanFrame {
        arbitration_id: make_can_id(6, msg_type as u8, 0x01, MOTOR_ID as u8, 0),
        data,
        dlc,
        is_extended: true,
        is_rx: true,
    }
}

/// `[0x00, 0x00] | TotalLength (u32 LE) | VersionCRC (u16 LE)`
fn meta_frame(total_len: u32, version_crc: u16) -> CanFrame {
    let mut data = [0u8; 8];
    data[2..6].copy_from_slice(&total_len.to_le_bytes());
    data[6..8].copy_from_slice(&version_crc.to_le_bytes());
    frame(MsgType::JsonDescData, data, 8)
}

/// `ChunkOffset (u16 LE) | JSON bytes`
fn chunk_frame(offset: u16, chunk: &[u8]) -> CanFrame {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&offset.to_le_bytes());
    data[2..2 + chunk.len()].copy_from_slice(chunk);
    frame(MsgType::JsonDescData, data, (2 + chunk.len()) as u8)
}

/// `Flags | EndpointID (u16 BE) | DataLen | value (little-endian)`
fn param_read_frame(endpoint_id: u16, value: &[u8], more: bool) -> CanFrame {
    let mut data = [0u8; 8];
    data[0] = if more { 0x80 } else { 0x00 };
    data[1..3].copy_from_slice(&endpoint_id.to_be_bytes());
    data[3] = value.len() as u8;
    data[4..4 + value.len()].copy_from_slice(value);
    frame(MsgType::ParamRead, data, 8)
}

/// Data frames the firmware streams per continuation cycle (50 frames of 6 JSON
/// bytes). Reproducing the limit exercises the continuation path, and the firmware
/// also repeats the metadata frame on every cycle, which the host must ignore.
const CHUNKS_PER_CYCLE: usize = 50;

/// Emulates the node: it answers **requests** instead of dumping frames, like the
/// firmware does. `JSON_DESC_READ` is answered with metadata plus up to
/// [`CHUNKS_PER_CYCLE`] data frames from the requested offset, and `PARAM_READ` with
/// at most 4 value bytes per cycle (`More` set while the value continues).
struct MockDevice {
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl MockDevice {
    fn start(
        mock: Arc<MockBus>,
        descriptor: &'static str,
        version_crc: u16,
        params: Arc<Mutex<HashMap<u16, Vec<u8>>>>,
    ) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = thread::spawn(move || {
            let mut served = 0usize;
            while !flag.load(Ordering::SeqCst) {
                let requests: Vec<CanFrame> = {
                    let sent = mock.sent.lock().expect("sent lock");
                    sent[served.min(sent.len())..].to_vec()
                };
                served += requests.len();
                for request in requests {
                    match can_id_parts(request.arbitration_id).msg_type {
                        t if t == MsgType::JsonDescRead as u8 => {
                            // Protocol 4.8: the descriptor offset is little-endian
                            // (unlike PARAM_READ's big-endian offset below).
                            let offset = u32::from_le_bytes([
                                request.data[0],
                                request.data[1],
                                request.data[2],
                                request.data[3],
                            ]);
                            let bytes = descriptor.as_bytes();
                            let start = (offset as usize).min(bytes.len());
                            mock.push_rx(meta_frame(bytes.len() as u32, version_crc));
                            for (index, chunk) in
                                bytes[start..].chunks(6).take(CHUNKS_PER_CYCLE).enumerate()
                            {
                                mock.push_rx(chunk_frame((start + index * 6) as u16, chunk));
                            }
                        }
                        t if t == MsgType::ParamRead as u8 => {
                            let endpoint_id =
                                ((request.data[1] as u16) << 8) | request.data[2] as u16;
                            // Protocol 4.7: the request offset is big-endian.
                            let offset = u32::from_be_bytes([
                                request.data[4],
                                request.data[5],
                                request.data[6],
                                request.data[7],
                            ]) as usize;
                            let value = params
                                .lock()
                                .expect("params lock")
                                .get(&endpoint_id)
                                .cloned();
                            if let Some(value) = value {
                                let start = offset.min(value.len());
                                let end = (start + 4).min(value.len());
                                mock.push_rx(param_read_frame(
                                    endpoint_id,
                                    &value[start..end],
                                    end < value.len(),
                                ));
                            }
                        }
                        _ => {}
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }

    fn stop(mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// Polls the controller in the background, the way the SDK's feedback thread does.
struct Poller {
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl Poller {
    fn start(ctrl: Arc<CyberBeastController>) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let handle = thread::spawn(move || {
            while !flag.load(Ordering::SeqCst) {
                let _ = ctrl.poll_feedback_once();
                thread::sleep(Duration::from_millis(1));
            }
        });
        Self {
            stop,
            handle: Some(handle),
        }
    }

    fn stop(mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

fn shared_bus() -> (Arc<MockBus>, Arc<dyn CanBus>) {
    let mock: Arc<MockBus> = Arc::new(MockBus::new());
    let bus: Arc<dyn CanBus> = Arc::clone(&mock) as Arc<dyn CanBus>;
    (mock, bus)
}

fn descriptor_requests(mock: &MockBus) -> usize {
    mock.sent
        .lock()
        .expect("sent lock")
        .iter()
        .filter(|frame| can_id_parts(frame.arbitration_id).msg_type == MsgType::JsonDescRead as u8)
        .count()
}

/// A node, a bus and a controller wired together: the emulated device answers the
/// controller's requests while the poller feeds responses back into the motors.
struct TestRig {
    mock: Arc<MockBus>,
    ctrl: Arc<CyberBeastController>,
    params: Arc<Mutex<HashMap<u16, Vec<u8>>>>,
    device: MockDevice,
    poller: Poller,
}

impl TestRig {
    fn start() -> Self {
        let (mock, bus) = shared_bus();
        let params: Arc<Mutex<HashMap<u16, Vec<u8>>>> = Arc::new(Mutex::new(HashMap::new()));
        let device = MockDevice::start(Arc::clone(&mock), DESCRIPTOR, 0x3F82, Arc::clone(&params));
        let ctrl = Arc::new(CyberBeastController::new(bus));
        let poller = Poller::start(Arc::clone(&ctrl));
        Self {
            mock,
            ctrl,
            params,
            device,
            poller,
        }
    }

    /// Value the emulated node answers for `endpoint_id`.
    fn set_param(&self, endpoint_id: u16, value: &[u8]) {
        self.params
            .lock()
            .expect("params lock")
            .insert(endpoint_id, value.to_vec());
    }

    fn connect(&self) -> Arc<CyberBeastMotor> {
        self.ctrl
            .add_motor(MOTOR_ID, MOTOR_ID, "odrive-default")
            .expect("connecting loads the endpoint map")
    }

    fn stop(self) {
        self.poller.stop();
        self.device.stop();
    }
}

#[test]
fn add_motor_loads_and_caches_the_endpoint_map() {
    let rig = TestRig::start();

    let motor = rig.connect();

    let map = motor
        .endpoint_map()
        .expect("the map is cached on the handle");
    assert_eq!(map.len(), 7);
    assert_eq!(map.total_len(), DESCRIPTOR.len() as u32);
    assert_eq!(map.version_crc(), 0x3F82);
    assert_eq!(map.get(242).unwrap().path, "axis0.motor.config.gear_ratio");
    assert_eq!(map.get(242).unwrap().value_type(), Some(ValueType::F32));
    assert_eq!(map.get(5).unwrap().value_type(), Some(ValueType::U64));
    assert_eq!(map.get(63).unwrap().kind, EndpointKind::Function);
    assert_eq!(map.resolve("gear_ratio").unwrap().endpoint_id, 242);

    // The table belongs to the handle, so a later lookup needs no transfer at all.
    let registered = rig
        .ctrl
        .get_motor(MOTOR_ID)
        .expect("the node stays registered");
    assert_eq!(registered.endpoint_map().unwrap().version_crc(), 0x3F82);
    // The descriptor is longer than one stream cycle, so exactly one continuation
    // request is expected -- and no re-fetch beyond that.
    assert_eq!(
        descriptor_requests(&rig.mock),
        2,
        "one request plus one continuation for {} bytes",
        DESCRIPTOR.len()
    );

    rig.stop();
}

#[test]
fn read_param_value_uses_the_device_declared_type() {
    let rig = TestRig::start();
    rig.set_param(242, &[0x00, 0x00, 0xF8, 0x40]); // 7.75, the hardware value
    rig.set_param(5, &[0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01]);
    rig.set_param(142, &[0x01]);
    let motor = rig.connect();

    let ratio = motor
        .read_param_value(242, Duration::from_millis(200))
        .expect("typed float read");
    assert_eq!(ratio.value, ParamValue::F32(7.75));
    assert_eq!(ratio.declared, "float");
    assert_eq!(ratio.access.label(), "rw");
    assert!(ratio.access.is_writable());
    assert_eq!(ratio.path.as_deref(), Some("axis0.motor.config.gear_ratio"));
    assert_eq!(ratio.raw, vec![0x00, 0x00, 0xF8, 0x40]);

    // uint64 needs two cycles: the first answer carries the More flag
    let serial = motor
        .read_param_value(5, Duration::from_millis(200))
        .expect("segmented read");
    assert_eq!(serial.value, ParamValue::U64(0x0123_4567_89AB_CDEF));
    assert_eq!(serial.raw.len(), 8);
    assert_eq!(serial.declared, "uint64");

    // uint8 stays a uint8 instead of being widened into a float
    let state = motor
        .read_param_value(142, Duration::from_millis(200))
        .expect("uint8 read");
    assert_eq!(state.value, ParamValue::U8(1));
    assert_eq!(state.access.label(), "r");
    assert!(!state.access.is_writable());

    rig.stop();
}

#[test]
fn repeated_reads_return_fresh_values() {
    let rig = TestRig::start();
    rig.set_param(242, &[0x00, 0x00, 0xF8, 0x40]); // 7.75
    let motor = rig.connect();

    let first = motor
        .read_param_value(242, Duration::from_millis(200))
        .expect("first read");
    assert_eq!(first.value, ParamValue::F32(7.75));

    rig.set_param(242, &[0x00, 0x00, 0x08, 0x41]); // 8.5
    let second = motor
        .read_param_value(242, Duration::from_millis(200))
        .expect("second read");
    assert_eq!(
        second.value,
        ParamValue::F32(8.5),
        "a second read must ask the device, not replay the cached answer"
    );

    // The request-then-wait pair the C ABI uses (and therefore Python) must not
    // hand out the previous read's value either.
    rig.set_param(242, &[0x00, 0x00, 0x18, 0x41]); // 9.5
    motor.send_param_read(242).expect("request");
    let polled = motor
        .get_param_f32(242, Duration::from_millis(200))
        .expect("polled read");
    assert_eq!(polled, 9.5, "the polled value must come from this request");

    rig.stop();
}

#[test]
fn read_param_value_reports_unknown_and_non_value_endpoints() {
    let rig = TestRig::start();
    let motor = rig.connect();

    let unknown = motor
        .read_param_value(999, Duration::from_millis(50))
        .unwrap_err()
        .to_string();
    assert!(
        unknown.contains("is not in the device's endpoint map"),
        "{unknown}"
    );

    let function = motor
        .read_param_value(63, Duration::from_millis(50))
        .unwrap_err()
        .to_string();
    assert!(function.contains("declared \"function\""), "{function}");
    assert!(function.contains("save_configuration"), "{function}");

    rig.stop();
}

#[test]
fn add_motor_fails_when_the_node_does_not_describe_itself() {
    // No device: the bus stays silent.
    let (mock, bus) = shared_bus();
    let ctrl = CyberBeastController::new(bus);

    let started = std::time::Instant::now();
    let message = match ctrl.add_motor(MOTOR_ID, MOTOR_ID, "odrive-default") {
        Ok(_) => panic!("a silent node must not produce a handle"),
        Err(err) => err.to_string(),
    };
    let elapsed = started.elapsed();

    assert!(
        message.contains("loading the endpoint descriptor from node 0x01"),
        "{message}"
    );
    assert!(
        descriptor_requests(&mock) >= 1,
        "the descriptor must have been requested"
    );
    // A wrong node id must fail quickly, not after 64 quiet windows.
    assert!(elapsed < Duration::from_secs(3), "took {elapsed:?}");
}

#[test]
fn add_motor_probe_stays_off_the_bus() {
    let rig = TestRig::start();

    let motor = rig
        .ctrl
        .add_motor_probe(MOTOR_ID, MOTOR_ID, "odrive-default")
        .expect("probe");

    assert!(motor.endpoint_map().is_none());
    assert_eq!(descriptor_requests(&rig.mock), 0);

    rig.stop();
}

#[test]
fn reading_without_a_loaded_map_is_reported_honestly() {
    let (_, bus) = shared_bus();
    let motor = CyberBeastMotor::new(MOTOR_ID, MOTOR_ID, "odrive-default", bus).expect("motor");

    let message = motor
        .read_param_value(242, Duration::from_millis(50))
        .unwrap_err()
        .to_string();

    assert!(
        message.contains("endpoint map of this motor is not loaded"),
        "{message}"
    );
}
