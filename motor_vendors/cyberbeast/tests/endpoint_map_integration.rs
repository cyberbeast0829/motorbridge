//! End-to-end checks of the endpoint-map workflow, without a CAN device.
//!
//! These cover the guarantee that a motor handle never works with half a table:
//! connecting loads the device's own descriptor and caches it, parameter reads use
//! the type the device declared, and a node that does not describe itself makes the
//! connect step fail instead of handing out a handle whose table is missing.

use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::test_support::MockBus;
use motor_vendor_cyberbeast::{
    can_id_parts, make_can_id, pack_mit_command, protocol, CyberBeastController, CyberBeastMotor,
    EndpointKind, MitCommandParams, MitRanges, MsgType, ParamValue, ValueType,
};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

const MOTOR_ID: u16 = 0x01;

/// Descriptor fixture in the device's own shape: nested `members`, a function under
/// `outputs`, and a `uint64` that needs a segmented read. It is longer than one
/// 50-frame cycle, so the transfer needs continuation requests.
const DESCRIPTOR: &str = r#"[{"name":"axis0","id":14,"type":"object","members":[
  {"name":"current_state","id":142,"type":"uint8","access":"r"},
  {"name":"requested_state","id":143,"type":"uint8","access":"rw"},
  {"name":"motor","id":20,"type":"object","members":[{"name":"config","id":21,"type":"object","members":[
    {"name":"gear_ratio","id":242,"type":"float","access":"rw"},
    {"name":"serial_number","id":5,"type":"uint64","access":"r"}]}]},
  {"name":"controller","id":22,"type":"object","members":[{"name":"config","id":23,"type":"object","members":[
    {"name":"mit_max_pos","id":335,"type":"float","access":"rw"},
    {"name":"mit_max_vel","id":336,"type":"float","access":"rw"},
    {"name":"mit_max_torque","id":337,"type":"float","access":"rw"},
    {"name":"mit_max_kp","id":338,"type":"float","access":"rw"},
    {"name":"mit_max_kd","id":339,"type":"float","access":"rw"}]}]}],
  "outputs":[{"name":"save_configuration","id":63,"type":"function"}]}]"#;

/// Endpoint ids of the MIT maxima in [`DESCRIPTOR`], with the values the tested
/// hardware declares (`mit_max_pos` 12.5, vel 65, torque 50, kp 500, kd 5).
const MIT_RANGE_PARAMS: [(u16, f32); 5] = [
    (335, 12.5),
    (336, 65.0),
    (337, 50.0),
    (338, 500.0),
    (339, 5.0),
];

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

/// Faults the emulated node injects, to exercise the host's recovery paths.
///
/// Everything here reproduces something seen or documented for the real protocol:
/// frames do get lost on a slcan adapter, the first request of a session is the one
/// most likely to be lost, and an interrupted transfer leaves frames in flight that
/// the next request sees first.
#[derive(Default)]
struct Faults {
    /// `ChunkOffset`s whose data frame is not sent, once each: a frame lost on the bus.
    drop_once: Mutex<Vec<u16>>,
    /// `ChunkOffset`s whose data frame is never sent: a chunk the device cannot deliver.
    drop_always: Mutex<Vec<u16>>,
    /// `JSON_DESC_READ` requests to ignore: the request never reached the node.
    drop_requests: Mutex<u32>,
    /// Frames pushed before the answer to the next request: residue of a transfer that
    /// was interrupted (a killed or timed-out master).
    residue: Mutex<Vec<CanFrame>>,
    /// Master frames the node never sees, from the start of the session: `drophead`.
    ///
    /// This is the documented slcan behaviour and JointSDK's injected fault of the same
    /// name: the adapter is still resetting its own input buffer while the host is
    /// already writing, so the first one or two frames are simply gone -- and Lawicel
    /// slcan reports nothing per frame, so the host has no signal at all.
    drophead: Mutex<u32>,
    /// Which PARAM_READ answer is not delivered (1 = the first), if any: the same loss in
    /// the other direction, which does produce a request the node saw.
    drop_param_reply: Mutex<Option<u32>>,
    /// PARAM_READ answers produced so far (the index `drop_param_reply` counts).
    param_replies: Mutex<u32>,
}

impl Faults {
    fn drop_chunk_once(self: &Arc<Self>, offset: u16) {
        self.drop_once.lock().expect("fault lock").push(offset);
    }

    fn drop_chunk_always(self: &Arc<Self>, offset: u16) {
        self.drop_always.lock().expect("fault lock").push(offset);
    }

    fn drop_requests(self: &Arc<Self>, count: u32) {
        *self.drop_requests.lock().expect("fault lock") = count;
    }

    fn drop_head(self: &Arc<Self>, count: u32) {
        *self.drophead.lock().expect("fault lock") = count;
    }

    fn drop_param_reply(self: &Arc<Self>, index: u32) {
        *self.drop_param_reply.lock().expect("fault lock") = Some(index);
    }

    fn push_residue(self: &Arc<Self>, frames: impl IntoIterator<Item = CanFrame>) {
        self.residue.lock().expect("fault lock").extend(frames);
    }
}

/// Emulates the node: it answers **requests** instead of dumping frames, like the
/// firmware does. `JSON_DESC_READ` is answered with metadata plus up to
/// [`CHUNKS_PER_CYCLE`] data frames from the requested offset, and `PARAM_READ` with
/// at most 4 value bytes per cycle (`More` set while the value continues).
struct MockDevice {
    stop: Arc<AtomicBool>,
    handle: Option<thread::JoinHandle<()>>,
}

impl MockDevice {
    fn start_with_faults(
        mock: Arc<MockBus>,
        descriptor: &'static str,
        version_crc: u16,
        params: Arc<Mutex<HashMap<u16, Vec<u8>>>>,
        faults: Arc<Faults>,
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
                    // A frame the adapter dropped in flight: the node never saw it, so
                    // nothing is answered and the host sees silence only.
                    let lost_in_flight = {
                        let mut head = faults.drophead.lock().expect("fault lock");
                        if *head > 0 {
                            *head -= 1;
                            true
                        } else {
                            false
                        }
                    };
                    if lost_in_flight {
                        continue;
                    }
                    match can_id_parts(request.arbitration_id).msg_type {
                        t if t == MsgType::JsonDescRead as u8 => {
                            // One lock scope per decision: `std::sync::Mutex` is not
                            // reentrant, and a guard held across a second `lock()` would
                            // deadlock this thread (and hang the test).
                            let request_lost = {
                                let mut remaining =
                                    faults.drop_requests.lock().expect("fault lock");
                                if *remaining > 0 {
                                    *remaining -= 1;
                                    true
                                } else {
                                    false
                                }
                            };
                            if request_lost {
                                continue;
                            }
                            // Residue of an interrupted transfer is delivered *before*
                            // the answer to this request, exactly as an adapter that
                            // still holds frames from the previous run would.
                            for stale in faults.residue.lock().expect("fault lock").drain(..) {
                                mock.push_rx(stale);
                            }
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
                                let offset = (start + index * 6) as u16;
                                let lost_once = {
                                    let mut drop = faults.drop_once.lock().expect("fault lock");
                                    let position = drop.iter().position(|it| *it == offset);
                                    position.map(|index| drop.remove(index)).is_some()
                                };
                                if lost_once
                                    || faults
                                        .drop_always
                                        .lock()
                                        .expect("fault lock")
                                        .contains(&offset)
                                {
                                    continue;
                                }
                                mock.push_rx(chunk_frame(offset, chunk));
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
                                // The answer may be produced and then lost on the way back.
                                let reply_index = {
                                    let mut seen = faults.param_replies.lock().expect("fault lock");
                                    *seen += 1;
                                    *seen
                                };
                                let answer_lost = {
                                    let mut plan =
                                        faults.drop_param_reply.lock().expect("fault lock");
                                    if *plan == Some(reply_index) {
                                        *plan = None;
                                        true
                                    } else {
                                        false
                                    }
                                };
                                if answer_lost {
                                    continue;
                                }
                                let start = offset.min(value.len());
                                let end = (start + 4).min(value.len());
                                mock.push_rx(param_read_frame(
                                    endpoint_id,
                                    &value[start..end],
                                    end < value.len(),
                                ));
                            }
                        }
                        t if t == MsgType::ParamWrite as u8 => {
                            // A write is acknowledged with the same endpoint id and DataLen 0.
                            let mut data = [0u8; 8];
                            data[1] = request.data[1];
                            data[2] = request.data[2];
                            mock.push_rx(frame(MsgType::ParamWrite, data, 8));
                        }
                        // `QUERY_POS_VEL` answers with two big-endian f32 values (turns and
                        // turns/s). The real node answers this, and it is what the session
                        // warm-up probes with.
                        t if t == MsgType::QueryPosVel as u8 => {
                            let mut data = [0u8; 8];
                            data[0..4].copy_from_slice(&0.0f32.to_be_bytes());
                            data[4..8].copy_from_slice(&0.0f32.to_be_bytes());
                            mock.push_rx(frame(MsgType::QueryPosVel, data, 8));
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

    /// Stop answering while the thread stays alive (the node is there but mute).
    fn stop_serving(&self) {
        self.stop.store(true, Ordering::SeqCst);
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

/// `QUERY_POS_VEL` frames the host sent (the warm-up probe is one of these).
fn posvel_requests(mock: &MockBus) -> usize {
    mock.sent
        .lock()
        .expect("sent lock")
        .iter()
        .filter(|frame| can_id_parts(frame.arbitration_id).msg_type == MsgType::QueryPosVel as u8)
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
        Self::start_with_faults(Arc::new(Faults::default()))
    }

    /// A node with a fault plan: lost frames, lost requests, leftover frames.
    fn start_with_faults(faults: Arc<Faults>) -> Self {
        let rig = Self::start_with_missing_mit_ranges_and_faults(faults);
        for (endpoint_id, value) in MIT_RANGE_PARAMS {
            rig.set_param(endpoint_id, &value.to_le_bytes());
        }
        // The device also declares its gear ratio (7.75 on the tested node).
        rig.set_param(242, &7.75f32.to_le_bytes());
        rig
    }

    /// A node that does not declare its MIT maxima: the connection still works and the
    /// protocol defaults stay in force, with the reason recorded on the handle.
    fn start_with_missing_mit_ranges() -> Self {
        Self::start_with_missing_mit_ranges_and_faults(Arc::new(Faults::default()))
    }

    fn start_with_missing_mit_ranges_and_faults(faults: Arc<Faults>) -> Self {
        let (mock, bus) = shared_bus();
        let params: Arc<Mutex<HashMap<u16, Vec<u8>>>> = Arc::new(Mutex::new(HashMap::new()));
        let device = MockDevice::start_with_faults(
            Arc::clone(&mock),
            DESCRIPTOR,
            0x3F82,
            Arc::clone(&params),
            faults,
        );
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

    /// A handle that sends nothing while it is being created.
    ///
    /// For the warm-up checks this matters: with the descriptor in front of it, the
    /// session's first frame is already a descriptor request, which is why the normal
    /// connect path never exposes the frame-loss window at all.
    fn connect_probe(&self) -> Arc<CyberBeastMotor> {
        self.ctrl
            .add_motor_probe(MOTOR_ID, MOTOR_ID, "odrive-default")
            .expect("a probe handle")
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
    assert_eq!(map.len(), 15);
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
    // The descriptor is longer than one stream cycle, so one continuation request per
    // extra cycle is expected -- and no re-fetch beyond that.
    assert_eq!(
        descriptor_requests(&rig.mock),
        DESCRIPTOR.len().div_ceil(6 * CHUNKS_PER_CYCLE),
        "one request per {CHUNKS_PER_CYCLE}-chunk cycle for {} bytes",
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
fn connecting_uses_the_device_declared_mit_ranges() {
    let rig = TestRig::start();
    let motor = rig.connect();

    assert_eq!(
        motor.mit_ranges(),
        MitRanges {
            pos: 12.5,
            vel: 65.0,
            kp: 500.0,
            kd: 5.0,
            torque: 50.0,
        }
    );
    assert!(motor.mit_ranges_from_device());
    assert_eq!(motor.mit_ranges_note(), None);
    assert_eq!(motor.mit_torque_limit(), 50.0);

    // The frame must be scaled by the device's maxima. Encoding with the protocol
    // defaults (12.566 / 30 / 100 / 18) would clamp velocity and torque and shrink Kd
    // by 20x, which is exactly the bug this guards against.
    let params = MitCommandParams {
        pos: 6.25,
        vel: 32.5,
        kp: 250.0,
        kd: 2.5,
        torque: 25.0,
    };
    motor
        .send_mit_command(params.pos, params.vel, params.kp, params.kd, params.torque)
        .expect("MIT command");
    let sent = rig.mock.sent.lock().expect("sent lock").clone();
    let frame = sent.last().expect("one frame was sent");
    assert_eq!(
        frame.data,
        pack_mit_command(&params, 12.5, 65.0, 500.0, 5.0, 50.0)
    );
    assert_ne!(
        frame.data,
        pack_mit_command(&params, 12.566, 30.0, 500.0, 100.0, 18.0)
    );

    rig.stop();
}

#[test]
fn missing_mit_declarations_keep_the_defaults_and_say_so() {
    let rig = TestRig::start_with_missing_mit_ranges();
    let motor = rig.connect();

    assert_eq!(motor.mit_ranges(), MitRanges::default());
    assert!(!motor.mit_ranges_from_device());
    // Nothing declared the gear ratio, so nothing is scaled and the value says so.
    assert_eq!(motor.gear_ratio(), 1.0);
    assert!(!motor.gear_ratio_from_device());
    let note = motor.mit_ranges_note().expect("the reason is recorded");
    assert!(note.contains("mit_max_pos"), "{note}");
    // Only the MIT scaling fell back: the endpoint table itself still loaded.
    assert_eq!(motor.endpoint_map().expect("map").len(), 15);

    rig.stop();
}

#[test]
fn typed_writes_use_the_declared_width() {
    let rig = TestRig::start();
    let motor = rig.connect();

    // `axis0.requested_state` is a uint8: the frame must carry one value byte, not four.
    motor
        .set_param_value(143, ParamValue::U8(8))
        .expect("uint8 write");
    let sent = rig.mock.sent.lock().expect("sent lock").clone();
    let write = sent
        .iter()
        .find(|frame| can_id_parts(frame.arbitration_id).msg_type == MsgType::ParamWrite as u8)
        .expect("a PARAM_WRITE frame");
    assert_eq!(write.data, protocol::encode_param_write_bytes(143, &[8]));
    assert_eq!(write.data[3], 1, "one value byte for a uint8 endpoint");

    // Float writes must go out little-endian too, as protocol v2.5 section 4.7 requires for
    // SDO parameter values. This bit us on hardware: a big-endian write of 100 was stored as
    // 1677721600 and silenced the node's heartbeat.
    motor.set_param_f32(242, 0.0824).expect("float write");
    let sent = rig.mock.sent.lock().expect("sent lock").clone();
    let write = sent
        .iter()
        .rev()
        .find(|frame| can_id_parts(frame.arbitration_id).msg_type == MsgType::ParamWrite as u8)
        .expect("a PARAM_WRITE frame");
    assert_eq!(
        write.data,
        protocol::encode_param_write_bytes(242, &0.0824f32.to_le_bytes())
    );
    assert_ne!(
        write.data,
        protocol::encode_param_write_bytes(242, &0.0824f32.to_be_bytes()),
        "the value must not be written big-endian"
    );

    // A four-byte float must not be pushed into a one-byte endpoint ...
    let err = motor
        .set_param_f32(143, 8.0)
        .expect_err("float write to a uint8 endpoint")
        .to_string();
    assert!(err.contains("declared \"uint8\", not \"float\""), "{err}");

    // ... and a read-only endpoint is refused outright.
    let err = motor
        .set_param_value(142, ParamValue::U8(1))
        .expect_err("write to a read-only endpoint")
        .to_string();
    assert!(err.contains("access=\"r\""), "{err}");

    rig.stop();
}

#[test]
fn mit_responses_are_converted_to_motor_side_units() {
    let rig = TestRig::start();
    let motor = rig.connect();

    assert_eq!(motor.gear_ratio(), 7.75);
    assert!(motor.gear_ratio_from_device());

    // A MIT response reporting 0.1 output rad: pos is 16-bit over +/-12.5 rad, vel and
    // current are 12-bit mid-scale (no motion), mode 4 = MIT.
    let pos_code = (((0.1f32 + 12.5) / 25.0) * 65535.0) as u16;
    let data = [
        (pos_code >> 8) as u8,
        pos_code as u8,
        0x80,
        0x00,
        0x80,
        0x04,
        78,
        80,
    ];
    let decoded = protocol::unpack_mit_response(&data, 12.5, 65.0, 80.0);
    assert!(
        (decoded.pos - 0.1).abs() < 0.001,
        "the fixture must encode 0.1 output rad, got {}",
        decoded.pos
    );

    motor
        .process_feedback_frame(CanFrame {
            arbitration_id: make_can_id(6, MsgType::MitControl as u8, 0x01, MOTOR_ID as u8, 0),
            data,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        })
        .expect("MIT response");

    let state = motor.latest_state().expect("state");
    let expected = decoded.pos * 7.75;
    assert!(
        (state.pos - expected).abs() < 0.002,
        "state.pos {} must be the motor-side value {expected}",
        state.pos
    );
    // Without the conversion the cached position would be 7.75x too small.
    assert!((state.pos - decoded.pos).abs() > 0.5);

    rig.stop();
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

#[test]
fn a_frame_lost_inside_the_descriptor_is_re_fetched() {
    // One data frame never makes it onto the bus: the chunk at offset 6, right after the
    // metadata frame -- the position of the first frame of a burst, which is the one a
    // slcan adapter drops most often. The transfer must notice the hole, ask for the
    // first missing byte instead of carrying on past it, and still deliver the exact
    // descriptor text.
    let faults = Arc::new(Faults::default());
    faults.drop_chunk_once(6);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));

    let motor = rig.connect();

    let map = motor.endpoint_map().expect("the map survives a lost frame");
    assert_eq!(
        map.json_text(),
        DESCRIPTOR,
        "every byte must come from the device, with no padding where the frame was lost"
    );
    assert_eq!(map.total_len(), DESCRIPTOR.len() as u32);
    assert_eq!(map.version_crc(), 0x3F82);
    assert_eq!(map.len(), 15);

    // The bytes that were streamed past the hole are re-sent, which costs exactly the
    // one additional request: asking for the first missing byte restarts the device
    // there (protocol 4.8).
    let baseline = DESCRIPTOR.len().div_ceil(6 * CHUNKS_PER_CYCLE);
    assert_eq!(descriptor_requests(&rig.mock), baseline + 1);

    rig.stop();
}

#[test]
fn a_chunk_the_device_never_sends_fails_with_the_byte_it_stopped_at() {
    // A chunk that is lost forever (or a device that cannot deliver it) must not turn
    // into a descriptor with padding in it, and must not spin for 64 requests either:
    // the error has to say where the transfer stopped.
    let faults = Arc::new(Faults::default());
    faults.drop_chunk_always(6);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));

    let started = std::time::Instant::now();
    let message = match rig.ctrl.add_motor(MOTOR_ID, MOTOR_ID, "odrive-default") {
        Ok(_) => panic!("an incomplete descriptor must not produce a handle"),
        Err(err) => err.to_string(),
    };
    let elapsed = started.elapsed();

    assert!(
        message.contains("loading the endpoint descriptor from node 0x01"),
        "{message}"
    );
    assert!(message.contains("stalled at byte 6 of"), "{message}");
    assert!(message.contains("brought nothing new"), "{message}");
    assert!(
        !message.contains("not valid JSON"),
        "the transfer must fail before anyone tries to parse it: {message}"
    );
    assert!(elapsed < Duration::from_secs(5), "took {elapsed:?}");

    rig.stop();
}

#[test]
fn a_lost_first_request_is_repeated_until_the_transfer_starts() {
    // The first frame a session sends can be lost while the adapter configures itself,
    // and adding a motor sends JSON_DESC_READ first. Repeating it is free (the device
    // restarts the transfer), so connecting must not depend on that frame surviving.
    let faults = Arc::new(Faults::default());
    faults.drop_requests(1);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));

    let motor = rig.connect();

    let map = motor.endpoint_map().expect("the retry loads the map");
    assert_eq!(map.json_text(), DESCRIPTOR);
    let baseline = DESCRIPTOR.len().div_ceil(6 * CHUNKS_PER_CYCLE);
    assert_eq!(descriptor_requests(&rig.mock), baseline + 1);

    rig.stop();
}

#[test]
fn residue_from_an_interrupted_transfer_restarts_the_transfer() {
    // A master that dies mid-transfer leaves frames in the adapter, and they are
    // delivered *before* the answer to the next request. Here the residue even carries a
    // metadata frame of another descriptor version, so the host cannot tell which
    // transfer the bytes belong to. It must throw the assembly away, ask again, and end
    // up with the device's real table -- never with a mixture.
    let faults = Arc::new(Faults::default());
    faults.push_residue([
        // Metadata of an earlier version: a different TotalLength and crc.
        {
            let mut data = [0u8; 8];
            data[2..6].copy_from_slice(&4567u32.to_le_bytes());
            data[6..8].copy_from_slice(&0x1234u16.to_le_bytes());
            frame(MsgType::JsonDescData, data, 8)
        },
        // A data frame of that earlier transfer, at offset 0 and at a later offset.
        chunk_frame(0, b"ZZZZZZ"),
        chunk_frame(744, b"QQQQQQ"),
    ]);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));

    let motor = rig.connect();

    let map = motor.endpoint_map().expect("a restart clears the residue");
    assert_eq!(
        map.json_text(),
        DESCRIPTOR,
        "no stale byte may survive into the table"
    );
    assert_eq!(map.total_len(), DESCRIPTOR.len() as u32);
    assert_eq!(map.version_crc(), 0x3F82);

    // One extra request: the restarted transfer.
    let baseline = DESCRIPTOR.len().div_ceil(6 * CHUNKS_PER_CYCLE);
    assert_eq!(descriptor_requests(&rig.mock), baseline + 1);

    rig.stop();
}

// ---------------------------------------------------------------------------
// Session warm-up and idempotent re-send
// ---------------------------------------------------------------------------
//
// The fault these cover is a dropped frame, not a dropped *answer*: the host's frame (or
// the device's) never reaches the other side, and with Lawicel slcan nobody is told. The
// only thing a host can do about it is ask again -- which is exactly what JointSDK's
// `jsdk_context_warmup` does (`hop` on the slcan HAL: the adapter is still resetting its
// own input buffer while the host is already writing, so the first one or two frames of a
// session are simply gone).
//
// Note what the *normal* connect path does about it: loading the endpoint descriptor means
// the session's first frames are descriptor requests, and that transfer already re-requests
// whatever did not arrive. The warm-up matters for the paths that skip it, and for the
// transports that open a serial link per session (`dm-device`, `dm-serial`).

#[test]
fn warm_up_spends_the_dropped_frames_on_probes_instead_of_the_first_command() {
    for dropped in 1..=3u32 {
        let faults = Arc::new(Faults::default());
        faults.drop_head(dropped);
        let rig = TestRig::start_with_faults(Arc::clone(&faults));
        // `add_motor_probe` sends nothing, so the warm-up probe really is the session's
        // first frame -- the position the descriptor load otherwise hides.
        let motor = rig.connect_probe();

        let retries = motor
            .warm_up(Duration::from_millis(500))
            .expect("the probe is re-sent until one lands");
        assert_eq!(retries, dropped, "one re-send per dropped frame");

        // The point of the exercise: the first command that *matters* now works, and the
        // value it returns is the device's own.
        rig.set_param(242, &7.75f32.to_le_bytes());
        let value = motor
            .read_param_f32(242, Duration::from_millis(300))
            .expect("a parameter read after the warm-up");
        assert_eq!(value, 7.75);
        assert_eq!(motor.tx_retries(), dropped, "only the warm-up re-sent");
        rig.stop();
    }
}

#[test]
fn warm_up_is_a_no_op_once_it_has_succeeded() {
    let rig = TestRig::start();
    let motor = rig.connect_probe();

    assert_eq!(
        motor
            .warm_up(Duration::from_millis(500))
            .expect("a healthy link"),
        0
    );
    let probes = posvel_requests(&rig.mock);
    assert_eq!(probes, 1, "one probe is enough on a link that answers");

    assert_eq!(
        motor
            .warm_up(Duration::from_millis(500))
            .expect("already warm"),
        0
    );
    assert_eq!(
        posvel_requests(&rig.mock),
        probes,
        "a warmed session probes nothing again"
    );
    rig.stop();
}

#[test]
fn a_lost_param_answer_is_asked_again_without_corrupting_the_value() {
    // The request reached the node; the answer did not come back. One lost frame must not
    // turn a read into a spurious timeout -- the request is idempotent, so it is sent again.
    let faults = Arc::new(Faults::default());
    faults.drop_param_reply(1);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));
    rig.set_param(242, &7.75f32.to_le_bytes());
    let motor = rig.connect();

    let value = motor
        .read_param_f32(242, Duration::from_millis(600))
        .expect("the read is re-sent after the lost answer");
    assert_eq!(value, 7.75);
    assert!(motor.tx_retries() >= 1, "and the retry is reported");
    rig.stop();
}

#[test]
fn a_lost_continuation_answer_is_asked_again_at_the_same_offset() {
    // `serial_number` is a uint64, so the read takes two cycles. The second answer is lost:
    // nothing arrives for the offset that is outstanding, so the host asks for that same
    // offset again and the assembled value is still exactly the device's.
    let faults = Arc::new(Faults::default());
    faults.drop_param_reply(2);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));
    rig.set_param(5, &0x0123_4567_89AB_CDEFu64.to_le_bytes());
    let motor = rig.connect();

    let bytes = motor
        .read_param_raw(5, Duration::from_millis(600))
        .expect("the outstanding offset is asked for again");
    assert_eq!(bytes, 0x0123_4567_89AB_CDEFu64.to_le_bytes());
    assert!(motor.tx_retries() >= 1, "and the retry is reported");
    rig.stop();
}

#[test]
fn a_segmented_read_that_never_gets_an_answer_times_out_without_inventing_bytes() {
    // The counterpart of the test above: when the node stops answering altogether, the read
    // must report the timeout -- never a half assembled value that looks like a real one.
    let faults = Arc::new(Faults::default());
    faults.drop_param_reply(1);
    let rig = TestRig::start_with_faults(Arc::clone(&faults));
    rig.set_param(5, &0x0123_4567_89AB_CDEFu64.to_le_bytes());
    let motor = rig.connect();
    // Stop the node before the read, so nothing can answer.
    rig.device.stop_serving();

    let err = motor
        .read_param_raw(5, Duration::from_millis(300))
        .expect_err("a silent node must not produce a value")
        .to_string();
    assert!(err.contains("timeout waiting for param read"), "{err}");
    assert!(err.contains("0x0005"), "{err}");
    rig.stop();
}
