use crate::motor::CyberBeastMotor;
use motor_core::bus::{open_can_bus, CanBus};
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::Duration;

/// Quiet window used while loading the endpoint map (protocol 4.8).
///
/// The device streams a burst of descriptor frames and then pauses, so this bounds
/// the gap between bursts, not the whole transfer.
pub const DEFAULT_ENDPOINT_MAP_TIMEOUT: Duration = Duration::from_millis(500);

pub struct CyberBeastController {
    controller: VendorController<CyberBeastMotor>,
}

/// Prefix an error with the node it happened on, keeping the error kind.
fn node_context(err: MotorError, motor_id: u16) -> MotorError {
    let prefix = format!("loading the endpoint descriptor from node 0x{motor_id:02X}: ");
    match err {
        MotorError::InvalidArgument(m) => MotorError::InvalidArgument(prefix + &m),
        MotorError::Io(m) => MotorError::Io(prefix + &m),
        MotorError::Timeout(m) => MotorError::Timeout(prefix + &m),
        MotorError::Protocol(m) => MotorError::Protocol(prefix + &m),
        MotorError::Unsupported(m) => MotorError::Unsupported(prefix + &m),
    }
}

impl CyberBeastController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
        }
    }

    pub fn new_socketcan(channel: &str) -> Result<Self> {
        Ok(Self::new(open_can_bus(channel)?))
    }

    /// Add a motor **and** load its endpoint map (protocol 4.8).
    ///
    /// The descriptor is read once here and cached inside the handle, so every later
    /// `read_param` / `write_param` uses the device's own table without another
    /// transfer. This fails when the node does not answer: a handle whose parameter
    /// table is silently missing would make later reads guess at value types.
    ///
    /// Use [`Self::add_motor_probe`] when probing ids that are expected to be silent
    /// (bus scanning).
    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<CyberBeastMotor>> {
        let motor = self.add_motor_probe(motor_id, feedback_id, model)?;
        motor
            .ensure_endpoint_map(DEFAULT_ENDPOINT_MAP_TIMEOUT)
            .map_err(|err| node_context(err, motor_id))?;
        Ok(motor)
    }

    /// Add a motor without sending anything on the bus.
    ///
    /// For scanning loops, where most candidate ids are silent and a descriptor
    /// transfer per candidate would be wasted (and slow).
    pub fn add_motor_probe(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<CyberBeastMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            CyberBeastMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn add_motor_with_master(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
        master_id: u8,
    ) -> Result<Arc<CyberBeastMotor>> {
        let motor = self.controller.add_motor_with(motor_id, |bus| {
            Ok(CyberBeastMotor::new(motor_id, feedback_id, model, bus)?.with_master_id(master_id))
        })?;
        motor
            .ensure_endpoint_map(DEFAULT_ENDPOINT_MAP_TIMEOUT)
            .map_err(|err| node_context(err, motor_id))?;
        Ok(motor)
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<CyberBeastMotor>> {
        self.controller.get_motor(motor_id)
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.controller.poll_feedback_once()
    }

    pub fn enable_all(&self) -> Result<()> {
        self.controller.enable_all()
    }

    pub fn disable_all(&self) -> Result<()> {
        self.controller.disable_all()
    }

    pub fn shutdown(&self) -> Result<()> {
        self.controller.shutdown()
    }

    pub fn close_bus(&self) -> Result<()> {
        self.controller.close_bus()
    }
}
