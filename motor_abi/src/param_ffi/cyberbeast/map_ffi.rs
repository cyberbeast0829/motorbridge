use crate::vendor_params::cyberbeast;
use crate::{set_last_error, set_last_string, MotorHandle};
use core::ffi::c_char;
use std::ptr;

/// Read the device's JSON endpoint descriptor (protocol 4.8).
///
/// Returns a pointer to the JSON text, or null on failure (the reason is available
/// through `motor_last_error_message`). The pointer stays valid until the next ABI
/// call on the same thread, exactly like `motor_last_error_message`.
///
/// `out_total_len` and `out_version_crc` may be null; otherwise they receive the
/// `TotalLength` and `VersionCRC` from the descriptor metadata, which the caller can
/// use to detect that the device's endpoint map changed.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_cyberbeast_endpoint_map(
    motor: *mut MotorHandle,
    timeout_ms: u32,
    out_total_len: *mut u32,
    out_version_crc: *mut u32,
) -> *const c_char {
    if motor.is_null() {
        set_last_error("motor is null");
        return ptr::null();
    }
    let motor_ref = unsafe { &*motor };
    let inner = match motor_ref.inner.lock() {
        Ok(inner) => inner,
        Err(_) => {
            set_last_error("motor handle lock poisoned");
            return ptr::null();
        }
    };
    match cyberbeast::read_endpoint_map(&inner, timeout_ms) {
        Ok((json, total_len, version_crc)) => {
            if !out_total_len.is_null() {
                unsafe { *out_total_len = total_len };
            }
            if !out_version_crc.is_null() {
                unsafe { *out_version_crc = version_crc };
            }
            set_last_string(&json)
        }
        Err(e) => {
            set_last_error(e);
            ptr::null()
        }
    }
}
