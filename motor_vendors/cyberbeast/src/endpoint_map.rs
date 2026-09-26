//! Parsed JSON endpoint descriptor (protocol 4.8).
//!
//! The device reports its own endpoint map: each endpoint is an object with a
//! `name`, an `id`, a `type` and (for values) an `access` flag, nested through
//! `members` (objects), `inputs` / `outputs` (functions) or plain arrays. The tested
//! firmware (0.6.9) reports 554 endpoints: 521 values (223 `float`, 124 `uint32`,
//! 81 `bool`, 47 `uint8`, 21 `int32`, 20 `uint16`, 4 `uint64`, 1 `int64`), 26
//! functions, 6 endpoint references and one JSON blob.
//!
//! Reading that map once, when the motor handle is created, is what lets
//! `read-param` / `write-param` work for **every** endpoint the device has instead
//! of a hand-maintained subset, and it is why the type of a value never has to be
//! guessed: [`EndpointMap`] is the lookup table the rest of the SDK uses, and
//! [`decode_value`] turns the bytes of a `PARAM_READ` response into a typed value
//! using the device's own declaration.

use motor_core::error::{MotorError, Result};
use serde_json::Value;
use std::collections::HashMap;

/// Value width and interpretation of a data endpoint, as declared by the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueType {
    F32,
    F64,
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    Bool,
}

impl ValueType {
    /// Map a descriptor `type` string; `None` for the non-value kinds.
    pub fn from_raw(raw: &str) -> Option<Self> {
        Some(match raw {
            "float" | "float32" => Self::F32,
            "float64" | "double" => Self::F64,
            "uint8" => Self::U8,
            "uint16" => Self::U16,
            "uint32" => Self::U32,
            "uint64" => Self::U64,
            "int8" => Self::I8,
            "int16" => Self::I16,
            "int32" => Self::I32,
            "int64" => Self::I64,
            "bool" => Self::Bool,
            _ => return None,
        })
    }

    /// Canonical name, used in error messages.
    pub fn label(self) -> &'static str {
        match self {
            Self::F32 => "float",
            Self::F64 => "float64",
            Self::U8 => "uint8",
            Self::U16 => "uint16",
            Self::U32 => "uint32",
            Self::U64 => "uint64",
            Self::I8 => "int8",
            Self::I16 => "int16",
            Self::I32 => "int32",
            Self::I64 => "int64",
            Self::Bool => "bool",
        }
    }

    /// Bytes this type occupies in a `PARAM_READ` response.
    ///
    /// `Bool` is declared as 1 byte; [`decode_value`] also accepts a 4-byte
    /// encoding, because firmware variants differ here.
    pub fn byte_width(self) -> usize {
        match self {
            Self::U8 | Self::I8 | Self::Bool => 1,
            Self::U16 | Self::I16 => 2,
            Self::F32 | Self::U32 | Self::I32 => 4,
            Self::F64 | Self::U64 | Self::I64 => 8,
        }
    }
}

/// What an endpoint object describes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EndpointKind {
    /// A readable (and possibly writable) value.
    Value(ValueType),
    /// A firmware function; not readable through `PARAM_READ`.
    Function,
    /// A reference to another endpoint; not readable through `PARAM_READ`.
    EndpointRef,
    /// A nested JSON object or blob.
    Json,
    /// A kind this SDK does not know; `raw_type` keeps the device's text.
    Other,
}

/// Declared access of an endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    ReadOnly,
    ReadWrite,
    /// No `access` field (functions) or an unrecognised value.
    Unknown,
}

impl Access {
    fn from_raw(raw: Option<&str>) -> Self {
        match raw {
            Some("r") => Self::ReadOnly,
            Some("rw") => Self::ReadWrite,
            _ => Self::Unknown,
        }
    }

    /// Text as it appears in the descriptor.
    pub fn label(self) -> &'static str {
        match self {
            Self::ReadOnly => "r",
            Self::ReadWrite => "rw",
            Self::Unknown => "-",
        }
    }

    /// Whether the device declares this endpoint as writable.
    pub fn is_writable(self) -> bool {
        self == Self::ReadWrite
    }
}

/// One endpoint of the device descriptor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EndpointEntry {
    /// Dotted path built while walking the descriptor, e.g.
    /// `axis0.motor.config.gear_ratio`.
    pub path: String,
    /// 16-bit SDO endpoint id used by `PARAM_READ` / `PARAM_WRITE`.
    pub endpoint_id: u16,
    /// Type text exactly as reported by the device (`float`, `uint32`, `function`, ...).
    pub raw_type: String,
    /// Parsed kind, derived from `raw_type`.
    pub kind: EndpointKind,
    /// Declared access.
    pub access: Access,
}

impl EndpointEntry {
    /// The declared value type, when this endpoint is a value.
    pub fn value_type(&self) -> Option<ValueType> {
        match self.kind {
            EndpointKind::Value(value_type) => Some(value_type),
            _ => None,
        }
    }
}

/// A decoded `PARAM_READ` value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    F32(f32),
    F64(f64),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    Bool(bool),
}

impl ParamValue {
    /// The value as `f64` when it is numeric (any integer type included).
    pub fn as_f64(self) -> Option<f64> {
        Some(match self {
            Self::F32(v) => f64::from(v),
            Self::F64(v) => v,
            Self::U8(v) => f64::from(v),
            Self::U16(v) => f64::from(v),
            Self::U32(v) => f64::from(v),
            Self::U64(v) => v as f64,
            Self::I8(v) => f64::from(v),
            Self::I16(v) => f64::from(v),
            Self::I32(v) => f64::from(v),
            Self::I64(v) => v as f64,
            Self::Bool(_) => return None,
        })
    }

    /// The value as a float when the device declared a float type.
    ///
    /// Use [`Self::as_f64`] to also accept integer endpoints.
    pub fn as_f32(self) -> Option<f32> {
        match self {
            Self::F32(v) => Some(v),
            _ => None,
        }
    }
}

impl std::fmt::Display for ParamValue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::F32(v) => write!(f, "{v}"),
            Self::F64(v) => write!(f, "{v}"),
            Self::U8(v) => write!(f, "{v}"),
            Self::U16(v) => write!(f, "{v}"),
            Self::U32(v) => write!(f, "{v}"),
            Self::U64(v) => write!(f, "{v}"),
            Self::I8(v) => write!(f, "{v}"),
            Self::I16(v) => write!(f, "{v}"),
            Self::I32(v) => write!(f, "{v}"),
            Self::I64(v) => write!(f, "{v}"),
            Self::Bool(v) => write!(f, "{v}"),
        }
    }
}

/// One typed `PARAM_READ` result together with what the device says the endpoint is.
#[derive(Debug, Clone)]
pub struct ParamReadout {
    pub endpoint_id: u16,
    /// Dotted path from the descriptor, when the map is loaded.
    pub path: Option<String>,
    /// Declared type text, e.g. `float`.
    pub declared: String,
    pub access: Access,
    pub value: ParamValue,
    /// The assembled little-endian bytes as received (possibly segmented).
    pub raw: Vec<u8>,
}

/// The device's endpoint map: every endpoint plus id/name lookups.
#[derive(Debug, Clone, Default)]
pub struct EndpointMap {
    total_len: u32,
    version_crc: u16,
    json: String,
    entries: Vec<EndpointEntry>,
    by_id: HashMap<u16, usize>,
    by_name: HashMap<String, usize>,
}

impl EndpointMap {
    /// Parse descriptor JSON text as returned by `JSON_DESC_READ`.
    ///
    /// The walk keeps every object that carries an `id` (nested `members`,
    /// `inputs`, `outputs` and arrays included) and builds dotted paths. Endpoints
    /// are returned in document order; a repeated id keeps its first occurrence.
    pub fn parse(json: &str) -> Result<Self> {
        let root: Value = serde_json::from_str(json).map_err(|err| {
            MotorError::Protocol(format!("endpoint descriptor is not valid JSON: {err}"))
        })?;
        let mut map = Self {
            json: json.to_string(),
            ..Self::default()
        };
        collect_endpoints(&root, "", &mut map.entries);
        for (index, entry) in map.entries.iter().enumerate() {
            map.by_id.entry(entry.endpoint_id).or_insert(index);
            map.by_name
                .entry(entry.path.to_lowercase())
                .or_insert(index);
        }
        Ok(map)
    }

    /// Attach the metadata frame values (`TotalLength`, `VersionCRC`).
    pub fn with_metadata(mut self, total_len: u32, version_crc: u16) -> Self {
        self.total_len = total_len;
        self.version_crc = version_crc;
        self
    }

    /// Descriptor length reported by the metadata frame.
    pub fn total_len(&self) -> u32 {
        self.total_len
    }

    /// Descriptor version CRC reported by the metadata frame.
    ///
    /// Compare it across connections to detect that the device's map changed.
    pub fn version_crc(&self) -> u16 {
        self.version_crc
    }

    /// The descriptor text, byte for byte as received.
    pub fn json_text(&self) -> &str {
        &self.json
    }

    /// Every endpoint, in document order.
    pub fn entries(&self) -> &[EndpointEntry] {
        &self.entries
    }

    /// Number of endpoints (values, functions and references).
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Number of endpoints that carry a value (readable parameters).
    pub fn value_count(&self) -> usize {
        self.entries
            .iter()
            .filter(|entry| entry.kind != EndpointKind::Function)
            .filter(|entry| !matches!(entry.kind, EndpointKind::EndpointRef | EndpointKind::Json))
            .count()
    }

    /// Look an endpoint up by its 16-bit id.
    pub fn get(&self, endpoint_id: u16) -> Option<&EndpointEntry> {
        self.by_id
            .get(&endpoint_id)
            .and_then(|index| self.entries.get(*index))
    }

    /// Case-insensitive substring search over the dotted paths, in document order.
    pub fn find(&self, needle: &str) -> Vec<&EndpointEntry> {
        let needle = needle.to_lowercase();
        self.entries
            .iter()
            .filter(|entry| entry.path.to_lowercase().contains(&needle))
            .collect()
    }

    /// Resolve an endpoint written by a human.
    ///
    /// Accepted forms, in order: an id (`0x00F7` or `247`), an exact dotted path,
    /// an exact last path segment (`node_id`), or a unique substring (`gear`).
    pub fn resolve(&self, spec: &str) -> Result<&EndpointEntry> {
        let spec = spec.trim();
        if spec.is_empty() {
            return Err(MotorError::InvalidArgument(
                "endpoint id/name is empty".to_string(),
            ));
        }
        if let SpecId::Id(endpoint_id) = parse_endpoint_id(spec) {
            return self.get(endpoint_id).ok_or_else(|| {
                MotorError::InvalidArgument(format!(
                    "endpoint 0x{endpoint_id:04X} ({endpoint_id}) is not in the device's endpoint map ({} entries, VersionCRC=0x{:04X})",
                    self.len(),
                    self.version_crc
                ))
            });
        }
        if let SpecId::OutOfRange = parse_endpoint_id(spec) {
            return Err(MotorError::InvalidArgument(format!(
                "\"{spec}\" does not fit in a 16-bit endpoint id"
            )));
        }

        let needle = spec.to_lowercase();
        if let Some(index) = self.by_name.get(&needle) {
            return Ok(&self.entries[*index]);
        }

        let last_segment: Vec<&EndpointEntry> = self
            .entries
            .iter()
            .filter(|entry| {
                entry
                    .path
                    .to_lowercase()
                    .rsplit('.')
                    .next()
                    .is_some_and(|name| name == needle)
            })
            .collect();
        match last_segment.len() {
            0 => {}
            1 => return Ok(last_segment[0]),
            _ => return Err(ambiguous_endpoint(spec, &last_segment)),
        }

        let found = self.find(&needle);
        match found.as_slice() {
            [] => Err(MotorError::InvalidArgument(format!(
                "no endpoint matches \"{spec}\" in the device's endpoint map ({} entries); \
                 use `find-endpoint --name <substring>` to search it",
                self.len()
            ))),
            [only] => Ok(only),
            many => Err(ambiguous_endpoint(spec, many)),
        }
    }

    /// One-line summary for logs and CLI output.
    pub fn summary(&self) -> String {
        format!(
            "{} endpoints ({} values), {} bytes, VersionCRC=0x{:04X}",
            self.len(),
            self.value_count(),
            self.total_len,
            self.version_crc
        )
    }
}

fn ambiguous_endpoint(spec: &str, candidates: &[&EndpointEntry]) -> MotorError {
    let listed = candidates
        .iter()
        .take(8)
        .map(|entry| format!("0x{:04X} {}", entry.endpoint_id, entry.path))
        .collect::<Vec<_>>()
        .join(", ");
    MotorError::InvalidArgument(format!(
        "\"{spec}\" is ambiguous: {} endpoints match ({listed}{})",
        candidates.len(),
        if candidates.len() > 8 { ", ..." } else { "" }
    ))
}

/// How a human-written endpoint specification parses into an id.
enum SpecId {
    /// A number that fits in 16 bits.
    Id(u16),
    /// A number too large to be an endpoint id: report it instead of clamping.
    OutOfRange,
    /// Not a number, so it must be a name.
    NotAnId,
}

/// Parse `0x00F7` / `247`; a decimal number is always an id, because endpoint names
/// never consist of digits alone.
fn parse_endpoint_id(spec: &str) -> SpecId {
    let (digits, radix) = match spec.strip_prefix("0x").or_else(|| spec.strip_prefix("0X")) {
        Some(hex) => (hex, 16),
        None => (spec, 10),
    };
    if digits.is_empty() || !digits.chars().all(|c| c.is_digit(radix)) {
        return SpecId::NotAnId;
    }
    match u16::from_str_radix(digits, radix) {
        Ok(endpoint_id) => SpecId::Id(endpoint_id),
        Err(_) => SpecId::OutOfRange,
    }
}

/// Walk the descriptor, collecting every object that carries an `id`.
fn collect_endpoints(node: &Value, prefix: &str, out: &mut Vec<EndpointEntry>) {
    match node {
        Value::Array(items) => {
            for item in items {
                collect_endpoints(item, prefix, out);
            }
        }
        Value::Object(object) => {
            let name = object.get("name").and_then(Value::as_str).unwrap_or("");
            let path = match (prefix.is_empty(), name.is_empty()) {
                (true, _) | (_, true) => format!("{prefix}{name}"),
                (false, false) => format!("{prefix}.{name}"),
            };
            if let Some(endpoint_id) = object.get("id").and_then(Value::as_u64) {
                if let Ok(endpoint_id) = u16::try_from(endpoint_id) {
                    let raw_type = object
                        .get("type")
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string();
                    let kind = match raw_type.as_str() {
                        "function" => EndpointKind::Function,
                        "endpoint_ref" => EndpointKind::EndpointRef,
                        "json" => EndpointKind::Json,
                        other => match ValueType::from_raw(other) {
                            Some(value_type) => EndpointKind::Value(value_type),
                            None => EndpointKind::Other,
                        },
                    };
                    out.push(EndpointEntry {
                        path: path.clone(),
                        endpoint_id,
                        raw_type,
                        kind,
                        access: Access::from_raw(object.get("access").and_then(Value::as_str)),
                    });
                }
            }
            for (key, child) in object {
                if matches!(key.as_str(), "name" | "id" | "type" | "access") {
                    continue;
                }
                if child.is_object() || child.is_array() {
                    collect_endpoints(child, &path, out);
                }
            }
        }
        _ => {}
    }
}

/// Decode little-endian `PARAM_READ` bytes using the device's declared type.
pub fn decode_value(bytes: &[u8], value_type: ValueType) -> Result<ParamValue> {
    fn fixed<const N: usize>(bytes: &[u8], value_type: ValueType) -> Result<[u8; N]> {
        bytes.try_into().map_err(|_| {
            MotorError::Protocol(format!(
                "endpoint is declared {} ({} byte(s)) but the device returned {} byte(s)",
                value_type.label(),
                N,
                bytes.len()
            ))
        })
    }

    Ok(match value_type {
        ValueType::F32 => ParamValue::F32(f32::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::F64 => ParamValue::F64(f64::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::U8 => ParamValue::U8(fixed::<1>(bytes, value_type)?[0]),
        ValueType::U16 => ParamValue::U16(u16::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::U32 => ParamValue::U32(u32::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::U64 => ParamValue::U64(u64::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::I8 => ParamValue::I8(fixed::<1>(bytes, value_type)?[0] as i8),
        ValueType::I16 => ParamValue::I16(i16::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::I32 => ParamValue::I32(i32::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::I64 => ParamValue::I64(i64::from_le_bytes(fixed(bytes, value_type)?)),
        ValueType::Bool => match bytes.len() {
            1 => ParamValue::Bool(bytes[0] != 0),
            4 => ParamValue::Bool(u32::from_le_bytes(fixed(bytes, value_type)?) != 0),
            other => {
                return Err(MotorError::Protocol(format!(
                    "endpoint is declared bool (1 byte) but the device returned {other} byte(s)"
                )))
            }
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mirrors the real descriptor shape: nested objects use `members`, functions
    /// use `inputs` / `outputs`, and only values carry an `access` field.
    const DESCRIPTOR: &str = r#"[
        {
            "name": "odrv",
            "id": 1000,
            "type": "object",
            "members": [
                {"name": "error", "id": 1, "type": "uint8", "access": "rw"},
                {"name": "vbus_voltage", "id": 2, "type": "float", "access": "r"}
            ],
            "outputs": [
                {"name": "save_configuration", "id": 63, "type": "function"}
            ]
        },
        {
            "name": "axis0",
            "id": 14,
            "type": "object",
            "members": [
                {"name": "current_state", "id": 142, "type": "uint8", "access": "r"},
                {"name": "motor", "id": 20, "type": "object", "members": [
                    {"name": "config", "id": 21, "type": "object", "members": [
                        {"name": "gear_ratio", "id": 242, "type": "float", "access": "rw"},
                        {"name": "gear_teeth", "id": 408, "type": "uint16", "access": "rw"}
                    ]}
                ]},
                {"name": "ref", "id": 400, "type": "endpoint_ref", "access": "r"}
            ]
        }
    ]"#;

    fn map() -> EndpointMap {
        EndpointMap::parse(DESCRIPTOR)
            .expect("fixture is valid json")
            .with_metadata(DESCRIPTOR.len() as u32, 0x3F82)
    }

    #[test]
    fn parse_builds_dotted_paths_and_lookups() {
        let map = map();

        assert_eq!(map.total_len(), DESCRIPTOR.len() as u32);
        assert_eq!(map.version_crc(), 0x3F82);
        assert_eq!(map.json_text(), DESCRIPTOR);

        let paths: Vec<&str> = map.entries().iter().map(|e| e.path.as_str()).collect();
        assert_eq!(
            paths,
            vec![
                "odrv",
                "odrv.error",
                "odrv.vbus_voltage",
                "odrv.save_configuration",
                "axis0",
                "axis0.current_state",
                "axis0.motor",
                "axis0.motor.config",
                "axis0.motor.config.gear_ratio",
                "axis0.motor.config.gear_teeth",
                "axis0.ref",
            ]
        );

        let gear = map.get(242).expect("id 242 is in the map");
        assert_eq!(gear.path, "axis0.motor.config.gear_ratio");
        assert_eq!(gear.raw_type, "float");
        assert_eq!(gear.value_type(), Some(ValueType::F32));
        assert_eq!(gear.access, Access::ReadWrite);

        let save = map.get(63).expect("id 63 is in the map");
        assert_eq!(save.kind, EndpointKind::Function);
        assert_eq!(save.access, Access::Unknown);
        assert_eq!(save.value_type(), None);
        assert_eq!(map.get(0xFFF).map(|e| e.path.as_str()), None);
    }

    #[test]
    fn value_count_ignores_functions_and_references() {
        let map = map();
        // 9 values out of 11 entries (one function, one endpoint_ref).
        assert_eq!(map.len(), 11);
        assert_eq!(map.value_count(), 9);
    }

    #[test]
    fn find_matches_case_insensitive_substrings_in_document_order() {
        let map = map();

        let gears = map.find("GEAR");
        let names: Vec<&str> = gears.iter().map(|e| e.path.as_str()).collect();
        assert_eq!(
            names,
            vec![
                "axis0.motor.config.gear_ratio",
                "axis0.motor.config.gear_teeth"
            ]
        );
        assert!(map.find("nothing_like_this").is_empty());
    }

    #[test]
    fn resolve_reports_out_of_range_ids() {
        let map = map();

        let too_big = map.resolve("70000").unwrap_err().to_string();
        assert!(
            too_big.contains("does not fit in a 16-bit endpoint id"),
            "{too_big}"
        );
        assert!(!too_big.contains("65535"), "{too_big}");
    }

    #[test]
    fn resolve_accepts_ids_paths_segments_and_substrings() {
        let map = map();

        assert_eq!(
            map.resolve("0x00F2").unwrap().path,
            "axis0.motor.config.gear_ratio"
        );
        assert_eq!(
            map.resolve("242").unwrap().path,
            "axis0.motor.config.gear_ratio"
        );
        assert_eq!(
            map.resolve("axis0.motor.config.gear_ratio")
                .unwrap()
                .endpoint_id,
            242
        );
        assert_eq!(map.resolve("current_state").unwrap().endpoint_id, 142);
        assert_eq!(map.resolve("  vbus_voltage  ").unwrap().endpoint_id, 2);
    }

    #[test]
    fn resolve_reports_unknown_and_ambiguous_endpoints() {
        let map = map();

        let unknown = map.resolve("does_not_exist").unwrap_err().to_string();
        assert!(unknown.contains("no endpoint matches"), "{unknown}");

        let out_of_map = map.resolve("0x0151").unwrap_err().to_string();
        assert!(
            out_of_map.contains("not in the device's endpoint map"),
            "{out_of_map}"
        );
        assert!(out_of_map.contains("11 entries"), "{out_of_map}");

        // "gear_" matches two paths in the same way, so neither is an exact segment.
        let ambiguous = map.resolve("gear_").unwrap_err().to_string();
        assert!(ambiguous.contains("ambiguous"), "{ambiguous}");
        assert!(ambiguous.contains("0x00F2"), "{ambiguous}");
    }

    #[test]
    fn parse_rejects_invalid_json() {
        let err = EndpointMap::parse("{not json").unwrap_err().to_string();
        assert!(err.contains("not valid JSON"), "{err}");
    }

    #[test]
    fn decode_value_handles_every_declared_width() {
        assert_eq!(
            decode_value(&[0x00, 0x00, 0xF8, 0x40], ValueType::F32).unwrap(),
            ParamValue::F32(7.75)
        );
        assert_eq!(
            decode_value(&[0x2A], ValueType::U8).unwrap(),
            ParamValue::U8(42)
        );
        assert_eq!(
            decode_value(&[0xFF], ValueType::I8).unwrap(),
            ParamValue::I8(-1)
        );
        assert_eq!(
            decode_value(&[0x34, 0x12], ValueType::U16).unwrap(),
            ParamValue::U16(0x1234)
        );
        assert_eq!(
            decode_value(&[0x00, 0x40, 0x00, 0x00], ValueType::U32).unwrap(),
            ParamValue::U32(16384)
        );
        assert_eq!(
            decode_value(&[0xFF, 0xFF, 0xFF, 0xFF], ValueType::I32).unwrap(),
            ParamValue::I32(-1)
        );
        assert_eq!(
            decode_value(
                &[0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01],
                ValueType::U64
            )
            .unwrap(),
            ParamValue::U64(0x0123_4567_89AB_CDEF)
        );
        assert_eq!(
            decode_value(&[0xFF; 8], ValueType::I64).unwrap(),
            ParamValue::I64(-1)
        );
        assert_eq!(
            decode_value(&[1], ValueType::Bool).unwrap(),
            ParamValue::Bool(true)
        );
        assert_eq!(
            decode_value(&[0], ValueType::Bool).unwrap(),
            ParamValue::Bool(false)
        );
        assert_eq!(
            decode_value(&[1, 0, 0, 0], ValueType::Bool).unwrap(),
            ParamValue::Bool(true)
        );
    }

    #[test]
    fn decode_value_reports_a_width_mismatch_as_a_protocol_error() {
        let err = decode_value(&[0x00, 0x40], ValueType::F32)
            .unwrap_err()
            .to_string();
        assert!(err.contains("declared float (4 byte(s))"), "{err}");
        assert!(err.contains("returned 2 byte(s)"), "{err}");

        let bool_err = decode_value(&[1, 2, 3], ValueType::Bool)
            .unwrap_err()
            .to_string();
        assert!(bool_err.contains("declared bool"), "{bool_err}");
    }

    #[test]
    fn param_value_display_and_numeric_accessors() {
        assert_eq!(ParamValue::F32(7.75).to_string(), "7.75");
        assert_eq!(ParamValue::U32(16384).to_string(), "16384");
        assert_eq!(ParamValue::Bool(false).to_string(), "false");
        assert_eq!(ParamValue::U32(40).as_f64(), Some(40.0));
        assert_eq!(ParamValue::U32(40).as_f32(), None);
        assert_eq!(ParamValue::Bool(true).as_f64(), None);
    }

    #[test]
    fn repeated_ids_keep_the_first_occurrence() {
        // The device nests objects whose ids can repeat (`odrv` / `odrv.error`);
        // the first document-order occurrence wins, and both paths stay searchable.
        let map = EndpointMap::parse(
            r#"[{"name":"odrv","id":1,"type":"object","members":[
                 {"name":"error","id":1,"type":"uint8","access":"rw"}]}]"#,
        )
        .unwrap();

        assert_eq!(map.len(), 2);
        assert_eq!(map.get(1).unwrap().path, "odrv");
        assert_eq!(map.resolve("odrv.error").unwrap().endpoint_id, 1);
    }
}
