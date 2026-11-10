//! OBD-II basics shared by the app: PID table and decoding (SAE J1979), unit conversion,
//! trouble-code formatting (SAE J2012).

/// Bytes a service 09 02 (VIN) reply must carry at least.
pub const VIN_REPLY_MIN_BYTES: usize = 20;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Units {
    Metric,
    Imperial,
    Us,
}

#[derive(Clone)]
pub struct Pid {
    pub pid: u8,
    pub name: &'static str,
    pub formula: &'static str,
    pub unit: &'static str,
    pub min: f64,
    pub max: f64,
    pub bytes: usize,
    pub src: crate::obd::Src,
}

pub fn pids() -> Vec<Pid> {
    vec![
        Pid { pid: 0x0C, name: "Engine speed", formula: "(256A+B)/4", unit: "rpm", min: 600.0, max: 6500.0, bytes: 2, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x0D, name: "Vehicle speed", formula: "A", unit: "km/h", min: 0.0, max: 180.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x05, name: "Coolant temperature", formula: "A-40", unit: "°C", min: 70.0, max: 105.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x04, name: "Calculated engine load", formula: "A×100/255", unit: "%", min: 15.0, max: 85.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x11, name: "Throttle position", formula: "A×100/255", unit: "%", min: 0.0, max: 100.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x0F, name: "Intake air temperature", formula: "A-40", unit: "°C", min: -10.0, max: 60.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x10, name: "MAF air flow rate", formula: "(256A+B)/100", unit: "g/s", min: 2.0, max: 60.0, bytes: 2, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x06, name: "Short term fuel trim B1", formula: "(A-128)×100/128", unit: "%", min: -10.0, max: 10.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x0E, name: "Timing advance", formula: "A/2-64", unit: "°", min: 0.0, max: 40.0, bytes: 1, src: crate::obd::Src::Mode01 },
        Pid { pid: 0x42, name: "Control module voltage", formula: "(256A+B)/1000", unit: "V", min: 12.0, max: 14.8, bytes: 2, src: crate::obd::Src::Mode01 },
    ]
}

/// Decode raw bytes with the formula of each PID (SAE J1979).
pub fn decode(pid: u8, a: u8, b: u8) -> f64 {
    let (a, b) = (a as f64, b as f64);
    match pid {
        0x0C => (256.0 * a + b) / 4.0,
        0x0D => a,
        0x05 | 0x0F => a - 40.0,
        0x04 | 0x11 => a * 100.0 / 255.0,
        0x10 => (256.0 * a + b) / 100.0,
        0x06 => (a - 128.0) * 100.0 / 128.0,
        0x0E => a / 2.0 - 64.0,
        0x42 => (256.0 * a + b) / 1000.0,
        _ => 0.0,
    }
}

#[cfg(test)]
fn encode(pid: u8, v: f64) -> (u8, u8) {
    let two = |x: f64| {
        let n = x.round().clamp(0.0, 65535.0) as u32;
        ((n >> 8) as u8, (n & 0xff) as u8)
    };
    let one = |x: f64| (x.round().clamp(0.0, 255.0) as u8, 0u8);
    match pid {
        0x0C => two(v * 4.0),
        0x0D => one(v),
        0x05 | 0x0F => one(v + 40.0),
        0x04 | 0x11 => one(v * 255.0 / 100.0),
        0x10 => two(v * 100.0),
        0x06 => one(v * 128.0 / 100.0 + 128.0),
        0x0E => one((v + 64.0) * 2.0),
        0x42 => two(v * 1000.0),
        _ => (0, 0),
    }
}

/// Convert a metric value for display.
/// Metric → imperial table (unit names, factors,
/// 16 units). It multiplies by one factor,
/// except deg C → deg F which is ×1.8 + 32. Imperial and U.S. use the same table.
/// Units not in the table (rpm, %, g/s, V, A, mV …) are shown unchanged.
pub const UNIT_TABLE: [(&str, &str, f64); 14] = [
    ("mm", "in", 0.04),
    ("m", "ft", 3.28),
    ("km", "mile", 0.621),
    ("km/h", "mph", 0.621),
    ("°C", "°F", 1.8), // then + 32
    ("MPa", "psi", 145.04),
    ("bar", "psi", 14.5),
    ("kPa", "psi", 0.145),
    ("Pa", "inH2o", 0.004),
    ("g", "oz", 0.035),
    ("kg", "lb", 2.2),
    ("t", "ton", 0.984),
    ("l", "gal", 0.22),
    ("Nm", "lb-ft", 0.738),
];
pub const DEG_F_OFFSET: f64 = 32.0;

/// Convert a metric value for display.
pub fn to_units(v: f64, unit: &'static str, units: Units) -> (f64, &'static str) {
    if units == Units::Metric {
        return (v, unit);
    }
    match UNIT_TABLE.iter().find(|(m, _, _)| *m == unit) {
        Some(("°C", f, k)) => (v * k + DEG_F_OFFSET, f),
        Some((_, f, k)) => (v * k, f),
        None => (v, unit),
    }
}

#[derive(Clone, PartialEq)]
pub enum DtcStatus {
    Current,
    Pending,
    History,
}

impl DtcStatus {
    pub fn label(&self) -> &'static str {
        match self {
            DtcStatus::Current => "Current",
            DtcStatus::Pending => "Pending",
            DtcStatus::History => "History",
        }
    }
}

#[derive(Clone)]
pub struct Dtc {
    pub raw: [u8; 2],
    /// UDS failure-type byte (3-byte DTC); None for OBD-II service 03/07 codes
    pub ft: Option<u8>,
    pub text: &'static str,
    pub status: DtcStatus,
}

impl Dtc {
    /// "P1667" for OBD-II codes, "P166712" (with the failure-type byte) for UDS codes.
    pub fn code(&self) -> String {
        let base = dtc_from_bytes(self.raw[0], self.raw[1]);
        match self.ft {
            Some(f) => format!("{base}{f:02X}"),
            None => base,
        }
    }
}

/// DTC formatter: top two bits of the first byte
/// pick P/C/B/U (0x50/0x43/0x42/0x55), then (a>>4)&3, a&0xF, b>>4, b&0xF as upper-case hex.
/// "@PCBU" values carry a 3-byte code and use value>>8 (status byte dropped);
/// "@PCBU": a 3-byte code value, shifted right by 8.
#[allow(dead_code)] // kept with its test: the 3-byte formatter
pub const PCBU_SHIFT: u32 = 8;
#[allow(dead_code)]
pub fn dtc_from_pcbu(v: u32) -> String {
    let code = v >> PCBU_SHIFT;
    dtc_from_bytes((code >> 8) as u8, code as u8)
}

pub fn dtc_from_bytes(a: u8, b: u8) -> String {
    let letter = ['P', 'C', 'B', 'U'][(a >> 6) as usize];
    format!("{letter}{}{:X}{:02X}", (a >> 4) & 0x3, a & 0x0F, b)
}

#[derive(Clone)]
pub struct System {
    pub name: &'static str,
    pub dtcs: Vec<Dtc>,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn codes() {
        assert_eq!(dtc_from_bytes(0x01, 0x02), "P0102");
        assert_eq!(dtc_from_bytes(0x40, 0x35), "C0035");
        assert_eq!(dtc_from_bytes(0xC1, 0x00), "U0100");
        assert_eq!(dtc_from_pcbu(0x01_02_09), "P0102");
    }
    #[test]
    fn rpm() {
        let (a, b) = encode(0x0C, 2000.0);
        assert_eq!(decode(0x0C, a, b), 2000.0);
    }
}
