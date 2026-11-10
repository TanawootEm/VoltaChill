//! Real car link: an ELM327-compatible Bluetooth LE adapter ("OBDII", service FFF0).
//! This drives a cheap ELM327 adapter so
//! the app can read a real car. BLE runs on its own thread; the UI sends `Cmd`s and
//! reads `Ev`s, so the window never blocks.
//!
//! Adapter facts (proven earlier with the BYD ATTO 3, see ~/Desktop/ATTO3/app.py):
//! write to FFF2 *with response* (without response is silently dropped), notify on FFF1,
//! a reply ends with '>'. The car must be ON/READY and no other app may hold the adapter.

use crate::car;
use std::sync::mpsc::{channel, Receiver, Sender};

/// PDC module request header (used for read-only diagnostics).
pub const PDC: u16 = 0x7E2;

#[cfg(not(target_os = "android"))]
use btleplug::api::{Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType};
#[cfg(not(target_os = "android"))]
use btleplug::platform::{Manager, Peripheral};
#[cfg(not(target_os = "android"))]
use futures::{FutureExt, StreamExt};
#[cfg(not(target_os = "android"))]
use std::pin::Pin;
#[cfg(not(target_os = "android"))]
use std::time::{Duration, Instant};
#[cfg(not(target_os = "android"))]
use uuid::Uuid;

#[cfg(not(target_os = "android"))]
const SVC: Uuid = Uuid::from_u128(0x0000fff0_0000_1000_8000_00805f9b34fb);
#[cfg(not(target_os = "android"))]
const WRITE: Uuid = Uuid::from_u128(0x0000fff2_0000_1000_8000_00805f9b34fb);
#[cfg(not(target_os = "android"))]
const NOTIFY: Uuid = Uuid::from_u128(0x0000fff1_0000_1000_8000_00805f9b34fb);

/// Where a live value comes from.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Src {
    /// SAE J1979 service 01, decoded by `car::decode`.
    Mode01,
    /// Manufacturer data (UDS 0x22) from one module; decoded by `decode_did`.
    Did { hdr: u16, did: u16 },
    /// Voltage at the OBD socket (= 12 V battery), measured by the adapter (ELM `ATRV`).
    Volt12,
    /// A broadcast CAN frame heard on the bus (listen only); decoded in the app by
    /// `sniff::live_bus_value`. Never sent as a request.
    Bus { id: u16 },
}

/// Parse an ELM `ATRV` reply such as "12.6V".
pub fn parse_volt(r: &str) -> Option<f64> {
    r.lines().find_map(|l| l.trim().trim_end_matches(['V', 'v']).parse::<f64>().ok()).filter(|v| (5.0..30.0).contains(v))
}

/// Values every ELM327 adapter can give: the 12 V battery at the OBD socket.
pub fn adapter_pids() -> Vec<car::Pid> {
    vec![car::Pid { pid: 0, name: "12V battery", formula: "ATRV", unit: "V", min: 11.8, max: 14.8, bytes: 0, src: Src::Volt12 }]
}

/// ATTO 3 broadcast frames whose meaning was confirmed on this car (see sniff::decode_known).
pub fn byd_bus_pids() -> Vec<car::Pid> {
    let b = |name, formula, unit, min, max, id| car::Pid { pid: 0, name, formula, unit, min, max, bytes: 0, src: Src::Bus { id } };
    vec![
        b("Gear", "242 b5 low nibble", "gear", 1.0, 4.0, 0x242),
        b("Handbrake", "218 b5 bit4 (0 = on)", "onoff", 0.0, 1.0, 0x218),
        b("A/C set temperature", "404 b4", "°C", 16.0, 32.0, 0x404),
        b("A/C fan level", "404 b2 high nibble", "", 0.0, 7.0, 0x404),
        b("BMS thermal request", "44A b6 high nibble", "", 1.0, 1.0, 0x44A),
        b("Vehicle speed (bus)", "121 12 bit x 0.06875", "km/h", 0.0, 180.0, 0x121),
        b("Wheel speed FL", "122 b0-1 x 0.06875", "km/h", 0.0, 180.0, 0x122),
        b("Wheel speed FR", "122 b2-3 x 0.06875", "km/h", 0.0, 180.0, 0x122),
        b("Wheel speed RL", "122 b4-5 x 0.06875", "km/h", 0.0, 180.0, 0x122),
        b("Wheel speed RR", "122 b6-7 x 0.06875", "km/h", 0.0, 180.0, 0x122),
    ]
}

/// BYD (ATTO 3) values confirmed by direct reads (memory: atto3-obd-ble-reader).
/// Standard service 01 is refused by the BYD gateway, so these are the only live data there.
/// BYD ATTO 3 / Yuan PLUS diagnostic ECUs: name, role, CAN request/response IDs, and the
/// internal security-algorithm mode (NOT a `27` subfunction on the wire). Read-only
/// reference for display.
#[allow(dead_code)] // reference table; no longer shown in the UI
pub const BYD_ECUS: &[(&str, &str, u16, u16, u8)] = &[
    ("BASU", "Traction battery (BMS)", 0x7E7, 0x7EF, 0x0E),
    ("PDC", "Power distribution / coolant", 0x7E2, 0x7EA, 0x0F),
    ("LBMS", "12 V / starter battery mgmt", 0x762, 0x76A, 0x31),
    ("R-BCM", "Body control / thermal", 0x747, 0x74F, 0x2D),
];

pub fn byd_pids() -> Vec<car::Pid> {
    use car::Pid;
    let p = |name, formula, unit, min, max, hdr, did| Pid { pid: 0, name, formula, unit, min, max, bytes: 2, src: Src::Did { hdr, did } };
    // Names, DIDs and formulas: ATTO 3 BMS (BASU 7E7/7EF) definitions from the user's
    // BYD_7E7_BMS_DIDs.xlsx, module YUAN_PLUSEV_BASU_BYD_07E7_07EF_CAN (the car answers 258 of
    // its 259 DIDs). 0x0031 is the MAX cell temperature (not coolant), 0x0032 the average.
    vec![
        p("SOC (State of Charge)", "(256·B+A)/100", "%", 10.0, 100.0, 0x7E7, 0x1FFC),
        p("Battery Pack Current Total Voltage", "256·B+A", "V", 300.0, 460.0, 0x7E7, 0x0008),
        p("Battery Pack Current Total Current (+ = discharge)", "(256·B+A−5000)/10", "A", -300.0, 150.0, 0x7E7, 0x0009),
        p("Battery temperature", "A−40", "°C", 10.0, 45.0, 0x7E7, 0x0032),
        p("Max cell temperature", "A−40", "°C", 10.0, 45.0, 0x7E7, 0x0031),
        p("Min cell temperature", "A−40", "°C", 10.0, 45.0, 0x7E7, 0x002F),
        p("Battery SOH", "A", "%", 80.0, 100.0, 0x7E7, 0x0029),
        p("Minimum Single Cell Voltage", "256·B+A", "mV", 3000.0, 3500.0, 0x7E7, 0x002B),
        p("Maximum Single Cell Voltage", "256·B+A", "mV", 3000.0, 3500.0, 0x7E7, 0x002D),
        p("Lowest cell number", "A+1", "", 1.0, 126.0, 0x7E7, 0x002A),
        p("Highest cell number", "A+1", "", 1.0, 126.0, 0x7E7, 0x002C),
        p("Max allowed charge power", "(256·B+A)×0.1", "kW", 0.0, 200.0, 0x7E7, 0x000A),
        p("Max allowed discharge power", "(256·B+A)×0.1", "kW", 0.0, 300.0, 0x7E7, 0x000E),
        p("Insulation resistance", "256·B+A", "kΩ", 500.0, 65535.0, 0x7E7, 0x0015),
        p("Charged energy (lifetime)", "256·B+A", "kWh", 0.0, 1e6, 0x7E7, 0x0011),
        p("Discharged energy (lifetime)", "256·B+A", "kWh", 0.0, 1e6, 0x7E7, 0x0012),
        p("Number of Charges", "256·B+A", "", 0.0, 3000.0, 0x7E7, 0x000B),
    ]
    .into_iter()
    .chain(obc_pids())
    .collect()
}

/// On-board charger (OBC, request 7E4 / response 7EC) live values. DIDs, names and formulas
/// are BYD's own, not guesses. Ranges are display hints only.
pub const OBC: u16 = 0x7E4;
pub fn obc_pids() -> Vec<car::Pid> {
    let p = |name, formula, unit, min, max, did| car::Pid { pid: 0, name, formula, unit, min, max, bytes: 2, src: Src::Did { hdr: OBC, did } };
    vec![
        p("OBC internal temp 1", "A−40", "°C", -20.0, 85.0, 0x0013),
        p("OBC internal temp 2", "A−40", "°C", -20.0, 85.0, 0x0014),
        p("OBC internal temp 3", "A−40", "°C", -20.0, 85.0, 0x0015),
        p("OBC internal temp 4", "A−40", "°C", -20.0, 85.0, 0x0016),
        p("Charging port temp", "A−40", "°C", -20.0, 70.0, 0x0017),
        p("OBC cooling fan", "A", "%", 0.0, 100.0, 0x0019),
        p("AC voltage", "256·B+A", "V", 0.0, 260.0, 0x0003),
        p("AC current", "A×0.2", "A", 0.0, 33.0, 0x0007),
        p("AC power", "(256·B+A)×0.2 W", "kW", 0.0, 7.5, 0x0012),
        p("OBC state", "A", "obcstate", 0.0, 13.0, 0x001A),
    ]
}

/// Decode a UDS 0x22 reply (`d` = bytes after `62 hi lo`) for module `hdr`.
pub fn decode_did(hdr: u16, did: u16, d: &[u8]) -> Option<f64> {
    let a = *d.first()? as f64;
    let le = || d.get(1).map(|b| *b as f64 * 256.0 + a).unwrap_or(a);
    if hdr == PDC {
        // Power Domain Controller (BYD expressions): 0019 coolant temp A-40, 0021 pump %,
        // 0014/0015 fan high/low speed status (0 off, 1 on)
        return Some(match did {
            0x0019 | 0x0204 | 0x0205 => a - 40.0,
            _ => a,
        });
    }
    if hdr == OBC {
        // BYD's expressions: BYTE 3 = first data byte, BYTE 4 = second
        return Some(match did {
            0x0003 | 0x0004 => le(),         // AC / DC side voltage, V
            0x0005 => a * 0.6 - 50.0,        // DC side current, A
            0x0007 => a * 0.2,               // AC side current, A
            0x0011 => le() / 500.0,          // charge amount, kWh
            0x0012 => le() * 0.2 / 1000.0,   // AC side power, W -> kW
            0x0013..=0x0017 | 0x0021..=0x0023 => a - 40.0, // internal / port temperatures
            _ => a,                          // duty %, frequency, status codes
        });
    }
    Some(match did {
        // BMS per-BIC block (found by the read-only DID scan, 2026-11-09): 10 groups x 8 DIDs
        // [min cell no, min cell mV (LE), max cell no, max cell mV, min temp no, min temp,
        //  max temp no, max temp] — temperatures A-40 like 0x0031/0x0032.
        0x016C..=0x01BB => match (did - 0x016C) % 8 {
            1 | 3 => le(),
            5 | 7 => a - 40.0,
            _ => a,
        },
        0x1FFC => le() / 100.0,
        0x0009 => (le() - 5000.0) / 10.0,
        0x0008 | 0x002B | 0x002D | 0x000B | 0x0004 | 0x0011 | 0x0012 | 0x0015 => le(),
        0x0040..=0x00BD => le(), // cell 1-126 accumulated balancing time, h
        0x000A | 0x000E => le() * 0.1,
        0x0013 | 0x0014 => le() * 0.001,
        0x002A | 0x002C | 0x002E | 0x0030 => a + 1.0, // cell / sensor numbers are 1-based
        0x0029 | 0x0016 | 0x0017 | 0x001B | 0x0027 => a,
        _ => a - 40.0, // the temperature DIDs (002F min, 0031 max, 0032 average)
    })
}

pub enum Cmd {
    Connect,
    /// Scan for nearby BLE devices and return the list (no connection made).
    ScanDevices,
    /// Connect to one specific device chosen from the scan list (its `BtDevice.id`).
    ConnectTo(String),
    /// Drop the Bluetooth link (state goes back to Off).
    Disconnect,
    /// Work out which live values this car gives.
    Discover,
    /// Read the VIN from the connected car (service 09 02, else UDS DID F190).
    ReadVin,
    /// READ-ONLY sweep: UDS 0x22 ReadDataByIdentifier for every DID in `ranges` on module
    /// `hdr`; reports the DIDs that answer positively. Sends nothing else.
    /// `extended`: enter the extended diagnostic session (10 03) first and return to the
    /// default session (10 01) afterwards.
    DidScan { hdr: u16, ranges: Vec<(u16, u16)>, extended: bool },
    /// READ-ONLY: UDS 0x22 for each DID in `dids` on module `hdr`, then (if any) `ext_dids` inside
    /// an extended session (10 03 … 10 01). Raw data bytes come back in Ev::Dids.
    ReadDids { hdr: u16, dids: Vec<u16>, ext_dids: Vec<u16> },
    /// Background monitoring (Android): while `on`, whenever the UI has sent nothing for a few
    /// seconds (screen off / app in the back) the worker itself reads `req` every ~2 s and
    /// appends rows to `csv`. Also starts / stops the foreground service.
    #[cfg_attr(not(target_os = "android"), allow(dead_code))]
    BgMonitor { on: bool, csv: Option<String>, req: Vec<(u8, Src)> },
    Live(Vec<(u8, Src)>),
    ReadDtc,
    ClearDtc,
    /// Listen to the CAN bus without sending anything (ELM327 ATMA, silent monitoring)
    /// for `ms`, only IDs where (id & mask) == filt.
    Sniff { filt: u16, mask: u16, ms: u64 },
}

/// One CAN frame heard on the bus.
#[derive(Clone)]
pub struct Frame {
    pub id: u16,
    pub data: Vec<u8>,
}

#[derive(Clone)]
pub struct RealDtc {
    pub a: u8,
    pub b: u8,
    /// UDS failure-type byte; None for OBD-II service 03/07 codes
    pub c: Option<u8>,
    pub status: car::DtcStatus,
}

/// One BYD diagnostic module.
pub struct BydModule {
    pub req: u16,
    pub resp: u16,
    pub name: &'static str,
}

const BYD_DTC_TABLE: &str = include_str!("byd_dtc.txt");

/// The 36 ATTO 3 modules (CAN request / response IDs, name).
pub fn byd_modules() -> &'static [BydModule] {
    static M: std::sync::OnceLock<Vec<BydModule>> = std::sync::OnceLock::new();
    M.get_or_init(|| {
        BYD_DTC_TABLE
            .lines()
            .filter_map(|l| {
                let mut it = l.splitn(4, ' ');
                if it.next()? != "M" {
                    return None;
                }
                let req = u16::from_str_radix(it.next()?, 16).ok()?;
                let resp = u16::from_str_radix(it.next()?, 16).ok()?;
                Some(BydModule { req, resp, name: it.next()? })
            })
            .collect()
    })
}

/// BYD's own description of a DTC on module `req`. `code` is "P1667" (+ `ft` byte): tries the
/// exact 7-character code, then the 5-character code, then any entry with the same base.
pub fn byd_dtc_text(req: u16, code: &str, ft: Option<u8>) -> Option<&'static str> {
    use std::collections::HashMap;
    static T: std::sync::OnceLock<HashMap<(u16, &'static str), &'static str>> = std::sync::OnceLock::new();
    let t = T.get_or_init(|| {
        BYD_DTC_TABLE
            .lines()
            .filter_map(|l| {
                let mut it = l.splitn(4, ' ');
                if it.next()? != "D" {
                    return None;
                }
                let req = u16::from_str_radix(it.next()?, 16).ok()?;
                Some(((req, it.next()?), it.next()?))
            })
            .collect()
    });
    if let Some(f) = ft {
        if let Some(x) = t.get(&(req, format!("{code}{f:02X}").as_str())) {
            return Some(x);
        }
    }
    if let Some(x) = t.get(&(req, code)) {
        return Some(x);
    }
    t.iter().find(|((r, c), _)| *r == req && c.starts_with(code)).map(|(_, x)| *x)
}

/// Modules to ask for trouble codes: every ATTO 3 module, plus the generic 7E0–7E7 ones the
/// BYD table does not list. (request, response, name)
pub fn dtc_targets() -> Vec<(u16, u16, &'static str)> {
    let mut v: Vec<(u16, u16, &'static str)> = byd_modules().iter().map(|m| (m.req, m.resp, m.name)).collect();
    for (hdr, name) in ECUS {
        if !v.iter().any(|t| t.0 == hdr) {
            v.push((hdr, hdr + 8, name));
        }
    }
    v
}

/// One Bluetooth device seen during a scan.
#[derive(Clone)]
pub struct BtDevice {
    pub id: String,
    pub name: String,
    /// looks like an OBD adapter (name match or FFF0 service)
    pub obd: bool,
}

pub enum Ev {
    State(LinkState),
    /// adapter traffic log line (desktop worker; Android logs to logcat instead)
    #[cfg_attr(target_os = "android", allow(dead_code))]
    Log(String),
    /// Result of Cmd::ScanDevices: the devices found (OBD-looking ones first).
    Devices(Vec<BtDevice>),
    Discovered(Vec<car::Pid>),
    /// VIN read from the car (None = the car did not give one).
    Vin(Option<String>),
    DidScanProgress { done: usize, total: usize, found: usize },
    /// result of Cmd::ReadDids: (DID, data bytes after 62 hi lo) for each DID that answered
    Dids(Vec<(u16, Vec<u8>)>),
    /// (DID, data bytes after `62 hi lo`) for every DID that answered.
    DidScanDone(Vec<(u16, Vec<u8>)>),
    Live(Vec<Option<(f64, [u8; 2])>>),
    /// (module request ID — 0x7DF for OBD-II, module name, its codes) for every module that answered
    Dtcs(Vec<(u16, &'static str, Vec<RealDtc>)>),
    /// modules asked so far while reading or clearing codes
    DtcProgress { done: usize, total: usize },
    Cleared(String),
    Frames { filt: u16, mask: u16, frames: Vec<Frame>, overflow: bool, silent: bool },
}

#[derive(Clone, PartialEq, Debug)]
pub enum LinkState {
    Off,
    Searching,
    Connected(String),
    Failed(String),
}

pub struct Link {
    tx: Sender<Cmd>,
    pub rx: Receiver<Ev>,
    pub state: LinkState,
    pub log: Vec<String>,
    /// A request is out and its answer has not come back yet.
    pub busy: bool,
}

impl Link {
    pub fn start() -> Self {
        let (tx, crx) = channel();
        let (etx, rx) = channel();
        #[cfg(not(target_os = "android"))]
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().expect("tokio");
            rt.block_on(worker(crx, etx));
        });
        #[cfg(target_os = "android")]
        std::thread::spawn(move || worker(crx, etx));
        Self { tx, rx, state: LinkState::Off, log: Vec::new(), busy: false }
    }

    pub fn send(&mut self, c: Cmd) {
        if !matches!(c, Cmd::Connect | Cmd::ScanDevices | Cmd::ConnectTo(_) | Cmd::Disconnect | Cmd::BgMonitor { .. }) {
            self.busy = true;
        }
        let _ = self.tx.send(c);
    }

    pub fn connected(&self) -> bool {
        matches!(self.state, LinkState::Connected(_))
    }

    pub fn push_log(&mut self, l: String) {
        self.log.push(l);
        if self.log.len() > 200 {
            self.log.remove(0);
        }
    }
}

#[cfg(not(target_os = "android"))]
type Notes = Pin<Box<dyn futures::Stream<Item = btleplug::api::ValueNotification> + Send>>;

#[cfg(not(target_os = "android"))]
struct Elm {
    p: Peripheral,
    w: Characteristic,
    notes: Notes,
    header: u16,
    proto: char,
    log: Sender<Ev>,
    /// adapter is set up for monitoring with this (filter, mask); None = normal mode
    mon: Option<(u16, u16)>,
    silent: bool,
}

#[cfg(not(target_os = "android"))]
impl Elm {
    async fn cmd(&mut self, s: &str, wait_ms: u64) -> String {
        while let Some(Some(_)) = self.notes.next().now_or_never() {} // drop stale bytes
        let mut data = s.as_bytes().to_vec();
        data.push(b'\r');
        if let Err(e) = self.p.write(&self.w, &data, WriteType::WithResponse).await {
            return format!("ERROR {e}");
        }
        let mut buf = Vec::new();
        let end = Instant::now() + Duration::from_millis(wait_ms);
        loop {
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            match tokio::time::timeout(left, self.notes.next()).await {
                Ok(Some(n)) => {
                    buf.extend_from_slice(&n.value);
                    if buf.contains(&b'>') {
                        break;
                    }
                }
                _ => break,
            }
        }
        let r = String::from_utf8_lossy(&buf).replace('>', "").replace('\r', "\n").trim().to_string();
        let _ = self.log.send(Ev::Log(format!("> {s}   {}", r.replace('\n', " | "))));
        r
    }

    async fn set_header(&mut self, h: u16) {
        if self.header == h || !self.is_can() {
            return;
        }
        self.cmd(&format!("ATSH{h:03X}"), 1500).await;
        if h != 0x7DF {
            // flow control for multi-frame replies from one module
            self.cmd(&format!("ATFCSH{h:03X}"), 1500).await;
            self.cmd("ATFCSD300000", 1500).await;
            self.cmd("ATFCSM1", 1500).await;
        }
        self.header = h;
    }

    /// Listen with ATMA for `ms`, then stop it and put the adapter back in normal mode.
    async fn sniff(&mut self, filt: u16, mask: u16, ms: u64) -> Ev {
        // Set-up is kept between windows (each AT command is a BLE round trip); `normal()`
        // undoes it before any request to the car.
        if self.mon.is_none() {
            // headers + spaces on, raw 8-byte frames, do not acknowledge (silent) where supported
            for c in ["ATH1", "ATS1", "ATCAF0", "ATD0"] {
                self.cmd(c, 1500).await;
            }
            self.silent = !self.cmd("ATCSM1", 1500).await.contains('?');
        }
        let (old_f, old_m) = self.mon.unwrap_or((0xFFFF, 0xFFFF));
        if old_f != filt {
            self.cmd(&format!("ATCF{filt:03X}"), 1500).await;
        }
        if old_m != mask {
            self.cmd(&format!("ATCM{mask:03X}"), 1500).await;
        }
        self.mon = Some((filt, mask));
        let silent = self.silent;
        while let Some(Some(_)) = self.notes.next().now_or_never() {}
        let mut buf = Vec::new();
        if self.p.write(&self.w, b"ATMA\r", WriteType::WithResponse).await.is_ok() {
            let end = Instant::now() + Duration::from_millis(ms);
            loop {
                let left = end.saturating_duration_since(Instant::now());
                if left.is_zero() {
                    break;
                }
                if let Ok(Some(n)) = tokio::time::timeout(left, self.notes.next()).await {
                    buf.extend_from_slice(&n.value);
                }
            }
            // any character stops monitoring; wait for the prompt
            let _ = self.p.write(&self.w, b"\r", WriteType::WithResponse).await;
            let end = Instant::now() + Duration::from_millis(2500);
            while !buf.ends_with(b">") {
                let left = end.saturating_duration_since(Instant::now());
                match tokio::time::timeout(left, self.notes.next()).await {
                    Ok(Some(n)) => buf.extend_from_slice(&n.value),
                    _ => break,
                }
            }
        }
        let text = String::from_utf8_lossy(&buf).to_string();
        let overflow = text.contains("BUFFER FULL");
        let frames = parse_monitor(&text);
        let _ = self.log.send(Ev::Log(format!(
            "monitor {filt:03X}/{mask:03X}: {} frames{}",
            frames.len(),
            if overflow { ", BUFFER FULL" } else { "" }
        )));
        Ev::Frames { filt, mask, frames, overflow, silent }
    }

    /// Leave monitoring set-up: no filter, formatting on, headers/spaces off.
    async fn normal(&mut self) {
        if self.mon.take().is_some() {
            for c in ["ATCF000", "ATCM000", "ATCRA", "ATAR", "ATCAF1", "ATH0", "ATS0"] {
                self.cmd(c, 1500).await;
            }
            self.header = 0; // force ATSH again on the next request
        }
    }

    fn is_can(&self) -> bool {
        matches!(self.proto, '6'..='9')
    }

    /// Ask one module for a manufacturer value (UDS 0x22). Returns the data bytes.
    async fn read_did(&mut self, hdr: u16, did: u16) -> Option<Vec<u8>> {
        self.set_header(hdr).await;
        let r = self.cmd(&format!("22{did:04X}"), 2000).await;
        messages(&r).into_iter().find(|m| m.len() > 3 && m[0] == 0x62 && m[1] == (did >> 8) as u8 && m[2] == did as u8).map(|m| m[3..].to_vec())
    }
}

/// Split an ELM327 reply (spaces off, headers off) into messages.
/// CAN multi-frame looks like "014 / 0:490201… / 1:…": a length line, then "n:" frames.
pub fn messages(r: &str) -> Vec<Vec<u8>> {
    let mut out: Vec<Vec<u8>> = Vec::new();
    let mut want: Option<usize> = None;
    for line in r.lines() {
        let l: String = line.chars().filter(|c| !c.is_whitespace()).collect::<String>().to_uppercase();
        if l.is_empty() || l.contains("SEARCHING") || l == "OK" {
            continue;
        }
        if l.len() == 3 && l.chars().all(|c| c.is_ascii_hexdigit()) {
            want = usize::from_str_radix(&l, 16).ok();
            out.push(Vec::new());
            continue;
        }
        let (cont, hex) = match l.split_once(':') {
            Some((idx, rest)) if idx.len() == 1 => (true, rest.to_string()),
            _ => (false, l.clone()),
        };
        if !hex.chars().all(|c| c.is_ascii_hexdigit()) || hex.len() % 2 != 0 {
            continue; // NO DATA, ERROR, STOPPED, ? …
        }
        let bytes: Vec<u8> = (0..hex.len()).step_by(2).filter_map(|i| u8::from_str_radix(&hex[i..i + 2], 16).ok()).collect();
        if cont && !out.is_empty() {
            let last = out.last_mut().unwrap();
            last.extend(bytes);
            if let Some(n) = want {
                last.truncate(n);
            }
        } else {
            want = None;
            out.push(bytes);
        }
    }
    out.retain(|m| !m.is_empty());
    out
}

/// VIN from service 09 02 replies: reply starts with 0x49 and is at least 20 bytes; the
/// 17 characters must be letters or digits.
pub fn vin_from_0902(msgs: &[Vec<u8>]) -> Option<String> {
    let mut all = Vec::new();
    for m in msgs.iter().filter(|m| m.first() == Some(&0x49)) {
        all.extend(if m.len() > 3 { &m[3..] } else { &m[..0] }); // skip 49 02 <count/index>
    }
    if all.len() + 3 < car::VIN_REPLY_MIN_BYTES {
        return None;
    }
    vin_chars(&all)
}

fn vin_chars(b: &[u8]) -> Option<String> {
    let s: String = b.iter().filter(|c| c.is_ascii_alphanumeric()).map(|c| *c as char).collect();
    (s.len() >= 17).then(|| s[s.len() - 17..].to_string())
}

pub const ECUS: [(u16, &str); 8] = [
    (0x7E0, "Module 7E0 (engine / VCU)"),
    (0x7E1, "Module 7E1 (transmission)"),
    (0x7E2, "Module 7E2 (hybrid / inverter)"),
    (0x7E3, "Module 7E3"),
    (0x7E4, "Module 7E4 (motor / BCM)"),
    (0x7E5, "Module 7E5"),
    (0x7E6, "Module 7E6"),
    (0x7E7, "Module 7E7 (battery / BMS)"),
];

/// Scan ~6 s for nearby BLE devices and return them (OBD-looking ones first). No connection.
#[cfg(not(target_os = "android"))]
async fn scan_devices(log: &Sender<Ev>) -> Vec<BtDevice> {
    let say = |s: String| {
        let _ = log.send(Ev::Log(s));
    };
    let Ok(m) = Manager::new().await else {
        say("Bluetooth not available".into());
        return Vec::new();
    };
    let Some(ad) = m.adapters().await.ok().and_then(|a| a.into_iter().next()) else {
        say("no Bluetooth adapter".into());
        return Vec::new();
    };
    let _ = ad.start_scan(ScanFilter::default()).await;
    say("Scanning for Bluetooth devices…".into());
    let mut out: Vec<BtDevice> = Vec::new();
    let end = Instant::now() + Duration::from_secs(6);
    while Instant::now() < end {
        tokio::time::sleep(Duration::from_millis(600)).await;
        out.clear();
        for p in ad.peripherals().await.unwrap_or_default() {
            if let Ok(Some(pr)) = p.properties().await {
                let name = pr.local_name.clone().unwrap_or_default();
                let u = name.to_uppercase();
                let obd = pr.services.contains(&SVC) || ["OBD", "ELM", "VLINK", "V-LINK", "VEEPEAK", "KONNWEI"].iter().any(|k| u.contains(k));
                if !name.is_empty() || obd {
                    out.push(BtDevice {
                        id: format!("{:?}", p.id()),
                        name: if name.is_empty() { "(unnamed)".into() } else { name },
                        obd,
                    });
                }
            }
        }
    }
    let _ = ad.stop_scan().await;
    out.sort_by(|a, b| b.obd.cmp(&a.obd).then_with(|| a.name.cmp(&b.name)));
    say(format!("Scan done: {} device(s)", out.len()));
    out
}

/// Connect to the OBD adapter. `target` = a specific device id from the scan list, or None to
/// auto-pick the first OBD-looking adapter (name OBD/ELM/Vlink or service FFF0).
#[cfg(not(target_os = "android"))]
async fn connect(log: &Sender<Ev>, target: Option<String>) -> Result<Elm, String> {
    let say = |s: String| {
        let _ = log.send(Ev::Log(s));
    };
    let m = Manager::new().await.map_err(|e| format!("Bluetooth: {e}"))?;
    let ad = m.adapters().await.map_err(|e| e.to_string())?.into_iter().next().ok_or("no Bluetooth adapter")?;
    ad.start_scan(ScanFilter::default()).await.map_err(|e| format!("scan: {e}"))?;
    match &target {
        Some(id) => say(format!("Connecting to the chosen device ({id})…")),
        None => say("Searching for the OBD adapter (name OBD/ELM/Vlink or service FFF0)…".into()),
    }
    let mut found = None;
    let end = Instant::now() + Duration::from_secs(15);
    while found.is_none() && Instant::now() < end {
        tokio::time::sleep(Duration::from_millis(500)).await;
        for p in ad.peripherals().await.unwrap_or_default() {
            if let Ok(Some(pr)) = p.properties().await {
                let name = pr.local_name.clone().unwrap_or_default();
                let u = name.to_uppercase();
                let hit = match &target {
                    Some(id) => format!("{:?}", p.id()) == *id,
                    None => pr.services.contains(&SVC) || ["OBD", "ELM", "VLINK", "V-LINK", "VEEPEAK", "KONNWEI"].iter().any(|k| u.contains(k)),
                };
                if hit {
                    say(format!("Found \"{name}\" ({})", p.id()));
                    found = Some((p, name));
                    break;
                }
            }
        }
    }
    let _ = ad.stop_scan().await;
    let (p, name) = found.ok_or("device not found — in range? car ON? another OBD app disconnected?")?;
    let mut last = String::new();
    let mut ok = false;
    for i in 1..=5 {
        match p.connect().await {
            Ok(()) => {
                ok = true;
                break;
            }
            Err(e) => {
                last = e.to_string();
                say(format!("connect try {i}/5 failed: {last}"));
                tokio::time::sleep(Duration::from_millis(800)).await;
            }
        }
    }
    if !ok {
        return Err(format!("could not connect: {last}"));
    }
    p.discover_services().await.map_err(|e| e.to_string())?;
    let chars = p.characteristics();
    let w = chars.iter().find(|c| c.uuid == WRITE).or_else(|| chars.iter().find(|c| c.properties.contains(CharPropFlags::WRITE))).cloned().ok_or("no write characteristic")?;
    let n = chars.iter().find(|c| c.uuid == NOTIFY).or_else(|| chars.iter().find(|c| c.properties.contains(CharPropFlags::NOTIFY))).cloned().ok_or("no notify characteristic")?;
    p.subscribe(&n).await.map_err(|e| e.to_string())?;
    let notes = p.notifications().await.map_err(|e| e.to_string())?;
    let mut elm = Elm { p, w, notes, header: 0, proto: '0', log: log.clone(), mon: None, silent: true };
    let id = elm.cmd("ATZ", 3000).await;
    for c in ["ATE0", "ATL0", "ATS0", "ATH0", "ATAT1", "ATST64"] {
        elm.cmd(c, 1500).await;
    }
    // find the car's protocol once (auto search), so screens that skip Auto Scan still work
    elm.cmd("ATSP0", 1500).await;
    elm.cmd("0100", 8000).await;
    let dp = elm.cmd("ATDPN", 1500).await;
    elm.proto = dp.trim().chars().last().filter(|c| c.is_ascii_hexdigit()).unwrap_or('0').to_ascii_uppercase();
    if !elm.is_can() {
        // BYD answers nothing on service 01 → the search can fail; ISO 15765 11-bit 500K is the usual bus
        elm.cmd("ATSP6", 1500).await;
        elm.proto = '6';
    }
    let chip = id.lines().find(|l| l.contains("ELM")).unwrap_or("ELM327?").trim().to_string();
    let _ = log.send(Ev::State(LinkState::Connected(format!("{name} · {chip}"))));
    Ok(elm)
}

#[cfg(not(target_os = "android"))]
async fn worker(rx: Receiver<Cmd>, tx: Sender<Ev>) {
    let mut elm: Option<Elm> = None;
    while let Ok(c) = rx.recv() {
        if let Some(e) = &elm {
            if !e.p.is_connected().await.unwrap_or(false) {
                elm = None;
                let _ = tx.send(Ev::State(LinkState::Failed("adapter disconnected".into())));
            }
        }
        if let Cmd::BgMonitor { .. } = c {
            continue; // desktop: the UI keeps polling, nothing to do
        }
        if let Cmd::Disconnect = c {
            if let Some(e) = elm.take() {
                let _ = e.p.disconnect().await;
            }
            let _ = tx.send(Ev::State(LinkState::Off));
            continue;
        }
        if let Cmd::ScanDevices = c {
            let devices = scan_devices(&tx).await;
            let _ = tx.send(Ev::Devices(devices));
            continue;
        }
        if let Cmd::Connect | Cmd::ConnectTo(_) = c {
            let target = if let Cmd::ConnectTo(id) = c { Some(id) } else { None };
            let _ = tx.send(Ev::State(LinkState::Searching));
            match connect(&tx, target).await {
                Ok(e) => elm = Some(e),
                Err(e) => {
                    let _ = tx.send(Ev::State(LinkState::Failed(e)));
                }
            }
            continue;
        }
        let Some(e) = elm.as_mut() else {
            // answer anyway so the UI never waits forever
            let _ = tx.send(match c {
                Cmd::Discover => Ev::Discovered(Vec::new()),
                Cmd::ReadVin => Ev::Vin(None),
                Cmd::DidScan { .. } => Ev::DidScanDone(Vec::new()),
                Cmd::ReadDids { .. } => Ev::Dids(Vec::new()),
                Cmd::Live(v) => Ev::Live(vec![None; v.len()]),
                Cmd::ReadDtc => Ev::Dtcs(Vec::new()),
                Cmd::Sniff { filt, mask, .. } => Ev::Frames { filt, mask, frames: Vec::new(), overflow: false, silent: true },
                _ => Ev::Cleared("not connected".into()),
            });
            let _ = tx.send(Ev::State(LinkState::Failed("not connected".into())));
            continue;
        };
        if !matches!(c, Cmd::Sniff { .. }) {
            e.normal().await;
        }
        let ev = match c {
            Cmd::Connect | Cmd::ScanDevices | Cmd::ConnectTo(_) | Cmd::Disconnect | Cmd::BgMonitor { .. } => unreachable!(),
            Cmd::Discover => Ev::Discovered(discover(e).await),
            Cmd::ReadVin => Ev::Vin(read_vin(e).await),
            Cmd::ReadDids { hdr, dids, ext_dids } => {
                let mut out = Vec::new();
                for did in dids {
                    if let Some(d) = e.read_did(hdr, did).await {
                        out.push((did, d));
                    }
                }
                if !ext_dids.is_empty() {
                    e.set_header(hdr).await;
                    e.cmd("1003", 1500).await;
                    for did in ext_dids {
                        if let Some(d) = e.read_did(hdr, did).await {
                            out.push((did, d));
                        }
                    }
                    e.cmd("1001", 1500).await;
                }
                Ev::Dids(out)
            }
            Cmd::DidScan { hdr, ranges, extended } => {
                let total: usize = ranges.iter().map(|(a, b)| (*b as usize).saturating_sub(*a as usize) + 1).sum();
                let mut found = Vec::new();
                let mut done = 0;
                if extended {
                    e.set_header(hdr).await;
                    e.cmd("1003", 1500).await;
                }
                for (a, b) in ranges {
                    for did in a..=b {
                        if let Some(d) = e.read_did(hdr, did).await {
                            found.push((did, d));
                        }
                        done += 1;
                        if done % 16 == 0 {
                            let _ = tx.send(Ev::DidScanProgress { done, total, found: found.len() });
                        }
                    }
                }
                if extended {
                    e.set_header(hdr).await;
                    e.cmd("1001", 1500).await;
                }
                Ev::DidScanDone(found)
            }
            Cmd::Live(list) => {
                let mut out = Vec::new();
                for (pid, src) in list {
                    out.push(match src {
                        Src::Mode01 => {
                            e.set_header(0x7DF).await;
                            let r = e.cmd(&format!("01{pid:02X}"), 1500).await;
                            messages(&r).into_iter().find(|m| m.len() >= 3 && m[0] == 0x41 && m[1] == pid).map(|m| {
                                let (a, b) = (m[2], *m.get(3).unwrap_or(&0));
                                (car::decode(pid, a, b), [a, b])
                            })
                        }
                        Src::Did { hdr, did } => e.read_did(hdr, did).await.and_then(|d| {
                            let v = decode_did(hdr, did, &d)?;
                            Some((v, [d[0], *d.get(1).unwrap_or(&0)]))
                        }),
                        Src::Volt12 => parse_volt(&e.cmd("ATRV", 1000).await).map(|v| (v, [0, 0])),
                        Src::Bus { .. } => None, // heard by sniffing, never requested
                    });
                }
                Ev::Live(out)
            }
            Cmd::ReadDtc => Ev::Dtcs(read_dtc(e).await),
            Cmd::ClearDtc => Ev::Cleared(clear_dtc(e).await),
            Cmd::Sniff { filt, mask, ms } => e.sniff(filt, mask, ms).await,
        };
        let _ = tx.send(ev);
    }
}

/// VIN from the car: standard service 09 02 first, then UDS DID F190 from the modules that
/// usually hold it (BYD refuses service 09).
#[cfg(not(target_os = "android"))]
async fn read_vin(e: &mut Elm) -> Option<String> {
    e.set_header(0x7DF).await;
    let r = e.cmd("0902", 3000).await;
    if let Some(v) = vin_from_0902(&messages(&r)) {
        return Some(v);
    }
    if e.is_can() {
        for (hdr, _) in [ECUS[0], ECUS[7], ECUS[2]] {
            if let Some(d) = e.read_did(hdr, 0xF190).await {
                if let Some(v) = vin_chars(&d) {
                    return Some(v);
                }
            }
        }
    }
    None
}

#[cfg(not(target_os = "android"))]
async fn discover(e: &mut Elm) -> Vec<car::Pid> {
    // which standard PIDs does the car declare? (01 00 / 01 20 / 01 40 bitmaps)
    let mut supported = Vec::new();
    e.set_header(0x7DF).await;
    for base in [0x00u8, 0x20, 0x40] {
        let r = e.cmd(&format!("01{base:02X}"), 2000).await;
        let Some(m) = messages(&r).into_iter().find(|m| m.len() >= 6 && m[0] == 0x41 && m[1] == base) else { break };
        let bits = u32::from_be_bytes([m[2], m[3], m[4], m[5]]);
        for i in 0..32u8 {
            if bits & (1 << (31 - i)) != 0 {
                supported.push(base + i + 1);
            }
        }
        if bits & 1 == 0 {
            break;
        }
    }
    let mut list: Vec<car::Pid> = car::pids().into_iter().filter(|p| supported.contains(&p.pid)).collect();
    // keep only the ones that really answer (BYD declares 0D but refuses it)
    let mut ok = Vec::new();
    for p in list.drain(..) {
        let r = e.cmd(&format!("01{:02X}", p.pid), 1500).await;
        if messages(&r).iter().any(|m| m.len() >= 3 && m[0] == 0x41 && m[1] == p.pid) {
            ok.push(p);
        }
    }
    if e.is_can() && e.read_did(0x7E7, 0x1FFC).await.is_some() {
        let _ = e.log.send(Ev::Log("BYD battery module answers on 7E7 → adding battery values".into()));
        ok.extend(byd_pids());
        ok.extend(byd_bus_pids());
    }
    if parse_volt(&e.cmd("ATRV", 1000).await).is_some() {
        ok.extend(adapter_pids());
    }
    ok
}

fn status_of(s: u8) -> car::DtcStatus {
    if s & 0x01 != 0 {
        car::DtcStatus::Current
    } else if s & 0x04 != 0 {
        car::DtcStatus::Pending
    } else {
        car::DtcStatus::History
    }
}

#[cfg(not(target_os = "android"))]
async fn read_dtc(e: &mut Elm) -> Vec<(u16, &'static str, Vec<RealDtc>)> {
    let mut out = Vec::new();
    // 1) standard OBD: 03 = stored, 07 = pending
    e.set_header(0x7DF).await;
    let mut std_codes = Vec::new();
    let mut std_ok = false;
    for (svc, reply, st) in [("03", 0x43u8, car::DtcStatus::Current), ("07", 0x47, car::DtcStatus::Pending)] {
        let r = e.cmd(svc, 3000).await;
        for m in messages(&r).into_iter().filter(|m| m.first() == Some(&reply)) {
            std_ok = true;
            let body = if e.is_can() { &m[2.min(m.len())..] } else { &m[1..] };
            for p in body.chunks(2).filter(|p| p.len() == 2 && (p[0] | p[1]) != 0) {
                if !std_codes.iter().any(|d: &RealDtc| d.a == p[0] && d.b == p[1]) {
                    std_codes.push(RealDtc { a: p[0], b: p[1], c: None, status: st.clone() });
                }
            }
        }
    }
    if std_ok {
        out.push((0x7DF, "OBD-II emissions (service 03/07)", std_codes));
    }
    // 2) UDS 19 02 FF (read DTC by status) from every module, listening to its own reply ID
    if e.is_can() {
        let targets = dtc_targets();
        let total = targets.len();
        for (k, (req, resp, name)) in targets.into_iter().enumerate() {
            let _ = e.log.send(Ev::DtcProgress { done: k, total });
            e.set_header(req).await;
            e.cmd(&format!("ATCRA{resp:03X}"), 1000).await;
            let r = e.cmd("1902FF", 3000).await;
            if let Some(m) = messages(&r).into_iter().find(|m| m.len() >= 3 && m[0] == 0x59 && m[1] == 0x02) {
                out.push((req, name, uds_codes(&m)));
            }
        }
        e.cmd("ATAR", 1000).await;
        e.set_header(0x7DF).await;
    }
    out
}

/// Clear codes: OBD-II service 04, then UDS 14 FF FF FF on every module. Only ever sent from
/// the confirm dialog.
#[cfg(not(target_os = "android"))]
async fn clear_dtc(e: &mut Elm) -> String {
    e.set_header(0x7DF).await;
    let r = e.cmd("04", 3000).await;
    let mut done = vec![];
    if messages(&r).iter().any(|m| m.first() == Some(&0x44)) {
        done.push("OBD-II".to_string());
    }
    if e.is_can() {
        let targets = dtc_targets();
        let total = targets.len();
        for (k, (req, resp, _)) in targets.into_iter().enumerate() {
            let _ = e.log.send(Ev::DtcProgress { done: k, total });
            e.set_header(req).await;
            e.cmd(&format!("ATCRA{resp:03X}"), 1000).await;
            if messages(&e.cmd("14FFFFFF", 3000).await).iter().any(|m| m.first() == Some(&0x54)) {
                done.push(format!("{req:03X}"));
            }
        }
        e.cmd("ATAR", 1000).await;
        e.set_header(0x7DF).await;
    }
    clear_summary(&done)
}

/// 3-byte DTC + status records of a positive `59 02 <mask>` reply.
pub fn uds_codes(m: &[u8]) -> Vec<RealDtc> {
    m.get(3..)
        .unwrap_or(&[])
        .chunks(4)
        .filter(|c| c.len() == 4 && (c[0] | c[1] | c[2]) != 0)
        .map(|c| RealDtc { a: c[0], b: c[1], c: Some(c[2]), status: status_of(c[3]) })
        .collect()
}

fn clear_summary(done: &[String]) -> String {
    if done.is_empty() {
        "Clear refused by the car (no module accepted it)".into()
    } else {
        format!("Clear accepted by: {}", done.join(", "))
    }
}

/// Monitor lines look like "323 03 00 7F 12 00 00 00 00" (header + up to 8 bytes).
pub fn parse_monitor(text: &str) -> Vec<Frame> {
    let mut out = Vec::new();
    for line in text.split(['\r', '\n']) {
        let mut it = line.split_whitespace();
        let Some(h) = it.next() else { continue };
        if h.len() != 3 || !h.chars().all(|c| c.is_ascii_hexdigit()) {
            continue;
        }
        let Ok(id) = u16::from_str_radix(h, 16) else { continue };
        let data: Option<Vec<u8>> = it.map(|b| if b.len() == 2 { u8::from_str_radix(b, 16).ok() } else { None }).collect();
        if let Some(d) = data.filter(|d| !d.is_empty() && d.len() <= 8) {
            out.push(Frame { id, data: d });
        }
    }
    out
}

// ======================= Android BLE bridge (Phase 2b) =======================
// The ELM327/UDS protocol logic lives here in Rust; the raw BLE bytes cross JNI to a
// Kotlin BLE manager (com.tanawoot.scanlab.Ble). Desktop btleplug code is untouched.
#[cfg(target_os = "android")]
mod android_ble {
    use super::{
        adapter_pids, byd_bus_pids, byd_pids, car, clear_summary, decode_did, dtc_targets, parse_volt, uds_codes, messages, vin_chars, vin_from_0902, parse_monitor, BtDevice, Cmd, Ev, LinkState,
        RealDtc, Src, ECUS,
    };
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::mpsc::{channel, Receiver, Sender};
    use std::sync::{Mutex, OnceLock};
    use std::time::{Duration, Instant};

    /// Whether the connected adapter negotiated a CAN protocol (set in `elm_init`).
    static IS_CAN: AtomicBool = AtomicBool::new(false);
    fn is_can() -> bool {
        IS_CAN.load(Ordering::Relaxed)
    }

    use jni::objects::{GlobalRef, JByteArray, JClass, JObject, JString};
    use jni::sys::jboolean;
    use jni::{JNIEnv, JavaVM};

    static EV_TX: OnceLock<Sender<Ev>> = OnceLock::new();
    static NOTE_TX: OnceLock<Sender<Vec<u8>>> = OnceLock::new();
    static CONN_TX: OnceLock<Sender<Result<String, String>>> = OnceLock::new();
    static SCAN: Mutex<Vec<BtDevice>> = Mutex::new(Vec::new());
    /// Global ref to the Ble class, cached from a Java thread (nativeInit) because a natively
    /// attached thread's FindClass uses the system classloader and can't see app classes.
    static BLE_CLASS: OnceLock<GlobalRef> = OnceLock::new();

    fn send_ev(ev: Ev) {
        if let Some(tx) = EV_TX.get() {
            let _ = tx.send(ev);
        }
    }

    fn vm() -> Option<JavaVM> {
        let ctx = ndk_context::android_context();
        unsafe { JavaVM::from_raw(ctx.vm().cast()).ok() }
    }

    const BLE: &str = "com/tanawoot/scanlab/Ble";

    /// The cached Ble class (preferred) as a short-lived JClass; None before nativeInit.
    fn ble_class() -> Option<JClass<'static>> {
        BLE_CLASS.get().map(|g| unsafe { JClass::from_raw(g.as_raw()) })
    }

    fn with_env<F: FnOnce(&mut JNIEnv)>(f: F) {
        if let Some(vm) = vm() {
            if let Ok(mut guard) = vm.attach_current_thread() {
                f(&mut guard);
                let _ = guard.exception_clear();
            }
        }
    }

    fn call_void(method: &str) {
        with_env(|env| {
            if let Some(cls) = ble_class() {
                let _ = env.call_static_method(&cls, method, "()V", &[]);
            } else {
                let _ = env.call_static_method(BLE, method, "()V", &[]);
            }
        });
    }

    fn ble_connect(id: &str) {
        with_env(|env| {
            if let Ok(s) = env.new_string(id) {
                if let Some(cls) = ble_class() {
                    let _ = env.call_static_method(&cls, "connect", "(Ljava/lang/String;)V", &[(&s).into()]);
                } else {
                    let _ = env.call_static_method(BLE, "connect", "(Ljava/lang/String;)V", &[(&s).into()]);
                }
            }
        });
    }

    fn ble_write(data: &[u8]) {
        with_env(|env| {
            if let Ok(arr) = env.byte_array_from_slice(data) {
                if let Some(cls) = ble_class() {
                    let _ = env.call_static_method(&cls, "write", "([B)V", &[(&arr).into()]);
                } else {
                    let _ = env.call_static_method(BLE, "write", "([B)V", &[(&arr).into()]);
                }
            }
        });
    }

    /// Write one ELM line and collect the notification bytes until the '>' prompt (or timeout).
    fn cmd(s: &str, nrx: &Receiver<Vec<u8>>, wait_ms: u64) -> String {
        while nrx.try_recv().is_ok() {} // drop stale bytes
        let mut data = s.as_bytes().to_vec();
        data.push(b'\r');
        ble_write(&data);
        let mut buf = Vec::new();
        let t0 = Instant::now();
        let end = t0 + Duration::from_millis(wait_ms);
        loop {
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            match nrx.recv_timeout(left) {
                Ok(b) => {
                    buf.extend_from_slice(&b);
                    if buf.contains(&b'>') {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        let prompt = buf.contains(&b'>');
        let r = String::from_utf8_lossy(&buf).replace('>', "").replace('\r', "\n").trim().to_string();
        if !prompt {
            // no '>' in time: the reply may still arrive and be read as the next command's answer
            log::info!("[elm] TIMEOUT {}ms > {s}   {}", t0.elapsed().as_millis(), r.replace('\n', " | "));
        }
        // every request/reply except the 1 Hz live polling, for adb logcat (tag "voltachill")
        if !(s.starts_with("22") || s.starts_with("ATSH") || s.starts_with("ATFC") || s == "3E80") {
            log::info!("[elm] > {s}   {}", r.replace('\n', " | "));
        }
        r
    }

    fn elm_init(nrx: &Receiver<Vec<u8>>) {
        cmd("ATZ", nrx, 3000);
        for c in ["ATE0", "ATL0", "ATS0", "ATH0", "ATAT1", "ATST64", "ATSP0"] {
            cmd(c, nrx, 1500);
        }
        cmd("0100", nrx, 8000);
        let dp = cmd("ATDPN", nrx, 1500);
        let can = dp.trim().chars().last().map(|c| "6789AB".contains(c)).unwrap_or(false);
        if !can {
            cmd("ATSP6", nrx, 1500); // force a CAN protocol (BYD and most modern cars)
        }
        // elm_init always ends on a CAN protocol here (auto-detected, or forced to SP6 above).
        IS_CAN.store(true, Ordering::Relaxed);
    }

    fn set_header(h: u16, cur: &mut u16, nrx: &Receiver<Vec<u8>>) {
        if *cur == h {
            return;
        }
        cmd(&format!("ATSH{h:03X}"), nrx, 1500);
        if h != 0x7DF {
            cmd(&format!("ATFCSH{h:03X}"), nrx, 1500);
            cmd("ATFCSD300000", nrx, 1500);
            cmd("ATFCSM1", nrx, 1500);
        }
        *cur = h;
    }

    fn read_one(pid: u8, src: &Src, cur: &mut u16, nrx: &Receiver<Vec<u8>>) -> Option<(f64, [u8; 2])> {
        match src {
            Src::Did { hdr, did } => {
                set_header(*hdr, cur, nrx);
                let r = cmd(&format!("22{did:04X}"), nrx, 2000);
                let Some(m) = messages(&r)
                    .into_iter()
                    .find(|m| m.len() > 3 && m[0] == 0x62 && m[1] == (did >> 8) as u8 && m[2] == *did as u8)
                else {
                    log::info!("[did] miss {hdr:03X} 22{did:04X}   {}", r.replace('\n', " | "));
                    return None;
                };
                let v = decode_did(*hdr, *did, &m[3..])?;
                Some((v, [0, 0]))
            }
            Src::Mode01 => {
                set_header(0x7DF, cur, nrx);
                let r = cmd(&format!("01{pid:02X}"), nrx, 1500);
                let m = messages(&r).into_iter().find(|m| m.len() >= 3 && m[0] == 0x41 && m[1] == pid)?;
                let (a, b) = (m[2], *m.get(3).unwrap_or(&0));
                Some((car::decode(pid, a, b), [a, b]))
            }
            Src::Volt12 => parse_volt(&cmd("ATRV", nrx, 1000)).map(|v| (v, [0, 0])),
            Src::Bus { .. } => None,
        }
    }

    // ---- ported diagnostics (same UDS logic as desktop, over the JNI bridge) ----

    /// Read DTCs: OBD-II service 03/07, then UDS 19 02 FF per module on CAN cars.
    fn read_dtc(header: &mut u16, nrx: &Receiver<Vec<u8>>) -> Vec<(u16, &'static str, Vec<RealDtc>)> {
        let mut out = Vec::new();
        set_header(0x7DF, header, nrx);
        let mut std_codes = Vec::new();
        let mut std_ok = false;
        for (svc, reply, st) in [("03", 0x43u8, car::DtcStatus::Current), ("07", 0x47, car::DtcStatus::Pending)] {
            let r = cmd(svc, nrx, 3000);
            for m in messages(&r).into_iter().filter(|m| m.first() == Some(&reply)) {
                std_ok = true;
                let body = if is_can() { &m[2.min(m.len())..] } else { &m[1..] };
                for p in body.chunks(2).filter(|p| p.len() == 2 && (p[0] | p[1]) != 0) {
                    if !std_codes.iter().any(|d: &RealDtc| d.a == p[0] && d.b == p[1]) {
                        std_codes.push(RealDtc { a: p[0], b: p[1], c: None, status: st.clone() });
                    }
                }
            }
        }
        if std_ok {
            out.push((0x7DF, "OBD-II emissions (service 03/07)", std_codes));
        }
        if is_can() {
            let targets = dtc_targets();
            let total = targets.len();
            for (k, (req, resp, name)) in targets.into_iter().enumerate() {
                send_ev(Ev::DtcProgress { done: k, total });
                set_header(req, header, nrx);
                cmd(&format!("ATCRA{resp:03X}"), nrx, 1000);
                let r = cmd("1902FF", nrx, 3000);
                if let Some(m) = messages(&r).into_iter().find(|m| m.len() >= 3 && m[0] == 0x59 && m[1] == 0x02) {
                    out.push((req, name, uds_codes(&m)));
                }
            }
            cmd("ATAR", nrx, 1000);
            set_header(0x7DF, header, nrx);
        }
        out
    }

    /// OBD-II 04 + UDS 14 FF FF FF on every module (from the confirm dialog only).
    fn clear_dtc(header: &mut u16, nrx: &Receiver<Vec<u8>>) -> String {
        set_header(0x7DF, header, nrx);
        let mut done = vec![];
        if messages(&cmd("04", nrx, 3000)).iter().any(|m| m.first() == Some(&0x44)) {
            done.push("OBD-II".to_string());
        }
        if is_can() {
            let targets = dtc_targets();
            let total = targets.len();
            for (k, (req, resp, _)) in targets.into_iter().enumerate() {
                send_ev(Ev::DtcProgress { done: k, total });
                set_header(req, header, nrx);
                cmd(&format!("ATCRA{resp:03X}"), nrx, 1000);
                if messages(&cmd("14FFFFFF", nrx, 3000)).iter().any(|m| m.first() == Some(&0x54)) {
                    done.push(format!("{req:03X}"));
                }
            }
            cmd("ATAR", nrx, 1000);
            set_header(0x7DF, header, nrx);
        }
        clear_summary(&done)
    }

    /// Monitor set-up kept between sniff windows: (filter, mask, silent). Each AT command is a
    /// BLE round trip, so it is only sent when something changes; `normal()` undoes it before
    /// any request to the car (same scheme as the desktop `Elm`).
    static MON: Mutex<Option<(u16, u16, bool)>> = Mutex::new(None);

    /// Leave monitoring set-up (no-op when not monitoring). Invalidates `*header`.
    fn normal(header: &mut u16, nrx: &Receiver<Vec<u8>>) {
        if MON.lock().unwrap().take().is_some() {
            for c in ["ATCF000", "ATCM000", "ATCRA", "ATAR", "ATCAF1", "ATH0", "ATS0"] {
                cmd(c, nrx, 1500);
            }
            *header = 0;
        }
    }

    /// ATMA bus monitor for `ms`. Set-up stays active for the next window.
    fn sniff(filt: u16, mask: u16, ms: u64, nrx: &Receiver<Vec<u8>>) -> Ev {
        let prev = *MON.lock().unwrap();
        let silent = match prev {
            Some((_, _, s)) => s,
            None => {
                for c in ["ATH1", "ATS1", "ATCAF0", "ATD0"] {
                    cmd(c, nrx, 1500);
                }
                !cmd("ATCSM1", nrx, 1500).contains('?')
            }
        };
        let (old_f, old_m) = prev.map(|(f, m, _)| (f, m)).unwrap_or((0xFFFF, 0xFFFF));
        if old_f != filt {
            cmd(&format!("ATCF{filt:03X}"), nrx, 1500);
        }
        if old_m != mask {
            cmd(&format!("ATCM{mask:03X}"), nrx, 1500);
        }
        *MON.lock().unwrap() = Some((filt, mask, silent));
        while nrx.try_recv().is_ok() {}
        let mut buf = Vec::new();
        ble_write(b"ATMA\r");
        let end = Instant::now() + Duration::from_millis(ms);
        loop {
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            match nrx.recv_timeout(left) {
                Ok(n) => buf.extend_from_slice(&n),
                Err(_) => break,
            }
        }
        ble_write(b"\r"); // any character stops monitoring
        let end = Instant::now() + Duration::from_millis(2500);
        while !buf.ends_with(b">") {
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            match nrx.recv_timeout(left) {
                Ok(n) => buf.extend_from_slice(&n),
                Err(_) => break,
            }
        }
        let text = String::from_utf8_lossy(&buf).to_string();
        let overflow = text.contains("BUFFER FULL");
        let frames = parse_monitor(&text);
        Ev::Frames { filt, mask, frames, overflow, silent }
    }

    /// VIN: service 09 02, else UDS DID F190 (same order as desktop).
    fn read_vin(header: &mut u16, nrx: &Receiver<Vec<u8>>) -> Option<String> {
        set_header(0x7DF, header, nrx);
        let r = cmd("0902", nrx, 3000);
        if let Some(v) = vin_from_0902(&messages(&r)) {
            return Some(v);
        }
        if !is_can() {
            return None;
        }
        for (hdr, _) in [ECUS[0], ECUS[7], ECUS[2]] {
            set_header(hdr, header, nrx);
            let r = cmd("22F190", nrx, 2000);
            if let Some(m) = messages(&r).into_iter().find(|m| m.len() > 3 && m[0] == 0x62 && m[1] == 0xF1 && m[2] == 0x90) {
                if let Some(v) = vin_chars(&m[3..]) {
                    return Some(v);
                }
            }
        }
        None
    }

    /// Which live values does this car answer? Standard mode-01 PIDs it declares and really
    /// answers, plus the BYD battery-module values when 7E7 replies (same as desktop).
    fn discover(header: &mut u16, nrx: &Receiver<Vec<u8>>) -> Vec<car::Pid> {
        let mut supported = Vec::new();
        set_header(0x7DF, header, nrx);
        for base in [0x00u8, 0x20, 0x40] {
            let r = cmd(&format!("01{base:02X}"), nrx, 2000);
            let Some(m) = messages(&r).into_iter().find(|m| m.len() >= 6 && m[0] == 0x41 && m[1] == base) else { break };
            let bits = u32::from_be_bytes([m[2], m[3], m[4], m[5]]);
            for i in 0..32u8 {
                if bits & (1 << (31 - i)) != 0 {
                    supported.push(base + i + 1);
                }
            }
            if bits & 1 == 0 {
                break;
            }
        }
        let mut ok: Vec<car::Pid> = car::pids()
            .into_iter()
            .filter(|p| supported.contains(&p.pid))
            .filter(|p| read_one(p.pid, &p.src, header, nrx).is_some())
            .collect();
        if is_can() && read_one(0, &Src::Did { hdr: 0x7E7, did: 0x1FFC }, header, nrx).is_some() {
            ok.extend(byd_pids());
            ok.extend(byd_bus_pids());
        }
        if parse_volt(&cmd("ATRV", nrx, 1000)).is_some() {
            ok.extend(adapter_pids());
        }
        ok
    }

    /// One background round: read the charge values and append a recording row.
    fn bg_round(header: &mut u16, nrx: &Receiver<Vec<u8>>, _tx: &Sender<Ev>, csv: Option<&str>, req: &[(u8, Src)]) {
        let v: Vec<f64> = req.iter().map(|(pid, src)| read_one(*pid, src, header, nrx).map(|x| x.0).unwrap_or(f64::NAN)).collect();
        if let Some(path) = csv {
            let row = crate::app::charge_csv_row(&crate::app::clock_text(), None, &v, "", "");
            if let Ok(mut fh) = std::fs::OpenOptions::new().append(true).open(path) {
                use std::io::Write;
                let _ = writeln!(fh, "{row}");
            }
        }
        // OBC state is the 17th value of charge_req() (index 16); 2 = charging
        let state = v.get(16).copied().unwrap_or(f64::NAN);
        log::info!("[bg] round: {} values, state {state}", v.iter().filter(|x| x.is_finite()).count());
    }

    pub fn worker(rx: Receiver<Cmd>, tx: Sender<Ev>) {
        let _ = EV_TX.set(tx.clone());
        let (ntx, nrx) = channel::<Vec<u8>>();
        let _ = NOTE_TX.set(ntx);
        let (ctx_, crx) = channel::<Result<String, String>>();
        let _ = CONN_TX.set(ctx_);
        let mut header = 0u16;

        // background monitor (see Cmd::BgMonitor)
        let mut bg: Option<(Option<String>, Vec<(u8, Src)>)> = None;
        let mut last_ui = Instant::now();
        let mut last_bg = Instant::now();
        loop {
            let c = match rx.recv_timeout(Duration::from_millis(1000)) {
                Ok(c) => c,
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    if let Some((csv, req)) = &bg {
                        if last_ui.elapsed() > Duration::from_secs(4) && last_bg.elapsed() >= Duration::from_secs(2) {
                            last_bg = Instant::now();
                            normal(&mut header, &nrx);
                            bg_round(&mut header, &nrx, &tx, csv.as_deref(), req);
                        }
                    }
                    continue;
                }
                Err(_) => break,
            };
            if let Cmd::BgMonitor { on, csv, req } = c {
                if on && bg.is_none() {
                    call_void("startMonitor");
                } else if !on && bg.is_some() {
                    call_void("stopMonitor");
                }
                bg = on.then_some((csv, req));
                continue;
            }
            last_ui = Instant::now();
            // leave bus-monitor mode before any request to the car (not before link set-up)
            if !matches!(c, Cmd::Sniff { .. } | Cmd::ScanDevices | Cmd::Connect | Cmd::ConnectTo(_) | Cmd::Disconnect) {
                normal(&mut header, &nrx);
            }
            match c {
                Cmd::BgMonitor { .. } => {} // handled above
                Cmd::Disconnect => {
                    if bg.take().is_some() {
                        call_void("stopMonitor");
                    }
                    *MON.lock().unwrap() = None;
                    call_void("disconnect");
                    let _ = tx.send(Ev::State(LinkState::Off));
                }
                Cmd::ScanDevices => {
                    log::info!("[ble] Cmd::ScanDevices -> call startScan");
                    SCAN.lock().unwrap().clear();
                    call_void("startScan"); // nativeOnScanDone sends Ev::Devices
                }
                Cmd::Connect | Cmd::ConnectTo(_) => {
                    let id = if let Cmd::ConnectTo(id) = &c {
                        Some(id.clone())
                    } else {
                        // auto: scan briefly, take the first OBD-looking device
                        SCAN.lock().unwrap().clear();
                        call_void("startScan");
                        std::thread::sleep(Duration::from_millis(6500));
                        let g = SCAN.lock().unwrap();
                        g.iter().find(|d| d.obd).or_else(|| g.first()).map(|d| d.id.clone())
                    };
                    let Some(id) = id else {
                        let _ = tx.send(Ev::State(LinkState::Failed("no OBD device found".into())));
                        continue;
                    };
                    while crx.try_recv().is_ok() {}
                    while nrx.try_recv().is_ok() {}
                    ble_connect(&id);
                    match crx.recv_timeout(Duration::from_secs(15)) {
                        Ok(Ok(info)) => {
                            header = 0;
                            *MON.lock().unwrap() = None; // fresh adapter: no monitor set-up
                            elm_init(&nrx);
                            let _ = tx.send(Ev::State(LinkState::Connected(info)));
                        }
                        Ok(Err(e)) => {
                            let _ = tx.send(Ev::State(LinkState::Failed(e)));
                        }
                        Err(_) => {
                            let _ = tx.send(Ev::State(LinkState::Failed("connect timeout".into())));
                        }
                    }
                }
                Cmd::Live(list) => {
                    let vals: Vec<Option<(f64, [u8; 2])>> =
                        list.iter().map(|(pid, src)| read_one(*pid, src, &mut header, &nrx)).collect();
                    let _ = tx.send(Ev::Live(vals));
                }
                Cmd::ReadDtc => {
                    let _ = tx.send(Ev::Dtcs(read_dtc(&mut header, &nrx)));
                }
                Cmd::ClearDtc => {
                    let _ = tx.send(Ev::Cleared(clear_dtc(&mut header, &nrx)));
                }
                Cmd::Sniff { filt, mask, ms } => {
                    let _ = tx.send(sniff(filt, mask, ms, &nrx));
                }
                Cmd::Discover => {
                    let _ = tx.send(Ev::Discovered(discover(&mut header, &nrx)));
                }
                Cmd::ReadVin => {
                    let _ = tx.send(Ev::Vin(read_vin(&mut header, &nrx)));
                }
                Cmd::ReadDids { hdr, dids, ext_dids } => {
                    let read = |did: u16, header: &mut u16| -> Option<Vec<u8>> {
                        set_header(hdr, header, &nrx);
                        let r = cmd(&format!("22{did:04X}"), &nrx, 1500);
                        messages(&r)
                            .into_iter()
                            .find(|m| m.len() > 3 && m[0] == 0x62 && m[1] == (did >> 8) as u8 && m[2] == did as u8)
                            .map(|m| m[3..].to_vec())
                    };
                    let mut out = Vec::new();
                    for did in dids {
                        if let Some(d) = read(did, &mut header) {
                            out.push((did, d));
                        }
                    }
                    if !ext_dids.is_empty() {
                        set_header(hdr, &mut header, &nrx);
                        cmd("1003", &nrx, 1500);
                        for did in ext_dids {
                            if let Some(d) = read(did, &mut header) {
                                out.push((did, d));
                            }
                        }
                        cmd("1001", &nrx, 1500);
                    }
                    let _ = tx.send(Ev::Dids(out));
                }
                Cmd::DidScan { hdr, ranges, extended } => {
                    let total: usize = ranges.iter().map(|(a, b)| (*b as usize).saturating_sub(*a as usize) + 1).sum();
                    let mut found = Vec::new();
                    let mut done = 0;
                    set_header(hdr, &mut header, &nrx);
                    if extended {
                        let r = cmd("1003", &nrx, 1500);
                        log::info!("[didscan] 10 03 -> {r}");
                    }
                    for (a, b) in ranges {
                        for did in a..=b {
                            let r = cmd(&format!("22{did:04X}"), &nrx, 1200);
                            if let Some(m) = messages(&r).into_iter().find(|m| m.len() > 3 && m[0] == 0x62 && m[1] == (did >> 8) as u8 && m[2] == did as u8) {
                                found.push((did, m[3..].to_vec()));
                            }
                            done += 1;
                            if done % 16 == 0 {
                                let _ = tx.send(Ev::DidScanProgress { done, total, found: found.len() });
                            }
                        }
                    }
                    if extended {
                        // requests kept the session alive; hand it back explicitly
                        let r = cmd("1001", &nrx, 1500);
                        log::info!("[didscan] 10 01 -> {r}");
                    }
                    let _ = tx.send(Ev::DidScanDone(found));
                }
            }
        }
    }

    // ---- callbacks from Kotlin (Ble.nativeOn*) ----
    /// Called once from Ble.init() on the Java main thread: cache the Ble class so the worker
    /// thread can call its static methods (startScan/connect/write) without FindClass.
    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeInit(env: JNIEnv, thiz: JObject) {
        if let Ok(cls) = env.get_object_class(&thiz) {
            if let Ok(g) = env.new_global_ref(&cls) {
                let _ = BLE_CLASS.set(g);
            }
        }
        log::info!("[ble] nativeInit: class cached");
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeOnDevice(mut env: JNIEnv, _o: JObject, id: JString, name: JString, is_obd: jboolean) {
        let id: String = env.get_string(&id).map(|s| s.into()).unwrap_or_default();
        let name: String = env.get_string(&name).map(|s| s.into()).unwrap_or_default();
        if id.is_empty() {
            return;
        }
        let mut v = SCAN.lock().unwrap();
        if let Some(d) = v.iter_mut().find(|d| d.id == id) {
            if (d.name == "(unnamed)" || d.name.is_empty()) && !name.is_empty() {
                d.name = name;
            }
            if is_obd != 0 {
                d.obd = true;
            }
        } else {
            v.push(BtDevice { id, name: if name.is_empty() { "(unnamed)".into() } else { name }, obd: is_obd != 0 });
        }
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeOnScanDone(_env: JNIEnv, _o: JObject) {
        let mut v = SCAN.lock().unwrap().clone();
        v.sort_by(|a, b| b.obd.cmp(&a.obd).then_with(|| a.name.cmp(&b.name)));
        log::info!("[ble] nativeOnScanDone -> {} device(s)", v.len());
        send_ev(Ev::Devices(v));
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeOnConnected(mut env: JNIEnv, _o: JObject, info: JString) {
        let info: String = env.get_string(&info).map(|s| s.into()).unwrap_or_default();
        if let Some(tx) = CONN_TX.get() {
            let _ = tx.send(Ok(info));
        }
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeOnDisconnected(mut env: JNIEnv, _o: JObject, reason: JString) {
        let reason: String = env.get_string(&reason).map(|s| s.into()).unwrap_or_default();
        *MON.lock().unwrap() = None;
        if let Some(tx) = CONN_TX.get() {
            let _ = tx.send(Err(reason.clone()));
        }
        send_ev(Ev::State(LinkState::Failed(reason)));
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_Ble_nativeOnNotify(env: JNIEnv, _o: JObject, data: JByteArray) {
        if let Ok(bytes) = env.convert_byte_array(&data) {
            if let Some(tx) = NOTE_TX.get() {
                let _ = tx.send(bytes);
            }
        }
    }
}

#[cfg(target_os = "android")]
fn worker(rx: Receiver<Cmd>, tx: Sender<Ev>) {
    android_ble::worker(rx, tx);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn monitor_lines() {
        let f = parse_monitor("323 03 00 7F 12 00 00 00 00\r055 80 01\rBUFFER FULL\r7E8 03 41 0D\rSTOPPED\r\r>");
        assert_eq!(f.len(), 3);
        assert_eq!(f[0].id, 0x323);
        assert_eq!(f[0].data, vec![0x03, 0, 0x7F, 0x12, 0, 0, 0, 0]);
        assert_eq!(f[1].data, vec![0x80, 0x01]);
    }
    #[test]
    fn multi_frame_vin() {
        let r = "014\n0:490201575657\n1:5A5A5A314B5A36\n2:57303030313233";
        let m = messages(r);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].len(), 0x14);
        assert_eq!(vin_from_0902(&m).as_deref(), Some("WVWZZZ1KZ6W000123"));
    }
    #[test]
    fn single_and_errors() {
        assert_eq!(messages("SEARCHING...\n410C1AF8"), vec![vec![0x41, 0x0C, 0x1A, 0xF8]]);
        assert!(messages("NO DATA").is_empty());
        assert_eq!(messages("7F0922"), vec![vec![0x7F, 0x09, 0x22]]);
    }
    #[test]
    fn byd_soc() {
        // 62 1F FC 14 1E → 0x1E14 = 7700 → 77.00 %
        let m = messages("621FFC141E");
        assert_eq!(decode_did(0x7E7, 0x1FFC, &m[0][3..]), Some(77.0));
        assert_eq!(decode_did(0x7E7, 0x0009, &[0x88, 0x13]), Some(0.0));
        // OBC (BYD formulas): 0x0013 = A-40, 0x0012 = LE x 0.2 W, 0x0007 = A x 0.2
        assert_eq!(decode_did(OBC, 0x0013, &[0x4B]), Some(35.0));
        assert_eq!(decode_did(OBC, 0x0012, &[0x10, 0x27]), Some(2.0));
        assert_eq!(decode_did(OBC, 0x0007, &[0x96]), Some(30.0));
    }
}

#[cfg(test)]
mod dtc_table_tests {
    use super::*;
    #[test]
    fn byd_table() {
        // The shipped module table is a stub in the public repo, so only assert on parsing
        // logic, not on specific rows. (Private builds ship the full table.)
        let _ = byd_modules();
        // unknown code with no matching table entry -> no text
        assert_eq!(byd_dtc_text(0x747, "P0000", None), None);
        // 59 02 FF + one record (P1667 12, status 09 = current)
        let d = uds_codes(&[0x59, 0x02, 0xFF, 0x16, 0x67, 0x12, 0x09]);
        assert_eq!(d.len(), 1);
        assert_eq!((d[0].a, d[0].b, d[0].c), (0x16, 0x67, Some(0x12)));
        assert!(matches!(d[0].status, car::DtcStatus::Current));
    }
}
