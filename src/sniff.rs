//! Bus Monitor: listen to the car's CAN bus while it charges (read only) and find the frames
//! whose bytes change when something happens — charging starts, the pack crosses 39 °C
//! (the BMS should start asking for coolant), or the user presses "Mark".
//!
//! The ELM327 can only listen to part of the bus at a time over BLE without overflowing,
//! so IDs are split into 8 groups of 0x100 (mask 0x700) and visited in turn.

use crate::obd::Frame;
use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;

pub const GROUPS: usize = 8;
pub const MASK: u16 = 0x700;
pub const LISTEN_MS: u64 = 4000;
/// Listening to one ID at a time keeps the ELM327 buffer from filling (a wide filter gives
/// only ~9 frames before BUFFER FULL on the BYD bus, as seen 2026-11-08).
pub const ONE_ID_MS: u64 = 1500;

/// What one listen window asks for.
pub enum Next {
    Group(usize),
    Id(u16),
}

/// One named signal from the CAN signal table. Only the start bit is known, not the length.
pub struct Sig {
    pub bit: u16,
    pub page: u8,
    pub name: &'static str,
}

impl Sig {
    /// (data byte, bit in byte) — code bit 8 is data byte 0 bit 0 (checked on 0x404 / 0x242).
    pub fn pos(&self) -> Option<(usize, u16)> {
        (self.bit >= 8).then(|| (((self.bit - 8) / 8) as usize, (self.bit - 8) % 8))
    }
}

static TABLE: &str = include_str!("byd_signals.txt");

pub fn signals(id: u16) -> &'static [Sig] {
    use std::sync::OnceLock;
    static MAP: OnceLock<BTreeMap<u16, Vec<Sig>>> = OnceLock::new();
    let map = MAP.get_or_init(|| {
        let mut m: BTreeMap<u16, Vec<Sig>> = BTreeMap::new();
        for line in TABLE.lines().filter(|l| !l.starts_with('#')) {
            let mut it = line.splitn(4, ' ');
            let (Some(f), Some(b), Some(p), Some(n)) = (it.next(), it.next(), it.next(), it.next()) else { continue };
            let (Ok(f), Ok(b), Ok(p)) = (u16::from_str_radix(f, 16), b.parse(), p.parse()) else { continue };
            m.entry(f).or_default().push(Sig { bit: b, page: p, name: n });
        }
        m
    });
    map.get(&id).map(|v| v.as_slice()).unwrap_or(&[])
}

/// Short label for an ID row: the most telling names (not the ...ONLINE... presence flags).
pub fn signal_summary(id: u16) -> (String, String) {
    let sigs: Vec<&Sig> = signals(id).iter().filter(|s| !s.name.contains("ONLINE")).collect();
    let all: Vec<String> = signals(id)
        .iter()
        .map(|s| match s.pos() {
            Some((b, k)) => format!("byte {b} bit {k}{}: {}", if s.page > 0 { format!(" (page {})", s.page) } else { String::new() }, s.name),
            None => format!("bit {}: {}", s.bit, s.name),
        })
        .collect();
    let short = match sigs.len() {
        0 => String::new(),
        1 => sigs[0].name.to_string(),
        n => format!("{} (+{})", sigs[0].name, n - 1),
    };
    (short, all.join("\n"))
}

/// BMS thermal-management request (44A byte 6) and pack current from the bus (44A bytes 3-4).
pub fn decode_44a(d: &[u8]) -> Option<(u8, f64)> {
    let b6 = *d.get(6)?;
    let raw = *d.get(3)? as f64 + *d.get(4)? as f64 * 256.0;
    Some((b6, raw * 0.1 - 500.0))
}

const SPEED_SCALE: f64 = 0.06875; // vehicle-speed scale

/// Vehicle speed from 0x121 (km/h); None if raw is the invalid value (> 4094).
pub fn auto_speed(d: &[u8]) -> Option<f64> {
    let raw = *d.first()? as u16 | ((*d.get(1)? as u16 & 0x0F) << 8);
    (raw <= 4094).then(|| raw as f64 * SPEED_SCALE)
}

/// Wheel speeds FL, FR, RL, RR from 0x122 (km/h).
pub fn wheel_speeds(d: &[u8]) -> Option<[f64; 4]> {
    let mut w = [0.0; 4];
    for (i, v) in w.iter_mut().enumerate() {
        let raw = (*d.get(i * 2)? as u16 | ((*d.get(i * 2 + 1)? as u16) << 8)) & 0x1FFF;
        *v = raw as f64 * SPEED_SCALE;
    }
    Some(w)
}

/// Numeric value of a Live Data bus item (`obd::byd_bus_pids`) from one frame of its ID.
/// Gear: 1 P, 2 R, 3 N, 4 D. Handbrake: 1 = on.
pub fn live_bus_value(name: &str, id: u16, d: &[u8]) -> Option<f64> {
    Some(match (id, name) {
        (0x242, _) => (*d.get(5)? & 0x0F) as f64,
        (0x218, _) => if *d.get(5)? & 0x10 == 0 { 1.0 } else { 0.0 },
        (0x404, "A/C set temperature") => *d.get(4)? as f64,
        (0x404, "A/C fan level") => (*d.get(2)? >> 4) as f64,
        (0x44A, _) => (decode_44a(d)?.0 >> 4) as f64,
        (0x121, _) => auto_speed(d)?,
        (0x122, n) => {
            let w = wheel_speeds(d)?;
            match n {
                "Wheel speed FL" => w[0],
                "Wheel speed FR" => w[1],
                "Wheel speed RL" => w[2],
                _ => w[3],
            }
        }
        _ => return None,
    })
}

/// Decoded value for IDs whose meaning OVMS documents.
pub fn decode_known(id: u16, d: &[u8]) -> Option<String> {
    Some(match id {
        0x055 => match d.get(4)? {
            0x80 => "car OFF".into(),
            0x82 => "car ON".into(),
            v => format!("on/off {v:02X}"),
        },
        0x151 => format!("{} km/h", d.first()?),
        // DiLink OTA: SPEED_AUTO_SPEED_121 = 12 bits (b0 + low nibble b1), autospeed km/h = raw × 0.06875
        0x121 => format!("{:.1} km/h", auto_speed(d)?),
        // DiLink OTA: SPECIAL_CMD_WHEEL_SPEED_FL/FR/RL/RR, 2 bytes LE each, same scale as 0x121
        0x122 => {
            let w = wheel_speeds(d)?;
            format!("wheels {:.0}/{:.0}/{:.0}/{:.0} km/h", w[0], w[1], w[2], w[3])
        }
        // DiLink OTA: BODYWORK_STEERING_WHEEL_ANGEL int16 LE × 0.1° (steeringangle converter)
        0x11F => format!("steering {:+.1}°", i16::from_le_bytes([*d.first()?, *d.get(1)?]) as f64 * 0.1),
        0x218 => (if d.get(5)? & 0x10 == 0 { "handbrake ON" } else { "handbrake off" }).into(),
        0x242 => match d.get(5)? & 0x0F {
            1 => "gear P".into(),
            2 => "gear R".into(),
            3 => "gear N".into(),
            4 => "gear D".into(),
            v => format!("gear ? ({v:X})"),
        },
        // found by the A/C tests on 2026-11-08 (not in OVMS): b4/b5 = set temperature L/R in °C,
        // b2 high nibble = fan level (0 = off)
        0x404 => format!("A/C set {}/{} °C, fan {}", d.get(4)?, d.get(5)?, d.get(2)? >> 4),
        // BYD table: 44A bit 32 STATISTIC_INSTANTANEOUS_CURRENT (A = raw×0.1−500, + = discharge),
        // bit 60 CHARGING_THERMAL_MANAGEMENT_REQUEST_STATUS
        0x44A => {
            let (tm, a) = decode_44a(d)?;
            format!("current {a:+.1} A, thermal req {tm:02X}")
        }
        0x323 => match d.first()? {
            0 => "not charging".into(),
            3 => "CHARGING".into(),
            7 => "waiting for timer".into(),
            v => format!("charge state {v:02X}"),
        },
        _ => return None,
    })
}

/// Meaning known from OVMS's BYD Atto 3 module (vehicle_byd_atto3.cpp).
pub fn known(id: u16) -> &'static str {
    match id {
        0x055 => "car on/off: byte4 80=off 82=on (OVMS)",
        0x404 => "A/C panel: b4/b5 set °C, b2 fan (our test)",
        0x44A => "b3-4 pack current, b6 BMS thermal request (BYD table)",
        0x151 => "speed km/h: byte0 (OVMS)",
        0x121 => "vehicle speed: 12 bits ×0.06875 km/h (DiLink OTA)",
        0x122 => "wheel speeds FL/FR/RL/RR ×0.06875 km/h (DiLink OTA)",
        0x11F => "steering angle int16 ×0.1° (DiLink OTA)",
        0x218 => "handbrake: byte5 bit4 (OVMS)",
        0x242 => "gear: byte5 low 4 bits 1=P 2=R 3=N 4=D (OVMS)",
        0x294 => "doors: byte0 bits (OVMS)",
        0x323 => "charge state: byte0 0=no 3=charging 7=wait (OVMS)",
        0x3D9 => "range / odometer (OVMS)",
        _ => "",
    }
}
/// A change counts as "at" an event when it is this close (seconds). One group is heard
/// for 4 s about every 40 s, so time resolution is coarse.
pub const EVENT_WINDOW_S: f64 = 60.0;

pub struct IdStat {
    pub count: usize,
    pub last: Vec<u8>,
    /// per byte: how many times it changed
    pub changes_per_byte: [u32; 8],
    /// (time, byte index, old, new) — newest 2000
    pub changes: Vec<(f64, usize, u8, u8)>,
}

#[derive(Default)]
pub struct SniffState {
    pub on: bool,
    pub group: usize,
    pub pass: usize,
    pub counts: [Option<usize>; GROUPS],
    pub poll_next: bool,
    pub ids: BTreeMap<u16, IdStat>,
    pub total: usize,
    pub overflows: usize,
    pub silent: Option<bool>,
    pub file: Option<PathBuf>,
    pub marks: Vec<f64>,
    /// listen to only this ID, over and over
    pub focus: Option<u16>,
    /// position in the one-ID round
    pub id_pos: usize,
    /// listen windows so far (battery is read every 4th)
    pub windows: usize,
    /// Bluetooth dropped while listening: reconnect and carry on in the same file
    pub resume: bool,
    pub retry_at: f64,
    pub reconnects: usize,
}

pub struct Event {
    pub name: String,
    pub t: f64,
}

pub struct Candidate {
    pub id: u16,
    pub byte: usize,
    pub old: u8,
    pub new: u8,
    pub dt: f64,
    pub total_changes: u32,
}

impl SniffState {
    /// Plan: focus ID if set; else one discovery pass over the 8 groups, then one ID at a
    /// time, with one group window after each full ID round to catch new IDs.
    pub fn next(&mut self) -> Next {
        if let Some(id) = self.focus {
            return Next::Id(id);
        }
        if self.pass == 0 || self.ids.is_empty() {
            return Next::Group(self.next_group());
        }
        let ids: Vec<u16> = self.ids.keys().copied().collect();
        if self.id_pos >= ids.len() {
            self.id_pos = 0;
            return Next::Group(self.next_group());
        }
        self.id_pos += 1;
        Next::Id(ids[self.id_pos - 1])
    }

    /// Next group to listen to, in turn. Quiet groups are not skipped: on 2026-11-08 the
    /// 0xx/1xx groups were silent in one window and busy in another.
    pub fn next_group(&mut self) -> usize {
        let g = self.group;
        self.group = (self.group + 1) % GROUPS;
        if self.group == 0 {
            self.pass += 1;
        }
        g
    }

    pub fn ingest(&mut self, t: f64, filt: u16, mask: u16, frames: &[Frame], temp: f64, kw: f64) {
        if mask == MASK {
            self.counts[(filt >> 8) as usize % GROUPS] = Some(frames.len());
        }
        self.total += frames.len();
        let mut lines = String::new();
        for f in frames {
            let hex: Vec<String> = f.data.iter().map(|b| format!("{b:02X}")).collect();
            lines += &format!("{t:.1},{:03X},{},{temp:.0},{kw:.1}\n", f.id, hex.join(" "));
            let st = self.ids.entry(f.id).or_insert_with(|| IdStat { count: 0, last: f.data.clone(), changes_per_byte: [0; 8], changes: Vec::new() });
            st.count += 1;
            for (i, (&o, &n)) in st.last.iter().zip(f.data.iter()).enumerate() {
                if o != n {
                    st.changes_per_byte[i] += 1;
                    if st.changes.len() >= 2000 {
                        st.changes.remove(0);
                    }
                    st.changes.push((t, i, o, n));
                }
            }
            st.last = f.data.clone();
        }
        if let Some(p) = &self.file {
            if let Ok(mut fh) = std::fs::OpenOptions::new().append(true).open(p) {
                let _ = fh.write_all(lines.as_bytes());
            }
        }
    }

    pub fn write_mark(&self, t: f64, n: usize) {
        self.write_event(t, "MARK", &n.to_string());
    }

    /// A non-frame line in the CSV: `t,KIND,info,,` (MARK, LOST, RECONNECT).
    pub fn write_event(&self, t: f64, kind: &str, info: &str) {
        if let Some(p) = &self.file {
            if let Ok(mut fh) = std::fs::OpenOptions::new().append(true).open(p) {
                let _ = writeln!(fh, "{t:.1},{kind},{info},,");
            }
        }
    }

    pub fn open_file(&mut self, dir: PathBuf, stamp: &str) -> Result<PathBuf, String> {
        std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
        let p = dir.join(format!("bus_{stamp}.csv"));
        std::fs::write(&p, "t_s,id,data,batt_temp_c,charge_kw\n").map_err(|e| e.to_string())?;
        self.file = Some(p.clone());
        Ok(p)
    }

    /// Bytes that changed close to an event, rarest-changing first (a status flag changes
    /// rarely; a counter or a measured value changes all the time).
    pub fn candidates(&self, ev_t: f64) -> Vec<Candidate> {
        let mut out = Vec::new();
        for (&id, st) in &self.ids {
            for &(t, i, o, n) in &st.changes {
                if (t - ev_t).abs() <= EVENT_WINDOW_S && st.changes_per_byte[i] <= 6 {
                    out.push(Candidate { id, byte: i, old: o, new: n, dt: t - ev_t, total_changes: st.changes_per_byte[i] });
                }
            }
        }
        out.sort_by(|a, b| a.total_changes.cmp(&b.total_changes).then(a.dt.abs().total_cmp(&b.dt.abs())));
        out.truncate(15);
        out
    }
}

/// Events found in the temperature / power timeline: (t, temp, kw).
pub fn events(timeline: &[(f64, f64, f64)], marks: &[f64]) -> Vec<Event> {
    let mut out = Vec::new();
    for w in timeline.windows(2) {
        let (a, b) = (w[0], w[1]);
        if a.2.abs() < 1.0 && b.2.abs() >= 1.0 && !out.iter().any(|e: &Event| e.name.starts_with("Charging")) {
            out.push(Event { name: "Charging starts".into(), t: b.0 });
        }
        if a.1 < 39.0 && b.1 >= 39.0 && !out.iter().any(|e: &Event| e.name.starts_with("Battery")) {
            out.push(Event { name: "Battery reaches 39 °C".into(), t: b.0 });
        }
    }
    for (i, &m) in marks.iter().enumerate() {
        out.push(Event { name: format!("Mark {}", i + 1), t: m });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn flag_found_at_event() {
        let mut s = SniffState::default();
        let f = |d: Vec<u8>| vec![Frame { id: 0x3A2, data: d }, Frame { id: 0x100, data: vec![0] }];
        s.ingest(0.0, 0x300, MASK, &f(vec![0, 0, 0, 0]), 30.0, 80.0);
        s.ingest(40.0, 0x300, MASK, &f(vec![0, 0, 0, 0]), 38.0, 80.0);
        s.ingest(80.0, 0x300, MASK, &f(vec![0, 0, 0x20, 0]), 39.5, 80.0);
        let ev = events(&[(0.0, 30.0, 0.0), (10.0, 30.0, 80.0), (75.0, 39.2, 80.0)], &[]);
        assert_eq!(ev.len(), 2);
        let c = s.candidates(ev[1].t);
        assert_eq!((c[0].id, c[0].byte, c[0].old, c[0].new), (0x3A2, 2, 0x00, 0x20));
    }
    #[test]
    fn plan_goes_group_pass_then_ids() {
        let mut s = SniffState::default();
        for g in 0..GROUPS {
            assert!(matches!(s.next(), Next::Group(x) if x == g));
        }
        s.ids.insert(0x242, IdStat { count: 1, last: vec![0], changes_per_byte: [0; 8], changes: vec![] });
        assert!(matches!(s.next(), Next::Id(0x242)));
        assert!(matches!(s.next(), Next::Group(_)));
        s.focus = Some(0x123);
        assert!(matches!(s.next(), Next::Id(0x123)));
    }
    #[test]
    fn byd_table() {
        // The shipped signal table is a stub in the public repo; assert on the decoder, not
        // on table rows. (Private builds ship the full table.)
        let (tm, a) = decode_44a(&[0x58, 0xDF, 0x41, 0x97, 0x13, 0x37, 0x14, 0x92]).unwrap();
        assert_eq!(tm, 0x14);
        assert!((a - 1.5).abs() < 0.01);
    }
    #[test]
    fn live_bus_values() {
        // same frames as known_values below
        assert_eq!(live_bus_value("Gear", 0x242, &[0x30, 0x75, 0x5E, 0, 0x2C, 0x84, 0xFF, 0x4D]), Some(4.0));
        assert_eq!(live_bus_value("Handbrake", 0x218, &[8, 6, 0, 0, 0x18, 0x6D, 0x30, 0x3C]), Some(1.0));
        let ac = [0x53, 0x65, 0x71, 0x7F, 0x11, 0x11, 0x9F, 0x5D];
        assert_eq!(live_bus_value("A/C set temperature", 0x404, &ac), Some(17.0));
        assert_eq!(live_bus_value("A/C fan level", 0x404, &ac), Some(7.0));
        assert_eq!(live_bus_value("BMS thermal request", 0x44A, &[0, 0, 0, 0x88, 0x13, 0, 0x14, 0]), Some(1.0));
    }

    #[test]
    fn known_values() {
        assert_eq!(decode_known(0x242, &[0x30, 0x75, 0x5E, 0, 0x2C, 0x84, 0xFF, 0x4D]).as_deref(), Some("gear D"));
        assert_eq!(decode_known(0x242, &[0, 0, 0, 0, 0, 0xA1]).as_deref(), Some("gear P"));
        assert_eq!(decode_known(0x404, &[0x53, 0x65, 0x71, 0x7F, 0x11, 0x11, 0x9F, 0x5D]).as_deref(), Some("A/C set 17/17 °C, fan 7"));
        assert_eq!(decode_known(0x121, &[0xB2, 0x00, 0, 0x90, 0xE5, 0xFD, 0x9B, 0x40]).as_deref(), Some("12.2 km/h"));
        assert_eq!(decode_known(0x122, &[0x19, 0, 0x16, 0, 0x19, 0, 0x15, 0x40]).as_deref(), Some("wheels 2/2/2/1 km/h"));
        assert_eq!(decode_known(0x11F, &[0x9C, 0xFF, 0, 0, 0, 0, 0, 0]).as_deref(), Some("steering -10.0°"));
        assert_eq!(decode_known(0x218, &[8, 6, 0, 0, 0x18, 0x6D, 0x30, 0x3C]).as_deref(), Some("handbrake ON"));
    }
}
