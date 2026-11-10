use crate::car::{self, Dtc, DtcStatus, Pid, System, Units};
use crate::obd::{self, Cmd, Ev, Link, LinkState, Src};
use crate::history::{self, Snap};
use crate::vin;
use eframe::egui::{
    self, Align, Align2, Color32, CornerRadius, FontId, Layout, Pos2, Rect, RichText, Sense, Stroke,
    StrokeKind, Vec2,
};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

// UI colour palette.
const HEAD_TOP: Color32 = Color32::from_rgb(0x59, 0xA7, 0xFF);
const HEAD_BOTTOM: Color32 = Color32::from_rgb(0x00, 0x66, 0xD9);
const ACCENT: Color32 = Color32::from_rgb(0x07, 0x70, 0xEB);
const GREY_TOP: Color32 = Color32::from_rgb(0xCE, 0xD6, 0xDE);
const TEXT_DARK: Color32 = Color32::from_rgb(0x22, 0x2B, 0x3A);
const RED: Color32 = Color32::from_rgb(0xE5, 0x3E, 0x3E);
const GREEN: Color32 = Color32::from_rgb(0x11, 0xA8, 0x46);

/// System status labels and Auto Scan prompts.

/// Live data display limits.
const MAX_SELECTED: usize = 8;
const MAX_GRAPH: usize = 5;
const HISTORY: usize = 120; // estimated

const GRAPH_COLORS: [Color32; 5] = [
    Color32::from_rgb(0x07, 0x70, 0xEB),
    Color32::from_rgb(0xF6, 0x3B, 0x67),
    Color32::from_rgb(0x11, 0xA8, 0x46),
    Color32::from_rgb(0xFF, 0x85, 0x2F),
    Color32::from_rgb(0x77, 0x41, 0xEC),
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Lang {
    En,
    Th,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Screen {
    Connect,
    VinInput,
    Menu,
    Codes,
    LiveData,
    Reports,
    Settings,
    Charge,
    Battery,
    Sniff,
    /// "Full vehicle check": one-tap read-only health report
    Check,
}

/// BYD battery-module values the Battery Pack screen reads on a real car (7E7, UDS 0x22):
/// SOC, pack voltage, battery temp, coolant/system temp, cell voltage min, cell voltage max,
/// SOH, charge cycles. Per-cell voltages are NOT exposed on this OBD path (BMS gives min/max
/// only), so the real screen never fabricates a per-cell grid.
/// Battery screen, every round (BMS 7E7, YUAN_PLUSEV_BASU definitions): SOC, pack V,
/// average temp, MAX temp, min / max cell mV, SOH, charges, MIN temp, lowest / highest cell
/// number, min / max temperature sensor number.
const BATT_DIDS: [u16; 13] = [0x1FFC, 0x0008, 0x0032, 0x0031, 0x002B, 0x002D, 0x0029, 0x000B, 0x002F, 0x002A, 0x002C, 0x002E, 0x0030];

/// Slow-changing BMS details read once per visit (and every 5 min): capacity now / factory,
/// limits, lifetime counters, balancing hours of cells 1-126; plus, in an extended session,
/// the per-cell voltage-sampling and temperature-sensor status bits.
fn batt_detail_req() -> Cmd {
    let mut dids = vec![0x1FFC, 0x1FFE, 0x000A, 0x000E, 0x000F, 0x0010, 0x0011, 0x0012, 0x0015];
    dids.extend(0x0040..=0x00BD);
    Cmd::ReadDids { hdr: 0x7E7, dids, ext_dids: vec![0x1100, 0x1101, 0x1102, 0x1103, 0x110A, 0x110B] }
}

/// Decoded BattDetail (NaN / false = not answered).
#[derive(Clone)]
struct BattDetail {
    cap_now: f64,
    cap_factory: f64,
    max_chg_kw: f64,
    max_dis_kw: f64,
    chg_ah: f64,
    dis_ah: f64,
    chg_kwh: f64,
    dis_kwh: f64,
    insul_kohm: f64,
    /// accumulated balancing time per cell (h), cells 1-126
    balance_h: Vec<f64>,
    /// voltage-sampling status bit per cell (true = abnormal); None = not read
    cell_bad: Option<Vec<bool>>,
    /// abnormal temperature sensors (count) from 110A/110B; None = not read
    temp_bad: Option<usize>,
}

impl BattDetail {
    fn parse(list: &[(u16, Vec<u8>)]) -> Self {
        let get = |did: u16| list.iter().find(|(d, _)| *d == did).map(|(_, v)| v.as_slice());
        let le = |v: &[u8], i: usize| -> f64 {
            match (v.get(i), v.get(i + 1)) {
                (Some(a), Some(b)) => *a as f64 + *b as f64 * 256.0,
                _ => f64::NAN,
            }
        };
        let le16 = |did: u16, i: usize| get(did).map(|v| le(v, i)).unwrap_or(f64::NAN);
        let le24 = |did: u16| get(did).filter(|v| v.len() >= 3).map(|v| v[0] as f64 + v[1] as f64 * 256.0 + v[2] as f64 * 65536.0).unwrap_or(f64::NAN);
        let bits = |dids: &[u16], n: usize| -> Option<Vec<bool>> {
            let mut out = Vec::new();
            for d in dids {
                let v = get(*d)?;
                for byte in v.iter().take(4) {
                    for bit in 0..8 {
                        out.push(byte & (1 << bit) != 0);
                    }
                }
            }
            out.truncate(n);
            Some(out)
        };
        BattDetail {
            cap_now: le16(0x1FFC, 2) * 0.01,
            cap_factory: le16(0x1FFE, 2) * 0.01,
            max_chg_kw: le16(0x000A, 0) * 0.1,
            max_dis_kw: le16(0x000E, 0) * 0.1,
            chg_ah: le24(0x000F),
            dis_ah: le24(0x0010),
            chg_kwh: le16(0x0011, 0),
            dis_kwh: le16(0x0012, 0),
            insul_kohm: le16(0x0015, 0),
            balance_h: (0x0040u16..=0x00BD).map(|d| le16(d, 0)).collect(),
            cell_bad: bits(&[0x1100, 0x1101, 0x1102, 0x1103], 126),
            temp_bad: bits(&[0x110A, 0x110B], 64).map(|v| v.iter().filter(|b| **b).count()),
        }
    }
}

#[derive(Default, Clone, Copy)]
struct BattState {
    got: bool,
    timer: f64,
    soc: f64,
    volt: f64,
    /// average / max / min cell temperature (0032 / 0031 / 002F)
    temp: f64,
    tmax: f64,
    tmin: f64,
    cmin: f64,
    cmax: f64,
    soh: f64,
    cycles: f64,
    /// pack-wide lowest / highest cell number (002A / 002C) and min / max temperature sensor
    /// number (002E / 0030), 1-based
    vmin_no: f64,
    vmax_no: f64,
    tmin_no: f64,
    tmax_no: f64,
    /// per BIC (battery information collector) 1–10, in DID order:
    /// [min cell no, min cell mV, max cell no, max cell mV, min temp no, min °C, max temp no, max °C]
    bic: [[f64; 8]; 10],
    bic_got: [bool; 10],
    /// BIC group read in the outstanding request (one group per round)
    bic_next: usize,
}

/// First DID of the BMS per-BIC block; group g is BIC_BASE + 8·g … +7.
const BIC_BASE: u16 = 0x016C;


struct LiveState {
    pids: Vec<Pid>,
    selected: Vec<bool>,
    ranges: Vec<(f64, f64)>,
    values: Vec<f64>,
    raw: Vec<(u8, u8)>,
    history: Vec<VecDeque<f64>>,
    paused: bool,
    graph: bool,
    /// table view instead of the card dashboard (when not in graph view)
    table: bool,
    recording: Option<Vec<String>>,
    range_edit: Option<(usize, String, String, Option<String>)>,
}

/// Charge Monitor thresholds. Taken from the Facebook reel by a BYD owner (unverified):
/// the BMS calls for coolant at 39 °C and cuts DC charge power at 46 °C (84 → 60 → 45 kW).
const COOL_START_C: f64 = 39.0;
const DERATE_C: f64 = 46.0;
const WARN_C: f64 = 36.0;
/// BYD battery-module values the monitor reads (7E7, UDS 0x22), see obd::byd_pids.
const CHARGE_DIDS: [u16; 7] = [0x1FFC, 0x0008, 0x0009, 0x0032, 0x0031, 0x002B, 0x002D];
/// On-board charger values polled with the charge values (module 7E4, BYD's own DIDs):
/// internal temp 1-4, charging port temp, cooling fan %, AC voltage, AC current, AC power, state.
const OBC_DIDS: [u16; 10] = [0x0013, 0x0014, 0x0015, 0x0016, 0x0017, 0x0019, 0x0003, 0x0007, 0x0012, 0x001A];
/// Power Domain Controller (7E2) values with them: powertrain coolant temp, water pump %,
/// cooling fan high / low speed status (BYD's own DIDs).
const PDC_DIDS: [u16; 4] = [0x0019, 0x0021, 0x0014, 0x0015];

/// One charge-screen request: BMS values then OBC values (decoded per module in obd).
fn charge_req() -> Vec<(u8, Src)> {
    CHARGE_DIDS
        .iter()
        .map(|d| (0u8, Src::Did { hdr: 0x7E7, did: *d }))
        .chain(OBC_DIDS.iter().map(|d| (0u8, Src::Did { hdr: obd::OBC, did: *d })))
        .chain(PDC_DIDS.iter().map(|d| (0u8, Src::Did { hdr: obd::PDC, did: *d })))
        .collect()
}

/// Charge recording columns (one row per charge_req() reading).
pub(crate) const CHARGE_CSV_HEADER: &str = "time,t_s,soc,volt,amp,kw,batt_temp,batt_max_temp,cell_min_mv,cell_max_mv,thermal_req_44A_b6,bus_current_a,obc_t1,obc_t2,obc_t3,obc_t4,port_temp,obc_fan_pct,ac_v,ac_a,ac_kw,obc_state,pt_coolant_c,pdc_pump_pct,fan_hi,fan_lo";

/// One recording row from the raw charge_req() values `v` (BMS 7, OBC 10, PDC 4).
pub(crate) fn charge_csv_row(clock: &str, t: Option<f64>, v: &[f64], tm: &str, bus: &str) -> String {
    let g = |i: usize| v.get(i).copied().unwrap_or(f64::NAN);
    let f = |x: f64, d: usize| if x.is_finite() { format!("{x:.d$}") } else { String::new() };
    let kw = -g(1) * g(2) / 1000.0;
    let mut cols = vec![
        clock.to_string(),
        t.map(|t| format!("{t:.0}")).unwrap_or_default(),
        f(g(0), 1),
        f(g(1), 0),
        f(g(2), 1),
        f(kw, 2),
        f(g(3), 0),
        f(g(4), 0),
        f(g(5), 0),
        f(g(6), 0),
        tm.to_string(),
        bus.to_string(),
    ];
    // OBC: t1-t4, port, fan, AC V, AC A, AC kW, state; PDC: coolant, pump, fan hi, fan lo
    let dec = [0, 0, 0, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0];
    for (k, d) in dec.iter().enumerate() {
        cols.push(f(g(7 + k), *d));
    }
    cols.join(",")
}

/// Latest on-board charger reading (NaN = not answered).
#[derive(Clone, Copy)]
struct ObcSample {
    temps: [f64; 4],
    port: f64,
    fan: f64,
    ac_v: f64,
    ac_a: f64,
    ac_kw: f64,
    state: f64,
    /// PDC: powertrain coolant °C, water pump %, fan high / low speed status
    coolant: f64,
    pump: f64,
    fan_hi: f64,
    fan_lo: f64,
}

impl ObcSample {
    fn max_temp(&self) -> f64 {
        self.temps.iter().copied().filter(|t| t.is_finite()).fold(f64::NAN, f64::max)
    }
}

#[derive(Clone, Copy)]
struct ChargeSample {
    t: f64, // seconds since the session started
    soc: f64,
    volt: f64,
    amp: f64,
    kw: f64,
    temp: f64,
    cool: f64,
    cmin: f64,
    cmax: f64,
    /// forced battery cooling running (demo function)
    forced: bool,
}

#[derive(Default)]
struct ChargeState {
    hist: VecDeque<ChargeSample>,
    t0: Option<f64>,
    timer: f64,
    /// open charge recording: rows are appended to this CSV as they come (also by the OBD
    /// worker while the screen is off, see Cmd::BgMonitor)
    rec: Option<PathBuf>,
    /// 44A byte 6 (BMS thermal-management request) and pack current from the bus
    tm: Option<u8>,
    bus_amp: Option<f64>,
    /// (t, old, new, battery °C) whenever 44A byte 6 changes
    tm_log: Vec<(f64, u8, u8, f64)>,
    cycle: usize,
    /// on-board charger: latest reading, session max internal temp, (t, max temp) history
    obc: Option<ObcSample>,
    obc_max: f64,
    obc_hist: VecDeque<(f64, f64)>,
    /// the open `rec` was started automatically when the OBC began charging
    auto_rec: bool,
}

impl ChargeState {
    fn last(&self) -> Option<&ChargeSample> {
        self.hist.back()
    }
    /// °C per minute over the last 3 minutes of samples.
    fn slope(&self) -> Option<f64> {
        let b = self.hist.back()?;
        let a = self.hist.iter().find(|s| b.t - s.t <= 180.0 && !s.temp.is_nan())?;
        let dt = (b.t - a.t) / 60.0;
        (dt >= 0.5 && !b.temp.is_nan()).then(|| (b.temp - a.temp) / dt)
    }
}

struct Report {
    time: String,
    vin: String,
    vehicle: String,
    summary: String,
    rows: Vec<(String, String, String, String)>,
}

pub struct ScanApp {
    lang: Lang,
    units: Units,
    stack: Vec<Screen>,
    systems: Vec<System>,
    codes_read: bool,
    vin_text: String,
    /// waiting for Ev::Vin after "Read VIN from car"
    vin_reading: bool,
    /// battery-module DID scan: progress (done, total, found) while running
    did_scan: Option<(usize, usize, usize)>,
    did_found: Vec<(u16, Vec<u8>)>,
    did_scan_ext: bool,
    current_vin: String,
    vehicle: String,
    live: LiveState,
    reports: Vec<Report>,
    toast: Option<(String, f64)>,
    confirm_clear: bool,
    /// modules asked so far while reading / clearing codes (done, total)
    dtc_progress: Option<(usize, usize)>,
    /// modules that answered with no codes (names), shown folded
    dtc_show_clean: bool,
    now: f64,
    frames: u64,
    screenshot: Option<PathBuf>,
    shot_requested: bool,
    shot: Arc<Mutex<Option<Arc<egui::ColorImage>>>>,
    link: Link,
    /// Bluetooth devices from the last scan, and whether a scan is in flight.
    bt_devices: Vec<obd::BtDevice>,
    bt_scanning: bool,
    /// Device the user last tapped "Connect" on, so its row can show the link state.
    bt_target: Option<String>,
    /// indexes of the live values asked for in the last real request
    live_req: Vec<usize>,
    /// Live Data: alternate request rounds with listen-only bus rounds
    live_round: u64,
    live_sniff_pending: bool,
    dtc_reading: bool,
    charge: ChargeState,
    charge_pending: bool,
    batt: BattState,
    batt_pending: bool,
    batt_detail: Option<BattDetail>,
    batt_detail_pending: bool,
    /// app time of the last detail read (re-read every 5 min while on the screen)
    batt_detail_t: f64,
    check: Option<CheckState>,
    /// battery history (oldest first) and per-cell balancing rows, loaded from / appended to
    /// the app's data folder
    history: Vec<Snap>,
    balance_hist: Vec<(i64, Vec<f64>)>,
    /// a Battery-screen snapshot is due once the pack values arrive (max one per hour)
    batt_snap_due: bool,
    /// what the OBD worker was last told about background monitoring (on, recording file)
    bg_sent: (bool, Option<PathBuf>),
    /// the outstanding Live request is the check's 12 V read
    check_live_pending: bool,
    /// the outstanding Sniff is the Charge screen's quick 44A read
    charge_sniff_pending: bool,
    sniff: crate::sniff::SniffState,
    /// VoltaChill logo texture + its background colour (the hero card uses the same colour)
    logo: Option<(egui::TextureHandle, Color32)>,
}

impl ScanApp {
    pub fn new(cc: &eframe::CreationContext<'_>, screenshot: Option<PathBuf>, open: Option<String>) -> Self {
        install_fonts(&cc.egui_ctx);
        let mut style = (*cc.egui_ctx.global_style()).clone();
        style.visuals = egui::Visuals::light();
        style.spacing.item_spacing = Vec2::new(8.0, 8.0);
        style.spacing.button_padding = Vec2::new(12.0, 6.0);
        cc.egui_ctx.set_global_style(style);

        let pids = car::pids();
        let n = pids.len();
        let ranges = pids.iter().map(|p| (p.min, p.max)).collect();
        let mut selected = vec![false; n];
        for s in selected.iter_mut().take(4) {
            *s = true;
        }
        let mut app = Self {
            lang: if cfg!(target_os = "android") { Lang::Th } else { Lang::En },
            units: Units::Metric,
            stack: vec![Screen::Connect],
            systems: Vec::new(),
            codes_read: false,
            vin_text: String::new(),
            vin_reading: false,
            did_scan: None,
            did_found: Vec::new(),
            did_scan_ext: false,
            current_vin: String::new(),
            vehicle: String::new(),
            live: LiveState {
                pids,
                selected,
                ranges,
                values: vec![0.0; n],
                raw: vec![(0, 0); n],
                history: vec![VecDeque::new(); n],
                paused: false,
                graph: false,
                table: false,
                recording: None,
                range_edit: None,
            },
            reports: Vec::new(),
            toast: None,
            confirm_clear: false,
            dtc_progress: None,
            dtc_show_clean: false,
            now: 0.0,
            frames: 0,
            screenshot,
            shot_requested: false,
            shot: Arc::new(Mutex::new(None)),
            link: Link::start(),
            bt_devices: Vec::new(),
            bt_scanning: false,
            bt_target: None,
            live_req: Vec::new(),
            live_round: 0,
            live_sniff_pending: false,
            dtc_reading: false,
            charge: ChargeState::default(),
            charge_pending: false,
            batt: BattState::default(),
            batt_pending: false,
            batt_detail: None,
            batt_detail_pending: false,
            batt_detail_t: 0.0,
            check: None,
            history: Vec::new(),
            balance_hist: Vec::new(),
            batt_snap_due: false,
            bg_sent: (false, None),
            check_live_pending: false,
            charge_sniff_pending: false,
            sniff: Default::default(),
            logo: None,
        };
        let (h, b) = history::load(&out_dir());
        app.history = h;
        app.balance_hist = b;
        app.reports = history::load_reports(&out_dir())
            .into_iter()
            .map(|(time, vin, vehicle, summary, rows)| Report { time, vin, vehicle, summary, rows })
            .collect();
        if let Some(o) = open {
            app.open_start(&o);
        } else {
            // normal launch: the Bluetooth scan screen IS the root (so Back from it exits the
            // app instead of landing on an extra Home screen); start a scan right away.
            app.stack = vec![Screen::Connect];
            app.bt_scanning = true;
            app.link.send(Cmd::ScanDevices);
        }
        app
    }

    /// `--open <screen>`: start on another screen (desktop screenshots; car screens show the
    /// "connect first" placeholder without a car).
    fn open_start(&mut self, o: &str) {
        match o {
            "vin" => self.go(Screen::VinInput),
            "settings" => self.go(Screen::Settings),
            "connect" => self.go(Screen::Connect),
            "battery" => self.go(Screen::Battery),
            "charge" => self.go(Screen::Charge),
            "th" => self.lang = Lang::Th,
            _ => {}
        }
    }

    fn tr<'a>(&self, en: &'a str, th: &'a str) -> &'a str {
        match self.lang {
            Lang::En => en,
            Lang::Th => th,
        }
    }

    fn screen(&self) -> Screen {
        *self.stack.last().unwrap_or(&Screen::Connect)
    }

    fn go(&mut self, s: Screen) {
        self.stack.push(s);
    }

    fn back(&mut self) {
        if self.stack.len() > 1 {
            self.stack.pop();
        }
    }

    fn toast(&mut self, msg: impl Into<String>) {
        self.toast = Some((msg.into(), self.now + 3.0));
    }

    // ---------------------------------------------------------------- ticking

    fn real(&self) -> bool {
        self.link.connected()
    }

    fn tick(&mut self, dt: f64) {
        self.poll_link();
        if self.sniff.on && self.real() && !self.link.busy {
            if self.sniff.poll_next {
                self.sniff.poll_next = false;
                self.charge_pending = true;
                self.link.send(Cmd::Live(charge_req()));
            } else {
                use crate::sniff::Next;
                let cmd = match self.sniff.next() {
                    Next::Group(g) => Cmd::Sniff { filt: (g as u16) << 8, mask: crate::sniff::MASK, ms: crate::sniff::LISTEN_MS },
                    Next::Id(id) => Cmd::Sniff { filt: id, mask: 0x7FF, ms: crate::sniff::ONE_ID_MS },
                };
                // read the battery every 4th window only, so more of the bus is heard
                self.sniff.windows += 1;
                self.sniff.poll_next = self.sniff.windows % 4 == 0;
                self.link.send(cmd);
            }
        } else if self.sniff.on && !self.real() {
            // Bluetooth dropped: remember to reconnect and resume into the same file
            self.sniff.on = false;
            self.sniff.resume = true;
            self.sniff.retry_at = self.now + 3.0;
            let t = self.now - *self.charge.t0.get_or_insert(self.now);
            self.sniff.write_event(t, "LOST", "bluetooth");
        }
        if self.sniff.resume {
            if self.real() {
                self.sniff.resume = false;
                self.sniff.on = true;
                self.sniff.reconnects += 1;
                let t = self.now - *self.charge.t0.get_or_insert(self.now);
                self.sniff.write_event(t, "RECONNECT", &self.sniff.reconnects.to_string());
            } else if self.link.state != LinkState::Searching && self.now >= self.sniff.retry_at {
                self.sniff.retry_at = self.now + 20.0;
                self.link.send(Cmd::Connect);
            }
        }
        if self.screen() == Screen::Charge {
            self.charge.timer += dt;
            if self.real() {
                if !self.link.busy && self.charge.timer >= 1.0 {
                    self.charge.timer = 0.0;
                    self.charge.cycle += 1;
                    if self.charge.cycle % 3 == 0 {
                        // every 3rd round: listen to 0x44A only (thermal request + bus current)
                        self.charge_sniff_pending = true;
                        self.link.send(Cmd::Sniff { filt: 0x44A, mask: 0x7FF, ms: 700 });
                    } else {
                        self.charge_pending = true;
                        self.link.send(Cmd::Live(charge_req()));
                    }
                }
            }
        }
        if self.screen() == Screen::Battery && self.real() {
            self.batt.timer += dt;
            let stale = self.batt_detail.is_none() || self.now - self.batt_detail_t > 300.0;
            if !self.link.busy && stale && !self.batt_detail_pending {
                self.batt_detail_pending = true;
                self.batt_detail_t = self.now;
                self.link.send(batt_detail_req());
            } else if !self.link.busy && self.batt.timer >= 1.0 {
                self.batt.timer = 0.0;
                self.batt_pending = true;
                // pack values every round + one BIC group (8 DIDs) per round, in turn
                let g = self.batt.bic_next as u16;
                let req = BATT_DIDS
                    .iter()
                    .copied()
                    .chain((0..8).map(|k| BIC_BASE + 8 * g + k))
                    .map(|d| (0u8, Src::Did { hdr: 0x7E7, did: d }))
                    .collect();
                self.link.send(Cmd::Live(req));
            }
        }
        self.check_tick();
        self.bg_sync();
        let live_running = matches!(self.screen(), Screen::LiveData) && !self.live.paused;
        if self.real() {
            if live_running && !self.link.busy && !self.live.pids.is_empty() {
                let mut idx: Vec<usize> = (0..self.live.pids.len()).filter(|i| self.live.selected[*i]).collect();
                // dashboard/table show every value; only the graph polls just the selected ones
                if idx.is_empty() || !self.live.graph {
                    idx = (0..self.live.pids.len()).collect();
                }
                // broadcast IDs wanted on screen; 0x121/0x122 (speeds) are heard together
                let mut bus: Vec<u16> = Vec::new();
                for i in &idx {
                    if let Src::Bus { id } = self.live.pids[*i].src {
                        let id = if id == 0x121 || id == 0x122 { 0x120 } else { id };
                        if !bus.contains(&id) {
                            bus.push(id);
                        }
                    }
                }
                idx.retain(|i| !matches!(self.live.pids[*i].src, Src::Bus { .. }));
                self.live_round += 1;
                if !bus.is_empty() && (self.live_round % 2 == 0 || idx.is_empty()) {
                    // speeds every other bus round, the slow-changing frames in turn
                    let n = self.live_round / 2;
                    let others: Vec<u16> = bus.iter().copied().filter(|b| *b != 0x120).collect();
                    let id = if bus.contains(&0x120) && (n % 2 == 0 || others.is_empty()) {
                        0x120
                    } else {
                        others[(n / 2) as usize % others.len()]
                    };
                    let mask = if id == 0x120 { 0x7FC } else { 0x7FF };
                    self.live_sniff_pending = true;
                    self.link.send(Cmd::Sniff { filt: id, mask, ms: 500 });
                } else if !idx.is_empty() {
                    let req = idx.iter().map(|i| (self.live.pids[*i].pid, self.live.pids[*i].src)).collect();
                    self.live_req = idx;
                    self.link.send(Cmd::Live(req));
                }
            }
        }
    }

    fn poll_link(&mut self) {
        while let Ok(ev) = self.link.rx.try_recv() {
            match ev {
                Ev::State(s) => {
                    match &s {
                        LinkState::Connected(n) => {
                            self.toast(format!("Bluetooth OBD connected: {n}"));
                            // find out which live values this car answers (fills Live Data)
                            self.set_pids(Vec::new());
                            self.codes_read = false;
                            self.link.send(Cmd::Discover);
                        }
                        LinkState::Failed(e) => {
                            self.link.busy = false;
                            self.toast(format!("OBD: {e}"));
                        }
                        _ => {}
                    }
                    self.link.state = s;
                }
                Ev::Log(l) => self.link.push_log(l),
                Ev::Devices(d) => {
                    self.bt_devices = d;
                    self.bt_scanning = false;
                }
                Ev::Dids(list) => {
                    self.link.busy = false;
                    if self.batt_detail_pending {
                        self.batt_detail_pending = false;
                        self.batt_detail = Some(BattDetail::parse(&list));
                        let last = self.history.iter().rev().find(|s| s.source == "battery").map(|s| s.unix).unwrap_or(0);
                        if self.check.as_ref().is_none_or(|c| c.done) && history::now_unix() - last > 3600 {
                            self.batt_snap_due = true;
                        }
                    }
                }
                Ev::DidScanProgress { done, total, found } => {
                    self.did_scan = Some((done, total, found));
                }
                Ev::DidScanDone(found) => {
                    self.link.busy = false;
                    self.did_scan = None;
                    let mut text = String::from("did,len,hex\n");
                    for (did, d) in &found {
                        let hex: Vec<String> = d.iter().map(|b| format!("{b:02X}")).collect();
                        text += &format!("{did:04X},{},{}\n", d.len(), hex.join(" "));
                    }
                    let stem = if self.did_scan_ext { "did_scan_7E7_ext" } else { "did_scan_7E7" };
                    let m = match save_file(stem, "csv", &text) {
                        Ok(p) => format!("{} {} DID → {p}", self.tr("Found", "พบ"), found.len()),
                        Err(e) => format!("{} {e}", self.tr("Save failed:", "บันทึกไม่ได้:")),
                    };
                    self.toast(m);
                    self.did_found = found;
                }
                Ev::Vin(v) => {
                    self.link.busy = false;
                    self.vin_reading = false;
                    match v {
                        Some(v) => self.vin_text = v,
                        None => {
                            let m = self.tr("The car did not give a VIN", "รถไม่ส่ง VIN มา").to_string();
                            self.toast(m);
                        }
                    }
                }
                Ev::Discovered(pids) => {
                    self.link.busy = false;
                    if pids.is_empty() {
                        self.toast("The car gives no live data this app knows");
                    }
                    self.set_pids(pids);
                }
                Ev::Live(vals) if self.check_live_pending => {
                    self.link.busy = false;
                    self.check_live_pending = false;
                    if let Some(c) = &mut self.check {
                        c.v12 = vals.first().copied().flatten().map(|(v, _)| v);
                    }
                }
                Ev::Live(vals) if self.charge_pending => {
                    self.link.busy = false;
                    self.charge_pending = false;
                    let v: Vec<f64> = vals.iter().map(|x| x.map(|(v, _)| v).unwrap_or(f64::NAN)).collect();
                    let t0 = *self.charge.t0.get_or_insert(self.now);
                    let t = self.now - t0;
                    if v.len() >= 17 {
                        let o = ObcSample {
                            temps: [v[7], v[8], v[9], v[10]],
                            port: v[11],
                            fan: v[12],
                            ac_v: v[13],
                            ac_a: v[14],
                            ac_kw: v[15],
                            state: v[16],
                            coolant: v.get(17).copied().unwrap_or(f64::NAN),
                            pump: v.get(18).copied().unwrap_or(f64::NAN),
                            fan_hi: v.get(19).copied().unwrap_or(f64::NAN),
                            fan_lo: v.get(20).copied().unwrap_or(f64::NAN),
                        };
                        let mx = o.max_temp();
                        if mx.is_finite() {
                            self.charge.obc_max = if self.charge.obc_hist.is_empty() { mx } else { self.charge.obc_max.max(mx) };
                            self.charge.obc_hist.push_back((t, mx));
                            while self.charge.obc_hist.len() > 20_000 {
                                self.charge.obc_hist.pop_front();
                            }
                        }
                        // auto-record AC charges: start when the OBC reports "charging" (state 2),
                        // save when it stops (only recordings this code started itself)
                        let charging = o.state == 2.0;
                        if charging && self.charge.rec.is_none() {
                            self.charge_rec_start();
                            self.charge.auto_rec = true;
                            let m = self.tr("AC charging — recording started", "เริ่มชาร์จ AC — เริ่มบันทึกอัตโนมัติ").to_string();
                            self.toast(m);
                        } else if !charging && o.state.is_finite() && self.charge.auto_rec {
                            if let Some(m) = self.charge_rec_save() {
                                self.toast(m);
                            }
                        }
                        self.charge.obc = Some(o);
                    }
                    if let Some(path) = &self.charge.rec {
                        let tm = self.charge.tm.map(|x| format!("{x:02X}")).unwrap_or_default();
                        let bus = self.charge.bus_amp.map(|x| format!("{x:.1}")).unwrap_or_default();
                        let row = charge_csv_row(&clock_text(), Some(t), &v, &tm, &bus);
                        if let Ok(mut fh) = std::fs::OpenOptions::new().append(true).open(path) {
                            use std::io::Write;
                            let _ = writeln!(fh, "{row}");
                        }
                    }
                    self.charge_push(ChargeSample { t, soc: v[0], volt: v[1], amp: v[2], kw: -v[1] * v[2] / 1000.0, temp: v[3], cool: v[4], cmin: v[5], cmax: v[6], forced: false });
                }
                Ev::Live(vals) if self.batt_pending => {
                    self.link.busy = false;
                    self.batt_pending = false;
                    // BATT_DIDS order, then one BIC group (8 values)
                    let v: Vec<f64> = vals.iter().map(|x| x.map(|(v, _)| v).unwrap_or(f64::NAN)).collect();
                    let n = BATT_DIDS.len();
                    let b = &mut self.batt;
                    b.got = true;
                    b.soc = v[0];
                    b.volt = v[1];
                    b.temp = v[2];
                    b.tmax = v[3];
                    b.cmin = v[4];
                    b.cmax = v[5];
                    b.soh = v[6];
                    b.cycles = v[7];
                    b.tmin = v[8];
                    b.vmin_no = v[9];
                    b.vmax_no = v[10];
                    b.tmin_no = v[11];
                    b.tmax_no = v[12];
                    if v.len() >= n + 8 {
                        let g = b.bic_next;
                        let grp: [f64; 8] = std::array::from_fn(|k| v[n + k]);
                        if grp.iter().all(|x| x.is_finite()) {
                            b.bic[g] = grp;
                            b.bic_got[g] = true;
                        }
                        b.bic_next = (g + 1) % 10;
                    }
                    if self.batt_snap_due && self.batt.got {
                        self.batt_snap_due = false;
                        self.record_snapshot("battery", f64::NAN);
                    }
                }
                Ev::Live(vals) => {
                    self.link.busy = false;
                    for (k, v) in vals.into_iter().enumerate() {
                        let Some(&i) = self.live_req.get(k) else { break };
                        if i >= self.live.values.len() {
                            break;
                        }
                        match v {
                            Some((v, raw)) => {
                                self.live.values[i] = v;
                                self.live.raw[i] = (raw[0], raw[1]);
                                let h = &mut self.live.history[i];
                                h.push_back(v);
                                while h.len() > HISTORY {
                                    h.pop_front();
                                }
                            }
                            None => self.live.values[i] = f64::NAN,
                        }
                    }
                    if let Some(rec) = &mut self.live.recording {
                        let row: Vec<String> = self.live.values.iter().map(|v| format!("{v:.2}")).collect();
                        rec.push(format!("{},{}", clock_text(), row.join(",")));
                    }
                }
                Ev::DtcProgress { done, total } => {
                    self.dtc_progress = Some((done, total));
                }
                Ev::Dtcs(list) => {
                    self.link.busy = false;
                    self.dtc_reading = false;
                    self.dtc_progress = None;
                    self.systems = list
                        .into_iter()
                        .map(|(req, name, codes)| System {
                            name,
                            dtcs: codes
                                .into_iter()
                                .map(|d| {
                                    let base = car::dtc_from_bytes(d.a, d.b);
                                    let text = obd::byd_dtc_text(req, &base, d.c).unwrap_or(dtc_group(d.a));
                                    Dtc { raw: [d.a, d.b], ft: d.c, text, status: d.status }
                                })
                                .collect(),
                        })
                        .collect();
                    if self.systems.is_empty() {
                        self.toast("No module answered the trouble-code request");
                    }
                    self.make_report();
                }
                Ev::Frames { frames, .. } if self.live_sniff_pending => {
                    self.link.busy = false;
                    self.live_sniff_pending = false;
                    // last frame of each ID wins
                    for i in 0..self.live.pids.len() {
                        let Src::Bus { id } = self.live.pids[i].src else { continue };
                        let Some(f) = frames.iter().rev().find(|f| f.id == id) else { continue };
                        if let Some(v) = crate::sniff::live_bus_value(self.live.pids[i].name, id, &f.data) {
                            self.live.values[i] = v;
                            let h = &mut self.live.history[i];
                            h.push_back(v);
                            while h.len() > HISTORY {
                                h.pop_front();
                            }
                        }
                    }
                }
                Ev::Frames { frames, .. } if self.charge_sniff_pending => {
                    self.link.busy = false;
                    self.charge_sniff_pending = false;
                    if let Some((tm, a)) = frames.iter().rev().filter(|f| f.id == 0x44A).find_map(|f| crate::sniff::decode_44a(&f.data)) {
                        if let Some(old) = self.charge.tm {
                            if old != tm {
                                let t = self.now - *self.charge.t0.get_or_insert(self.now);
                                let temp = self.charge.last().map(|s| s.temp).unwrap_or(f64::NAN);
                                self.charge.tm_log.push((t, old, tm, temp));
                                self.toast(format!("44A byte 6: {old:02X} → {tm:02X} at {temp:.0} °C"));
                            }
                        }
                        self.charge.tm = Some(tm);
                        self.charge.bus_amp = Some(a);
                    }
                }
                Ev::Frames { filt, mask, frames, overflow, silent } => {
                    self.link.busy = false;
                    let t0 = *self.charge.t0.get_or_insert(self.now);
                    let (temp, kw) = self.charge.last().map(|s| (s.temp, s.kw)).unwrap_or((f64::NAN, f64::NAN));
                    self.sniff.ingest(self.now - t0, filt, mask, &frames, temp, kw);
                    self.sniff.silent = Some(silent);
                    if overflow {
                        self.sniff.overflows += 1;
                    }
                }
                Ev::Cleared(msg) => {
                    self.link.busy = false;
                    self.dtc_progress = None;
                    self.toast(msg);
                    self.read_codes();
                }
            }
        }
    }

    /// Open a new charge recording file (header written now, rows appended as they come).
    fn charge_rec_start(&mut self) {
        let dir = out_dir();
        let _ = std::fs::create_dir_all(&dir);
        let p = dir.join(format!("charge_{}.csv", history::now_unix()));
        if std::fs::write(&p, format!("{CHARGE_CSV_HEADER}\n")).is_ok() {
            self.charge.rec = Some(p);
        } else {
            let m = self.tr("Cannot create the recording file", "สร้างไฟล์บันทึกไม่ได้").to_string();
            self.toast(m);
        }
    }

    /// Close the open charge recording (rows are already on disk).
    fn charge_rec_save(&mut self) -> Option<String> {
        let p = self.charge.rec.take()?;
        self.charge.auto_rec = false;
        Some(format!("{} {}", self.tr("Saved:", "บันทึกแล้ว:"), p.display()))
    }

    fn charge_push(&mut self, s: ChargeSample) {
        self.charge.hist.push_back(s);
        while self.charge.hist.len() > 20_000 {
            self.charge.hist.pop_front();
        }
    }

    fn sniff_ui(&mut self, ui: &mut egui::Ui) {
        use crate::sniff::{self, GROUPS};
        let real = self.real();
        ui.add_space(4.0);
        let mut msg = None;
        ui.horizontal(|ui| {
            let running = self.sniff.on || self.sniff.resume;
            let label = if running { self.tr("■ Stop listening", "■ หยุดฟัง") } else { self.tr("▶ Start listening", "▶ เริ่มฟัง") };
            if ui.add_enabled(real || running, blue_button(label, Vec2::new(200.0, 40.0))).clicked() {
                if running {
                    self.sniff.on = false;
                    self.sniff.resume = false;
                } else {
                    let stamp: String = clock_text().chars().filter(|c| c.is_ascii_digit()).collect();
                    match self.sniff.open_file(out_dir(), &stamp) {
                        Ok(p) => msg = Some(format!("{} {}", self.tr("Writing", "บันทึกลง"), p.display())),
                        Err(e) => msg = Some(format!("File: {e}")),
                    }
                    self.sniff.on = true;
                }
            }
            if ui.add_enabled(self.sniff.on, grey_button(self.tr("⚑ Mark", "⚑ ทำเครื่องหมาย"), Vec2::new(150.0, 40.0))).clicked() {
                let t = self.now - *self.charge.t0.get_or_insert(self.now);
                self.sniff.marks.push(t);
                self.sniff.write_mark(t, self.sniff.marks.len());
                msg = Some(format!("Mark {} @ {:.0} s", self.sniff.marks.len(), t));
            }
            if self.sniff.resume {
                ui.spinner();
                ui.label(RichText::new(self.tr("Bluetooth lost — reconnecting by itself, listening goes on in the same file", "บลูทูธหลุด — กำลังต่อใหม่เอง แล้วจะฟังต่อในไฟล์เดิม")).color(RED));
            } else if !real {
                ui.label(RichText::new(self.tr("Connect the Bluetooth OBD first (Diagnostic → Connect).", "เชื่อมต่อ OBD บลูทูธก่อน (Diagnostic → เชื่อมต่อ)")).color(RED));
            } else if self.sniff.reconnects > 0 {
                ui.label(RichText::new(format!("{} {}", self.tr("Reconnected after Bluetooth drops:", "ต่อใหม่หลังบลูทูธหลุด:"), self.sniff.reconnects)).color(Color32::GRAY));
            }
        });
        if let Some(m) = msg {
            self.toast(m);
        }
        ui.add_space(6.0);
        // ---- status line
        let s = self.charge.last().copied();
        let fmt = |v: Option<f64>, d: usize| v.filter(|v| !v.is_nan()).map(|v| format!("{v:.d$}")).unwrap_or("—".into());
        ui.label(RichText::new(format!(
            "{} {} frames · {} IDs · {} {}°C · {} kW · SOC {}%{}{}",
            self.tr("Heard", "ได้ยิน"),
            self.sniff.total,
            self.sniff.ids.len(),
            self.tr("battery", "แบต"),
            fmt(s.map(|s| s.temp), 0),
            fmt(s.map(|s| s.kw), 1),
            fmt(s.map(|s| s.soc), 0),
            match self.sniff.silent {
                Some(false) => "  ·  ⚠ adapter has no silent mode (it ACKs frames — normal for a CAN node)",
                _ => "",
            },
            if self.sniff.overflows > 0 { "  ·  BUFFER FULL seen (some frames lost)" } else { "" },
        )).size(14.0));
        if let Some(p) = &self.sniff.file {
            ui.label(RichText::new(p.display().to_string()).size(12.0).color(Color32::GRAY));
        }
        // ---- group chips
        ui.horizontal(|ui| {
            for g in 0..GROUPS {
                let now = self.sniff.on && (self.sniff.group + GROUPS - 1) % GROUPS == g;
                let txt = match self.sniff.counts[g] {
                    Some(n) => format!("{g}xx {n}"),
                    None => format!("{g}xx ?"),
                };
                let fill = if now { ACCENT } else if self.sniff.counts[g].unwrap_or(0) > 0 { Color32::WHITE } else { GREY_TOP };
                let tc = if now { Color32::WHITE } else { TEXT_DARK };
                ui.add(egui::Button::new(RichText::new(txt).monospace().size(12.0).color(tc)).fill(fill).corner_radius(6));
            }
        });
        ui.add_space(6.0);
        // ---- two columns: candidates per event | all IDs
        let timeline: Vec<(f64, f64, f64)> = self.charge.hist.iter().map(|s| (s.t, s.temp, s.kw)).collect();
        let evs = sniff::events(&timeline, &self.sniff.marks);
        let h = ui.available_height();
        let mut new_focus: Option<Option<u16>> = None;
        ui.columns(2, |cols| {
            egui::Frame::new().fill(Color32::WHITE).corner_radius(8).inner_margin(10).show(&mut cols[0], |ui| {
                ui.set_width(ui.available_width());
                ui.set_min_height(h - 24.0);
                ui.label(RichText::new(self.tr("Bytes that changed at each event", "ไบต์ที่เปลี่ยนตรงแต่ละเหตุการณ์")).strong().size(15.0));
                if evs.is_empty() {
                    ui.label(RichText::new(self.tr(
                        "No event yet. Events: charging starts, battery reaches 39 °C, or a Mark.",
                        "ยังไม่มีเหตุการณ์ (เริ่มชาร์จ, แบตถึง 39°C หรือกด Mark)",
                    )).color(Color32::GRAY));
                }
                egui::ScrollArea::vertical().id_salt("cand").show(ui, |ui| {
                    for ev in &evs {
                        ui.add_space(4.0);
                        ui.label(RichText::new(format!("{}  @ {:.0} s", ev.name, ev.t)).strong().color(ACCENT));
                        let c = self.sniff.candidates(ev.t);
                        if c.is_empty() {
                            ui.label(RichText::new("—").color(Color32::GRAY));
                        }
                        for k in c {
                            ui.label(RichText::new(format!(
                                "{:03X} byte {}: {:02X} → {:02X}   {:+.0} s   ({}× total)",
                                k.id, k.byte, k.old, k.new, k.dt, k.total_changes
                            )).monospace().size(12.5));
                        }
                    }
                });
            });
            egui::Frame::new().fill(Color32::WHITE).corner_radius(8).inner_margin(10).show(&mut cols[1], |ui| {
                ui.set_width(ui.available_width());
                ui.set_min_height(h - 24.0);
                ui.label(RichText::new(self.tr("IDs heard — click an ID to listen to it only", "ID ที่ได้ยิน — คลิก ID เพื่อฟังเฉพาะตัวนั้น")).strong().size(15.0));
                if let Some(fid) = self.sniff.focus {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(format!("{} {fid:03X}  {}", self.tr("Focus:", "โฟกัส:"), sniff::known(fid))).strong().color(ACCENT));
                        if ui.small_button(self.tr("« back to all IDs", "« กลับไปฟังทุก ID")).clicked() {
                            new_focus = Some(None);
                        }
                    });
                    if let Some(st) = self.sniff.ids.get(&fid) {
                        ui.label(RichText::new(self.tr("Latest changes (newest first):", "การเปลี่ยนล่าสุด (ใหม่สุดก่อน):")).size(12.5).color(Color32::GRAY));
                        if let Some(v) = sniff::decode_known(fid, &st.last) {
                            ui.label(RichText::new(format!("{} {v}", self.tr("Now:", "ตอนนี้:"))).strong().size(18.0).color(GREEN));
                        }
                        // BYD signal names of this frame, with the current byte value
                        for sg in sniff::signals(fid).iter().filter(|s| !s.name.contains("ONLINE")).take(16) {
                            if let Some((b, k)) = sg.pos() {
                                let now = st.last.get(b).map(|v| format!("{v:02X}")).unwrap_or("--".into());
                                ui.label(RichText::new(format!("b{b}.{k} [{now}]  {}", sg.name)).monospace().size(11.5));
                            }
                        }
                        for &(t, i, o, n) in st.changes.iter().rev().take(12) {
                            ui.label(RichText::new(format!("{t:7.1} s   byte {i}: {o:02X} → {n:02X}")).monospace().size(12.5));
                        }
                    }
                    ui.separator();
                }
                egui::ScrollArea::vertical().id_salt("ids").show(ui, |ui| {
                    for (id, st) in &self.sniff.ids {
                        ui.horizontal(|ui| {
                            if ui.link(RichText::new(format!("{id:03X}")).monospace().strong()).clicked() {
                                new_focus = Some(Some(*id));
                            }
                            for (i, b) in st.last.iter().enumerate() {
                                let c = if st.changes_per_byte[i] > 0 { Color32::from_rgb(0xF2, 0x6B, 0x1D) } else { TEXT_DARK };
                                ui.label(RichText::new(format!("{b:02X}")).monospace().color(c));
                            }
                            ui.label(RichText::new(format!("×{}", st.count)).size(11.5).color(Color32::GRAY));
                            if let Some(v) = sniff::decode_known(*id, &st.last) {
                                ui.label(RichText::new(v).strong().size(12.5).color(GREEN));
                            }
                            let (short, all) = sniff::signal_summary(*id);
                            if !short.is_empty() {
                                ui.label(RichText::new(short).size(11.0).color(Color32::GRAY)).on_hover_text(all);
                            }
                            let k = sniff::known(*id);
                            if !k.is_empty() {
                                ui.label(RichText::new(k).size(11.5).color(ACCENT));
                            }
                        });
                    }
                });
            });
        });
        if let Some(f) = new_focus {
            self.sniff.focus = f;
            self.sniff.id_pos = 0;
        }
    }

    /// 44A byte 6 = CHARGING_THERMAL_MANAGEMENT_REQUEST_STATUS (name and position from BYD's own
    /// signal table; the meaning of each value is not in the update, so the raw value is shown).
    /// The app logo (assets/logo.png) as a texture, loaded once, and its corner colour.
    fn logo(&mut self, ctx: &egui::Context) -> (egui::TextureHandle, Color32) {
        self.logo
            .get_or_insert_with(|| {
                let img = image::load_from_memory(include_bytes!("../assets/logo.png")).expect("logo.png").to_rgba8();
                let [r, g, b, _] = img.get_pixel(4, 4).0;
                let ci = egui::ColorImage::from_rgba_unmultiplied([img.width() as usize, img.height() as usize], img.as_raw());
                (ctx.load_texture("voltachill-logo", ci, egui::TextureOptions::LINEAR), Color32::from_rgb(r, g, b))
            })
            .clone()
    }

    /// OBC state code (BYD's "Charge/Discharge System Operating Status") as short text.
    fn obc_state_text(&self, v: f64) -> &'static str {
        if !v.is_finite() {
            return "—";
        }
        let (en, th) = match v as i32 {
            0 => ("Initialising", "เริ่มต้น"),
            1 => ("Ready to charge", "พร้อมชาร์จ"),
            2 => ("Charging", "กำลังชาร์จ"),
            3 => ("Charge ended", "ชาร์จจบ"),
            4 => ("Paused", "หยุดชั่วคราว"),
            5 => ("Stopped", "หยุดชาร์จ"),
            6 => ("V2L ready", "พร้อมจ่ายไฟออก"),
            7 => ("V2L on", "กำลังจ่ายไฟออก"),
            8 => ("V2L ended", "จ่ายไฟออกจบ"),
            9 => ("V2L not allowed", "ไม่อนุญาตจ่ายไฟออก"),
            13 => ("Constant voltage out", "จ่ายแรงดันคงที่"),
            _ => ("Other", "อื่น ๆ"),
        };
        if self.lang == Lang::Th { th } else { en }
    }

    /// On-board charger card: internal / port temperatures, fan, AC power and state, with the
    /// session maximum and the temperature trend — for watching heat on long AC charges.
    fn obc_panel(&mut self, ui: &mut egui::Ui) {
        let Some(o) = self.charge.obc else { return };
        let fmt = |x: f64, d: usize, u: &str| if x.is_finite() { format!("{x:.d$} {u}") } else { "—".into() };
        let mx = o.max_temp();
        // trend over the last 10 minutes of OBC readings
        let trend = {
            let h = &self.charge.obc_hist;
            match (h.back(), h.iter().find(|(t, _)| h.back().is_some_and(|b| b.0 - t <= 600.0))) {
                (Some(&(t1, v1)), Some(&(t0, v0))) if t1 - t0 >= 60.0 => Some((v1 - v0) / (t1 - t0) * 600.0),
                _ => None,
            }
        };
        egui::Frame::new().fill(Color32::WHITE).corner_radius(10).inner_margin(12).show(ui, |ui| {
            ui.set_width(ui.available_width());
            // the AC frame header already names the OBC; show its state only
            ui.label(RichText::new(format!("{} {}", self.tr("State:", "สถานะ:"), self.obc_state_text(o.state))).size(14.0).color(Color32::GRAY));
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(fmt(mx, 0, "°C")).size(30.0).strong().color(TEXT_DARK));
                ui.vertical(|ui| {
                    ui.label(RichText::new(self.tr("hottest internal sensor", "จุดร้อนสุดภายใน")).size(13.0).color(Color32::GRAY));
                    let mut extra = format!("{} {}", self.tr("max this session", "สูงสุดรอบนี้"), fmt(self.charge.obc_max, 0, "°C"));
                    if let Some(r) = trend {
                        extra += &format!(" · {r:+.1} °C / 10 {}", self.tr("min", "นาที"));
                    }
                    ui.label(RichText::new(extra).size(13.0).color(Color32::GRAY));
                });
            });
            ui.add_space(4.0);
            egui::Grid::new("obc").num_columns(4).spacing([14.0, 6.0]).show(ui, |ui| {
                let cells = [
                    ("T1", fmt(o.temps[0], 0, "°C")),
                    ("T2", fmt(o.temps[1], 0, "°C")),
                    ("T3", fmt(o.temps[2], 0, "°C")),
                    ("T4", fmt(o.temps[3], 0, "°C")),
                    (self.tr("Port", "หัวชาร์จ"), fmt(o.port, 0, "°C")),
                    (self.tr("Fan", "พัดลม"), fmt(o.fan, 0, "%")),
                    ("AC", format!("{} · {}", fmt(o.ac_v, 0, "V"), fmt(o.ac_a, 1, "A"))),
                    (self.tr("Power", "กำลัง"), fmt(o.ac_kw, 2, "kW")),
                    (self.tr("Coolant", "น้ำหล่อเย็น"), fmt(o.coolant, 0, "°C")),
                    (self.tr("Pump", "ปั๊มน้ำ"), fmt(o.pump, 0, "%")),
                    (self.tr("Fan hi/lo", "พัดลม สูง/ต่ำ"), {
                        // BYD: 0 off, 1 on, 255 invalid data
                        let b = |x: f64| if x == 1.0 { "ON" } else if x == 0.0 { "off" } else { "—" };
                        format!("{} / {}", b(o.fan_hi), b(o.fan_lo))
                    }),
                    ("", String::new()),
                ];
                for (k, (l, v)) in cells.iter().enumerate() {
                    ui.label(RichText::new(*l).size(13.0).color(Color32::GRAY));
                    ui.label(RichText::new(v).monospace().size(14.0).color(TEXT_DARK));
                    if k % 2 == 1 {
                        ui.end_row();
                    }
                }
            });
        });
    }

    fn thermal_request_panel(&mut self, ui: &mut egui::Ui) {
        let real = self.real();
        egui::Frame::new().fill(Color32::WHITE).corner_radius(10).inner_margin(12).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal_wrapped(|ui| {
                ui.label(RichText::new(self.tr("BMS thermal-management request", "BMS ขอจัดการความร้อน")).strong().size(16.0).color(ACCENT));
            });
            match (real, self.charge.tm) {
                (false, _) => {
                    ui.label(RichText::new("—").color(Color32::GRAY));
                }
                (true, None) => {
                    ui.horizontal_wrapped(|ui| {
                        ui.spinner();
                        ui.label(self.tr("listening for 44A…", "กำลังฟัง 44A…"));
                    });
                }
                (true, Some(tm)) => {
                    let base = tm >> 4 == 1; // bits 4-7 = 1 parked / below 39 °C (2026-11-08)
                    let col = if base { GREEN } else { Color32::from_rgb(0xF2, 0x6B, 0x1D) };
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(format!("{tm:02X}")).monospace().strong().size(30.0).color(col));
                        // bits 4-7 seen on 2026-11-08: 1 at 32–36 °C, 2 at 39–40 °C, 4 at 41 °C (DC charge)
                        let note = if base {
                            self.tr("no thermal request", "ยังไม่ขอจัดการความร้อน")
                        } else {
                            self.tr("BMS is asking for thermal management", "BMS กำลังขอจัดการความร้อน")
                        };
                        ui.label(RichText::new(note).color(col));
                    });
                }
            }
            for (t, o, n, temp) in self.charge.tm_log.iter().rev().take(5) {
                ui.label(RichText::new(format!("{t:7.0} s   {o:02X} → {n:02X}   {} {temp:.0} °C", self.tr("battery", "แบต"))).monospace().size(12.5));
            }
        });
    }

    fn battery_ui(&mut self, ui: &mut egui::Ui) {
        // dark theme, like a scan-tool battery page
        const BG: Color32 = Color32::from_rgb(0x1B, 0x1E, 0x24);
        const CARD: Color32 = Color32::from_rgb(0x25, 0x29, 0x32);
        const TILE: Color32 = Color32::from_rgb(0x2E, 0x33, 0x3D);
        const HEAD: Color32 = Color32::from_rgb(0x2F, 0x9B, 0xEA);
        const LBL: Color32 = Color32::from_rgb(0xAA, 0xB2, 0xBF);
        const ORANGE: Color32 = Color32::from_rgb(0xEF, 0xA9, 0x2C);
        const REDC: Color32 = Color32::from_rgb(0xE5, 0x48, 0x3F);
        let b = self.batt;
        let det = self.batt_detail.clone();
        let dash = "—".to_string();
        let fin = |x: f64| b.got && x.is_finite();
        let num = |x: f64, d: usize, u: &str| if x.is_finite() { format!("{x:.d$} {u}") } else { "—".to_string() };
        let cell_no = |x: f64| if fin(x) { format!("#{x:.0}") } else { "—".to_string() };
        let volts = |mv: f64| if fin(mv) { format!("{:.3} V", mv / 1000.0) } else { "—".to_string() };
        let deg = |t: f64| if fin(t) { format!("{t:.0} °C") } else { "—".to_string() };

        let cells = match (fin(b.volt), fin(b.cmax) && fin(b.cmin)) {
            (true, true) => format!("{:.0}", b.volt / ((b.cmax + b.cmin) / 2000.0)),
            _ => dash.clone(),
        };
        let (cap_now, cap_fac) = det.as_ref().map(|d| (d.cap_now, d.cap_factory)).unwrap_or((f64::NAN, f64::NAN));
        let pack = vec![
            (self.tr("Cell count", "จำนวนเซลล์"), cells),
            (self.tr("Pack voltage", "แรงดันแพ็ก"), if fin(b.volt) { format!("{:.1} V", b.volt) } else { dash.clone() }),
            ("SOC", if fin(b.soc) { format!("{:.1} %", b.soc) } else { dash.clone() }),
            ("SOH", if fin(b.soh) { format!("{:.0} %", b.soh) } else { dash.clone() }),
            (self.tr("Capacity now", "ความจุปัจจุบัน"), num(cap_now, 1, "Ah")),
            (self.tr("Factory capacity", "ความจุโรงงาน"), num(cap_fac, 1, "Ah")),
            (self.tr("Capacity left", "ความจุคงเหลือ"), num(cap_now / cap_fac * 100.0, 1, "%")),
            (self.tr("Charge count", "จำนวนครั้งที่ชาร์จ"), if fin(b.cycles) { format!("{:.0}", b.cycles) } else { dash.clone() }),
        ];
        let volt_rows = vec![
            (self.tr("Maximum cell voltage", "แรงดันเซลล์สูงสุด"), volts(b.cmax)),
            (self.tr("Max position", "ตำแหน่งสูงสุด"), cell_no(b.vmax_no)),
            (self.tr("Minimum cell voltage", "แรงดันเซลล์ต่ำสุด"), volts(b.cmin)),
            (self.tr("Min position", "ตำแหน่งต่ำสุด"), cell_no(b.vmin_no)),
            (self.tr("Cell difference", "ผลต่างเซลล์"), if fin(b.cmax) && fin(b.cmin) { format!("{:.3} V", (b.cmax - b.cmin) / 1000.0) } else { dash.clone() }),
        ];
        let mut temp_rows = vec![
            (self.tr("Maximum temperature", "อุณหภูมิสูงสุด"), deg(b.tmax)),
            (self.tr("Max position", "ตำแหน่งสูงสุด"), cell_no(b.tmax_no)),
            (self.tr("Minimum temperature", "อุณหภูมิต่ำสุด"), deg(b.tmin)),
            (self.tr("Min position", "ตำแหน่งต่ำสุด"), cell_no(b.tmin_no)),
            (self.tr("Average", "เฉลี่ย"), deg(b.temp)),
            (self.tr("Temperature difference", "ผลต่างอุณหภูมิ"), if fin(b.tmax) && fin(b.tmin) { format!("{:.0} °C", b.tmax - b.tmin) } else { dash.clone() }),
        ];
        if let Some(n) = det.as_ref().and_then(|d| d.temp_bad) {
            temp_rows.push((self.tr("Faulty sensors", "เซนเซอร์ผิดปกติ"), n.to_string()));
        }
        let bms_rows = det.as_ref().map(|d| {
            vec![
                (self.tr("Max charge power", "ชาร์จได้สูงสุด"), num(d.max_chg_kw, 1, "kW")),
                (self.tr("Max discharge power", "จ่ายไฟได้สูงสุด"), num(d.max_dis_kw, 1, "kW")),
                (self.tr("Charged (lifetime)", "ชาร์จเข้าสะสม"), format!("{} · {}", num(d.chg_kwh, 0, "kWh"), num(d.chg_ah, 0, "Ah"))),
                (self.tr("Discharged (lifetime)", "ใช้ไปสะสม"), format!("{} · {}", num(d.dis_kwh, 0, "kWh"), num(d.dis_ah, 0, "Ah"))),
                (self.tr("Insulation", "ความต้านทานฉนวน"), num(d.insul_kohm, 0, "kΩ")),
                (
                    self.tr("Cell sampling faults", "เซลล์วัดค่าผิดปกติ"),
                    d.cell_bad.as_ref().map(|v| v.iter().filter(|x| **x).count().to_string()).unwrap_or(dash.clone()),
                ),
            ]
        });

        let card = |ui: &mut egui::Ui, title: &str, rows: &[(&str, String)]| {
            egui::Frame::new().fill(CARD).corner_radius(12).inner_margin(14).show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.label(RichText::new(title).size(17.0).color(HEAD));
                let (r, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 1.0), Sense::hover());
                ui.painter().rect_filled(r, 0.0, Color32::from_gray(70));
                ui.add_space(4.0);
                egui::Grid::new(title).num_columns(2).spacing([18.0, 9.0]).show(ui, |ui| {
                    for (k, v) in rows {
                        ui.label(RichText::new(*k).size(15.0).color(LBL));
                        ui.label(RichText::new(v).monospace().size(16.0).color(Color32::WHITE));
                        ui.end_row();
                    }
                });
            });
        };

        egui::Frame::new().fill(BG).inner_margin(10).show(ui, |ui| {
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                let w = ui.available_width();
                let t_pack = self.tr("Battery Pack Information", "ข้อมูลแพ็กแบต");
                let t_volt = self.tr("Voltage Information", "ข้อมูลแรงดัน");
                let t_temp = self.tr("Temperature Information", "ข้อมูลอุณหภูมิ");
                let t_bms = self.tr("BMS", "ข้อมูล BMS");
                if w > 900.0 {
                    ui.columns(if bms_rows.is_some() { 4 } else { 3 }, |c| {
                        card(&mut c[0], t_pack, &pack);
                        card(&mut c[1], t_volt, &volt_rows);
                        card(&mut c[2], t_temp, &temp_rows);
                        if let Some(r) = &bms_rows {
                            card(&mut c[3], t_bms, r);
                        }
                    });
                } else {
                    card(ui, t_pack, &pack);
                    ui.add_space(10.0);
                    card(ui, t_volt, &volt_rows);
                    ui.add_space(10.0);
                    card(ui, t_temp, &temp_rows);
                    if let Some(r) = &bms_rows {
                        ui.add_space(10.0);
                        card(ui, t_bms, r);
                    }
                }

                // ---- cells 1-126: accumulated balancing hours; highest / lowest voltage cell
                // (pack-wide 002C / 002A) orange / red; a voltage-sampling fault gets a red frame
                ui.add_space(14.0);
                ui.label(RichText::new(self.tr("Cells 1–126 · balancing hours", "เซลล์ 1–126 · ชั่วโมงบาลานซ์สะสม")).size(17.0).color(HEAD));
                ui.add_space(6.0);
                match &det {
                    None => {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label(RichText::new(self.tr("Reading cell data…", "กำลังอ่านข้อมูลรายเซลล์…")).color(LBL));
                        });
                    }
                    Some(d) => {
                        let cols = if w > 900.0 { 14 } else { 6 };
                        let gap = 5.0;
                        let tw = (w - gap * (cols - 1) as f32) / cols as f32;
                        let th = 50.0;
                        let n = d.balance_h.len();
                        let (area, _) = ui.allocate_exact_size(Vec2::new(w, (th + gap) * n.div_ceil(cols) as f32), Sense::hover());
                        let p = ui.painter();
                        for (i, h) in d.balance_h.iter().enumerate() {
                            let no = i as f64 + 1.0;
                            let r = Rect::from_min_size(area.min + Vec2::new((i % cols) as f32 * (tw + gap), (i / cols) as f32 * (th + gap)), Vec2::new(tw, th));
                            let is_max = fin(b.vmax_no) && b.vmax_no == no;
                            let is_min = fin(b.vmin_no) && b.vmin_no == no;
                            let bg = if is_min { REDC } else if is_max { ORANGE } else { TILE };
                            p.rect_filled(r, 6.0, bg);
                            if d.cell_bad.as_ref().is_some_and(|v| v.get(i).copied().unwrap_or(false)) {
                                p.rect_stroke(r, 6.0, Stroke::new(2.5, REDC), StrokeKind::Inside);
                            }
                            let sub = if is_min || is_max { Color32::from_white_alpha(220) } else { LBL };
                            p.text(r.center_top() + Vec2::new(0.0, 6.0), Align2::CENTER_TOP, format!("#{}", i + 1), FontId::proportional(12.0), sub);
                            let txt = if is_max {
                                volts(b.cmax)
                            } else if is_min {
                                volts(b.cmin)
                            } else if h.is_finite() {
                                format!("{h:.0} h")
                            } else {
                                "—".to_string()
                            };
                            p.text(r.center_bottom() - Vec2::new(0.0, 8.0), Align2::CENTER_BOTTOM, txt, FontId::monospace(13.0), Color32::WHITE);
                        }
                    }
                }

                // ---- history (history.csv / balance.csv)
                ui.add_space(14.0);
                ui.label(RichText::new(self.tr("History", "ประวัติแบต")).size(17.0).color(HEAD));
                ui.add_space(6.0);
                let snaps: Vec<&Snap> = self.history.iter().filter(|s| s.cap_left_pct().is_finite() || s.soh.is_finite()).collect();
                if snaps.is_empty() {
                    ui.label(RichText::new(self.tr("No history yet", "ยังไม่มีประวัติ")).color(LBL));
                } else {
                    // capacity left % (green) and SOH (blue) over the snapshots
                    let (r, _) = ui.allocate_exact_size(Vec2::new(w, 170.0), Sense::hover());
                    let p = ui.painter();
                    p.rect_filled(r, 10.0, CARD);
                    let plot = Rect::from_min_max(r.min + Vec2::new(44.0, 16.0), r.max - Vec2::new(14.0, 28.0));
                    let vals: Vec<f64> = snaps.iter().flat_map(|s| [s.cap_left_pct(), s.soh]).filter(|v| v.is_finite()).collect();
                    let lo = vals.iter().cloned().fold(f64::INFINITY, f64::min).floor() - 1.0;
                    let hi = vals.iter().cloned().fold(f64::NEG_INFINITY, f64::max).ceil() + 1.0;
                    let n = snaps.len();
                    let x = |i: usize| if n < 2 { plot.center().x } else { plot.left() + plot.width() * i as f32 / (n - 1) as f32 };
                    let y = |v: f64| plot.bottom() - plot.height() * ((v - lo) / (hi - lo)) as f32;
                    for v in [lo, (lo + hi) / 2.0, hi] {
                        p.line_segment([Pos2::new(plot.left(), y(v)), Pos2::new(plot.right(), y(v))], Stroke::new(1.0, Color32::from_gray(60)));
                        p.text(Pos2::new(plot.left() - 6.0, y(v)), Align2::RIGHT_CENTER, format!("{v:.0}%"), FontId::proportional(11.0), LBL);
                    }
                    for (sel, col) in [(0usize, Color32::from_rgb(0x4C, 0xD9, 0x2E)), (1, HEAD)] {
                        let pts: Vec<Pos2> = snaps
                            .iter()
                            .enumerate()
                            .filter_map(|(i, s)| {
                                let v = if sel == 0 { s.cap_left_pct() } else { s.soh };
                                v.is_finite().then(|| Pos2::new(x(i), y(v)))
                            })
                            .collect();
                        if pts.len() >= 2 {
                            p.add(egui::Shape::line(pts.clone(), Stroke::new(2.0, col)));
                        }
                        for q in pts {
                            p.circle_filled(q, 3.0, col);
                        }
                    }
                    p.text(Pos2::new(plot.left(), r.bottom() - 12.0), Align2::LEFT_CENTER, &snaps[0].time, FontId::proportional(11.0), LBL);
                    p.text(Pos2::new(plot.right(), r.bottom() - 12.0), Align2::RIGHT_CENTER, &snaps[n - 1].time, FontId::proportional(11.0), LBL);
                    p.text(r.right_top() + Vec2::new(-12.0, 8.0), Align2::RIGHT_TOP, format!("■ {}  ■ SOH", self.tr("capacity left", "ความจุคงเหลือ")), FontId::proportional(11.5), LBL);
                    ui.add_space(6.0);
                    // last snapshots
                    egui::Frame::new().fill(CARD).corner_radius(10).inner_margin(10).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        egui::Grid::new("hist").num_columns(5).spacing([12.0, 6.0]).show(ui, |ui| {
                            for h in [self.tr("Time", "เวลา"), self.tr("Cap.", "ความจุ"), "SOH", self.tr("Spread", "ผลต่าง"), self.tr("Charges", "ชาร์จ")] {
                                ui.label(RichText::new(h).size(13.0).color(LBL));
                            }
                            ui.end_row();
                            for s in snaps.iter().rev().take(6) {
                                let f = |v: f64, d: usize, u: &str| if v.is_finite() { format!("{v:.d$}{u}") } else { "—".to_string() };
                                ui.label(RichText::new(&s.time).size(13.0).color(Color32::WHITE));
                                ui.label(RichText::new(f(s.cap_left_pct(), 1, "%")).monospace().size(13.0).color(Color32::WHITE));
                                ui.label(RichText::new(f(s.soh, 0, "%")).monospace().size(13.0).color(Color32::WHITE));
                                ui.label(RichText::new(f(s.spread_mv, 0, " mV")).monospace().size(13.0).color(Color32::WHITE));
                                ui.label(RichText::new(f(s.charges, 0, "")).monospace().size(13.0).color(Color32::WHITE));
                                ui.end_row();
                            }
                        });
                    });
                    // cells whose balancing time grew the most
                    let growth = history::balance_growth(&self.balance_hist, 5);
                    if !growth.is_empty() {
                        let days = match (self.balance_hist.first(), self.balance_hist.last()) {
                            (Some(a), Some(b)) => (b.0 - a.0) as f64 / 86400.0,
                            _ => 0.0,
                        };
                        ui.add_space(6.0);
                        ui.label(RichText::new(format!("{} ({:.0} {})", self.tr("Most balancing added", "บาลานซ์เพิ่มมากสุด"), days, self.tr("days", "วัน"))).size(14.0).color(LBL));
                        ui.horizontal_wrapped(|ui| {
                            for (cell, dh) in growth {
                                egui::Frame::new().fill(TILE).corner_radius(6).inner_margin(egui::Margin::symmetric(10, 6)).show(ui, |ui| {
                                    ui.label(RichText::new(format!("#{cell}  +{dh:.0} h")).monospace().size(14.0).color(Color32::WHITE));
                                });
                            }
                        });
                    }
                }

                // ---- BIC groups (cell / sensor numbers are 1-based: BYD formula A + 1)
                ui.add_space(14.0);
                ui.label(RichText::new(self.tr("Cell groups (BIC 1–10)", "กลุ่มเซลล์ (BIC 1–10)")).size(17.0).color(HEAD));
                ui.add_space(6.0);
                let cols = if w > 900.0 { 5 } else { 2 };
                let gap = 8.0;
                let tw = (w - gap * (cols - 1) as f32) / cols as f32;
                let th = 96.0;
                let (area, _) = ui.allocate_exact_size(Vec2::new(w, (th + gap) * 10usize.div_ceil(cols) as f32), Sense::hover());
                let hi = self.tr("max", "สูง");
                let lo = self.tr("min", "ต่ำ");
                // which BIC holds the pack's highest / lowest cell (by value)
                let gmax = (0..10).filter(|g| b.bic_got[*g]).max_by(|x, y| b.bic[*x][3].total_cmp(&b.bic[*y][3]));
                let gmin = (0..10).filter(|g| b.bic_got[*g]).min_by(|x, y| b.bic[*x][1].total_cmp(&b.bic[*y][1]));
                let p = ui.painter();
                for g in 0..10 {
                    let r = Rect::from_min_size(area.min + Vec2::new((g % cols) as f32 * (tw + gap), (g / cols) as f32 * (th + gap)), Vec2::new(tw, th));
                    let is_max = gmax == Some(g);
                    let is_min = gmin == Some(g);
                    let bg = if is_min { REDC } else if is_max { ORANGE } else { TILE };
                    p.rect_filled(r, 8.0, bg);
                    let sub = if is_min || is_max { Color32::from_white_alpha(220) } else { LBL };
                    p.text(r.left_top() + Vec2::new(12.0, 10.0), Align2::LEFT_TOP, format!("BIC {}", g + 1), FontId::proportional(14.0), sub);
                    if !b.bic_got[g] {
                        p.text(r.center(), Align2::CENTER_CENTER, "…", FontId::proportional(22.0), sub);
                        continue;
                    }
                    let x = b.bic[g];
                    let mono = FontId::monospace(15.0);
                    p.text(r.left_top() + Vec2::new(12.0, 32.0), Align2::LEFT_TOP, format!("{hi} {:.3}  #{:.0}", x[3] / 1000.0, x[2] + 1.0), mono.clone(), Color32::WHITE);
                    p.text(r.left_top() + Vec2::new(12.0, 52.0), Align2::LEFT_TOP, format!("{lo} {:.3}  #{:.0}", x[1] / 1000.0, x[0] + 1.0), mono.clone(), Color32::WHITE);
                    p.text(r.left_top() + Vec2::new(12.0, 72.0), Align2::LEFT_TOP, format!("{:.0}–{:.0} °C", x[5], x[7]), FontId::proportional(14.0), sub);
                }
                ui.add_space(70.0);
            });
        });
    }

    fn charge_ui(&mut self, ui: &mut egui::Ui) {
        let real = self.real();
        ui.add_space(4.0);
        let Some(s) = self.charge.last().copied() else {
            ui.add_space(20.0);
            if real {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label(self.tr("Waiting for the first reading…", "รอค่าแรกจากรถ…"));
                });
            } else {
                ui.label(RichText::new(self.tr(
                    "Not connected. Connect the car's Bluetooth OBD to see live battery and charge data.",
                    "ยังไม่ได้เชื่อมต่อ — ต่อ OBD บลูทูธของรถเพื่อดูข้อมูลแบตและการชาร์จแบบสด",
                )).size(15.0).color(TEXT_DARK));
                ui.add_space(8.0);
                if ui.add(blue_button(self.tr("Go to Connect / Scan", "ไปหน้าเชื่อมต่อ / สแกน"), Vec2::new(240.0, 42.0))).clicked() {
                    self.go(Screen::Connect);
                }
            }
            return;
        };
        // ---- status (battery temperature vs the DC-charge thresholds); drawn in the DC frame
        let (col, banner) = if s.temp.is_nan() {
            (Color32::GRAY, self.tr("No battery temperature from the car", "รถไม่ส่งอุณหภูมิแบตมา").to_string())
        } else if s.temp >= DERATE_C {
            (RED, format!("{} {DERATE_C:.0}°C — {}", self.tr("At/above", "ถึง/เกิน"), self.tr("charge power is being cut (84 → 60 → 45 kW)", "กำลังชาร์จถูกลด (84 → 60 → 45 kW)")))
        } else if s.temp >= COOL_START_C {
            (Color32::from_rgb(0xF2, 0x6B, 0x1D), format!("{} {COOL_START_C:.0}°C — {}", self.tr("Above", "เกิน"), self.tr("BMS cooling should be ON; full power until 46°C", "BMS ควรเริ่มทำความเย็นแล้ว ชาร์จเต็มกำลังได้ถึง 46°C")))
        } else if s.temp >= WARN_C {
            (Color32::from_rgb(0xE0, 0xA1, 0x00), self.tr("Getting close to 39°C — start battery cooling before / while charging", "ใกล้ 39°C — ควรเริ่มทำความเย็นแบตก่อน/ระหว่างชาร์จ").to_string())
        } else {
            (GREEN, self.tr("Battery cool — full charge power available", "แบตเย็น — ชาร์จเต็มกำลังได้").to_string())
        };
        // ---- big numbers
        let nan = |v: f64, d: usize| if v.is_nan() { "—".to_string() } else { format!("{v:.d$}") };
        let power = if s.kw.is_nan() {
            "—".to_string()
        } else if s.kw > 0.5 {
            format!("{:.1}", s.kw)
        } else if s.kw < -0.5 {
            format!("−{:.1}", -s.kw)
        } else {
            "0".into()
        };
        let cards: [(&str, String, &str, String); 4] = [
            (self.tr("Battery temperature", "อุณหภูมิแบต"), nan(s.temp, 0), "°C", format!("{} {}°C", self.tr("max", "สูงสุด"), nan(s.cool, 0))),
            (self.tr("Charge power", "กำลังชาร์จ"), power, "kW", format!("{} V · {} A", nan(s.volt, 0), nan(s.amp, 1))),
            ("SOC", nan(s.soc, 1), "%", String::new()),
            (self.tr("Cell spread", "ส่วนต่างเซลล์"), nan(s.cmax - s.cmin, 0), "mV", format!("{}–{} mV", nan(s.cmin, 0), nan(s.cmax, 0))),
        ];
        ui.horizontal(|ui| {
            let w = (ui.available_width() - 3.0 * 12.0) / 4.0;
            for (i, (name, val, unit, sub)) in cards.iter().enumerate() {
                let (r, _) = ui.allocate_exact_size(Vec2::new(w, 110.0), Sense::hover());
                let p = ui.painter();
                p.rect_filled(r, 12.0, Color32::WHITE);
                p.text(r.left_top() + Vec2::new(14.0, 18.0), Align2::LEFT_CENTER, *name, FontId::proportional(14.0), Color32::GRAY);
                let c = if i == 0 { col } else { TEXT_DARK };
                let g = p.text(r.left_top() + Vec2::new(14.0, 58.0), Align2::LEFT_CENTER, val, FontId::proportional(36.0), c);
                p.text(Pos2::new(g.right() + 6.0, g.bottom() - 10.0), Align2::LEFT_BOTTOM, *unit, FontId::proportional(16.0), Color32::GRAY);
                p.text(r.left_bottom() + Vec2::new(14.0, -14.0), Align2::LEFT_CENTER, sub, FontId::proportional(12.5), Color32::GRAY);
            }
        });
        ui.add_space(8.0);
        // ---- buttons
        let mut msg = None;
        ui.horizontal(|ui| {
            let rec = if self.charge.rec.is_some() { "■ Stop recording" } else { "● Record CSV" };
            if ui.add(grey_button(rec, Vec2::new(160.0, 32.0))).clicked() {
                if self.charge.rec.is_some() {
                    msg = self.charge_rec_save();
                } else {
                    self.charge_rec_start();
                }
            }
            if ui.add(grey_button(self.tr("Clear graph", "ล้างกราฟ"), Vec2::new(120.0, 32.0))).clicked() {
                self.charge.hist.clear();
                self.charge.t0 = None;
            }
        });
        if let Some(m) = msg {
            self.toast(m);
        }

        // ---- AC and DC charging in their own frames; the one charging now goes first
        let ac_now = self.charge.obc.is_some_and(|o| o.state == 2.0);
        let dc_now = !ac_now && s.kw > 3.0;
        let draw_ac = |app: &mut Self, ui: &mut egui::Ui| {
            let title = app.tr("AC charging · on-board charger (OBC)", "ชาร์จ AC · ตัวชาร์จในรถ (OBC)").to_string();
            charge_section(ui, &title, Color32::from_rgb(0x14, 0xA3, 0x8B), Color32::from_rgb(0xE8, 0xF7, 0xF4), ac_now, app.tr("charging", "กำลังชาร์จ"), |ui| {
                if app.charge.obc.is_some() {
                    app.obc_panel(ui);
                } else {
                    ui.label(RichText::new(app.tr("Waiting for the on-board charger…", "รอค่าจากตัวชาร์จในรถ…")).color(Color32::GRAY));
                }
            });
        };
        let draw_dc = |app: &mut Self, ui: &mut egui::Ui| {
            let title = app.tr("DC charging · fast charger", "ชาร์จ DC · ตู้ชาร์จเร็ว").to_string();
            charge_section(ui, &title, Color32::from_rgb(0xF2, 0x71, 0x1C), Color32::from_rgb(0xFF, 0xF2, 0xE8), dc_now, app.tr("charging", "กำลังชาร์จ"), |ui| {
                app.dc_banner(ui, &s, col, &banner);
                ui.add_space(8.0);
                app.thermal_request_panel(ui);
                ui.add_space(8.0);
                app.charge_graph(ui);
            });
        };
        ui.add_space(10.0);
        if ac_now {
            draw_ac(self, ui);
            ui.add_space(12.0);
            draw_dc(self, ui);
        } else {
            draw_dc(self, ui);
            ui.add_space(12.0);
            draw_ac(self, ui);
        }
        ui.add_space(70.0);
    }

    /// Battery-temperature status against the DC-charge thresholds (39 / 46 °C) + trend.
    fn dc_banner(&self, ui: &mut egui::Ui, s: &ChargeSample, col: Color32, msg: &str) {
        egui::Frame::new().fill(col).corner_radius(10).inner_margin(12).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(msg).size(17.0).strong().color(Color32::WHITE));
            if let (Some(k), false) = (self.charge.slope(), s.temp.is_nan()) {
                let eta = if k > 0.02 && s.temp < DERATE_C {
                    format!("  ·  {} {DERATE_C:.0}°C ≈ {:.0} {}", self.tr("at this rate reaches", "ด้วยอัตรานี้จะถึง"), (DERATE_C - s.temp) / k, self.tr("min", "นาที"))
                } else {
                    String::new()
                };
                ui.label(RichText::new(format!("{:+.2} °C/min{eta}", k)).size(13.5).color(Color32::WHITE));
            }
        });
    }

    fn charge_graph(&self, ui: &mut egui::Ui) {
        // fixed height: the graph sits inside the DC frame with the AC frame below it
        let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 320.0), Sense::hover());
        let p = ui.painter();
        p.rect_filled(rect, 10.0, Color32::WHITE);
        let plot = Rect::from_min_max(rect.min + Vec2::new(48.0, 30.0), rect.max - Vec2::new(48.0, 26.0));
        let h = &self.charge.hist;
        if h.len() < 2 {
            return;
        }
        let (t0, t1) = (h.front().unwrap().t, h.back().unwrap().t.max(h.front().unwrap().t + 60.0));
        let x = |t: f64| plot.left() + plot.width() * ((t - t0) / (t1 - t0)) as f32;
        let (tmin, tmax) = (20.0, 55.0);
        let yt = |v: f64| plot.bottom() - plot.height() * (((v - tmin) / (tmax - tmin)).clamp(0.0, 1.0)) as f32;
        let yk = |v: f64| plot.bottom() - plot.height() * ((v / 100.0).clamp(0.0, 1.0)) as f32;
        for v in [20.0, 30.0, 40.0, 50.0] {
            p.line_segment([Pos2::new(plot.left(), yt(v)), Pos2::new(plot.right(), yt(v))], Stroke::new(1.0, Color32::from_gray(232)));
            p.text(Pos2::new(plot.left() - 6.0, yt(v)), Align2::RIGHT_CENTER, format!("{v:.0}°"), FontId::proportional(11.0), Color32::GRAY);
        }
        for k in [0.0, 50.0, 100.0] {
            p.text(Pos2::new(plot.right() + 6.0, yk(k)), Align2::LEFT_CENTER, format!("{k:.0} kW"), FontId::proportional(11.0), GRAPH_COLORS[0]);
        }
        for (v, c) in [(COOL_START_C, Color32::from_rgb(0xF2, 0x6B, 0x1D)), (DERATE_C, RED)] {
            let y = yt(v);
            let mut xx = plot.left();
            while xx < plot.right() {
                p.line_segment([Pos2::new(xx, y), Pos2::new((xx + 8.0).min(plot.right()), y)], Stroke::new(1.5, c));
                xx += 14.0;
            }
            p.text(Pos2::new(plot.left() + 4.0, y - 8.0), Align2::LEFT_CENTER, format!("{v:.0}°C"), FontId::proportional(11.0), c);
        }
        for w in h.iter().collect::<Vec<_>>().windows(2) {
            if w[0].forced {
                let r = Rect::from_min_max(Pos2::new(x(w[0].t), plot.top()), Pos2::new(x(w[1].t), plot.bottom()));
                p.rect_filled(r, 0.0, Color32::from_rgba_unmultiplied(0x28, 0xC9, 0xE3, 28));
            }
        }
        let line = |f: &dyn Fn(&ChargeSample) -> f64, y: &dyn Fn(f64) -> f32| -> Vec<Pos2> {
            h.iter().filter(|s| !f(s).is_nan()).map(|s| Pos2::new(x(s.t), y(f(s)))).collect()
        };
        p.add(egui::Shape::line(line(&|s| s.kw.abs(), &yk), Stroke::new(2.0, GRAPH_COLORS[0])));
        p.add(egui::Shape::line(line(&|s| s.temp, &yt), Stroke::new(2.5, GRAPH_COLORS[1])));
        // legend laid out left to right, wrapping to a second line on a narrow (phone) graph
        let items = [
            (self.tr("Battery °C", "อุณหภูมิแบต °C"), GRAPH_COLORS[1], 4.0),
            (self.tr("Charge power kW", "กำลังชาร์จ kW"), GRAPH_COLORS[0], 4.0),
            (self.tr("Cooling on", "ทำความเย็นอยู่"), Color32::from_rgba_unmultiplied(0x28, 0xC9, 0xE3, 70), 12.0),
        ];
        let (mut x, mut y) = (rect.left() + 48.0, rect.top() + 14.0);
        for (txt, col, h) in items {
            let font = FontId::proportional(12.5);
            let tw = p.layout_no_wrap(txt.to_string(), font.clone(), TEXT_DARK).size().x;
            if x + 20.0 + tw > rect.right() - 8.0 {
                x = rect.left() + 48.0;
                y += 16.0;
            }
            p.rect_filled(Rect::from_center_size(Pos2::new(x + 7.0, y), Vec2::new(14.0, h)), 2.0, col);
            let r = p.text(Pos2::new(x + 20.0, y), Align2::LEFT_CENTER, txt, font, TEXT_DARK);
            x = r.right() + 18.0;
        }
        let mins = (t1 - t0) / 60.0;
        p.text(Pos2::new(plot.right(), rect.bottom() - 12.0), Align2::RIGHT_CENTER, format!("{mins:.0} min"), FontId::proportional(11.0), Color32::GRAY);
    }

    fn set_pids(&mut self, pids: Vec<Pid>) {
        let n = pids.len();
        self.live.ranges = pids.iter().map(|p| (p.min, p.max)).collect();
        self.live.selected = vec![false; n]; // dashboard shows all; tap cards to pick graph values
        self.live.values = vec![f64::NAN; n];
        self.live.raw = vec![(0, 0); n];
        self.live.history = vec![VecDeque::new(); n];
        self.live.recording = None;
        self.live.pids = pids;
        self.live_req.clear();
    }


    fn header(&mut self, ui: &mut egui::Ui) {
        let rect = ui.max_rect();
        gradient_rect(ui.painter(), rect, HEAD_TOP, HEAD_BOTTOM);
        // No text in the header (user request): only Back, a link-status dot and the language chip.
        ui.horizontal_centered(|ui| {
            ui.add_space(8.0);
            if self.stack.len() > 1 {
                let b = egui::Button::new(RichText::new("‹  ".to_string() + self.tr("Back", "กลับ")).color(Color32::WHITE).size(15.0))
                    .fill(Color32::from_white_alpha(40))
                    .corner_radius(CornerRadius::same(14));
                if ui.add(b).clicked() {
                    self.back();
                }
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                ui.add_space(8.0);
                let lang = if self.lang == Lang::En { "ไทย" } else { "EN" };
                if ui.add(chip(lang)).clicked() {
                    self.lang = if self.lang == Lang::En { Lang::Th } else { Lang::En };
                }
                ui.add_space(6.0);
                // link status as a colour only: green connected, amber connecting, grey not connected
                let col = match &self.link.state {
                    LinkState::Connected(_) => Color32::from_rgb(0x6B, 0xF1, 0x8A),
                    LinkState::Searching => Color32::from_rgb(0xFF, 0xDE, 0x7A),
                    _ => Color32::from_white_alpha(110),
                };
                let (dot, _) = ui.allocate_exact_size(Vec2::splat(14.0), Sense::hover());
                ui.painter().circle_filled(dot.center(), 6.0, col);
            });
        });
    }

    /// Shown for a car-dependent screen when no OBD is connected. No demo/simulated data.
    fn not_connected(&mut self, ui: &mut egui::Ui) {
        ui.vertical_centered(|ui| {
            ui.add_space(70.0);
            let (r, _) = ui.allocate_exact_size(Vec2::new(136.0, 92.0), Sense::hover());
            draw_obd_connector(ui.painter(), r);
            ui.add_space(14.0);
            ui.label(RichText::new(self.tr("Not connected", "ยังไม่ได้เชื่อมต่อ")).size(24.0).strong().color(TEXT_DARK));
            ui.add_space(8.0);
            ui.label(RichText::new(self.tr(
                "Connect the car's Bluetooth OBD to see live data here.",
                "ต่อ OBD บลูทูธกับรถ เพื่อดูข้อมูลสดที่นี่",
            )).size(16.0).color(Color32::GRAY));
            ui.add_space(22.0);
            if ui.add(blue_button(self.tr("Connect / Scan", "เชื่อมต่อ / สแกน"), Vec2::new(260.0, 50.0))).clicked() {
                self.back();
            }
        });
    }

    fn connect(&mut self, ui: &mut egui::Ui) {
        // Car head unit: the whole home page fits the screen (no scroll; tiles flex), and the
        // UI scale is derived from it (see CAR_HOME_H). Phone: scrolls as before.
        if car_mode() {
            self.connect_body(ui, true);
        } else {
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| self.connect_body(ui, false));
        }
    }

    fn connect_body(&mut self, ui: &mut egui::Ui, car: bool) {
        {
            let w = ui.available_width();
            ui.add_space(10.0);
            // ---- hero card: brand colour (taken from the logo's own background) + logo
            let (tex, brand) = self.logo(ui.ctx());
            let (hero, _) = ui.allocate_exact_size(Vec2::new(w, 148.0), Sense::hover());
            ui.painter().rect_filled(hero, 18.0, brand);
            {
                let p = ui.painter();
                let ls = 128.0;
                let lr = Rect::from_center_size(Pos2::new(hero.right() - 16.0 - ls / 2.0, hero.center().y), Vec2::splat(ls));
                p.image(tex.id(), lr, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
                p.text(hero.min + Vec2::new(24.0, 22.0), Align2::LEFT_TOP, "VoltaChill", FontId::proportional(36.0), Color32::WHITE);
                p.text(hero.min + Vec2::new(26.0, 74.0), Align2::LEFT_TOP, self.tr("BYD diagnostic tool", "เครื่องมือวินิจฉัย BYD"), FontId::proportional(18.0), Color32::from_white_alpha(225));
                let (stxt, scol) = match &self.link.state {
                    LinkState::Connected(n) => (format!("●  {} {n}", self.tr("Connected", "เชื่อมต่อแล้ว")), Color32::from_rgb(0x8C, 0xF5, 0xB0)),
                    LinkState::Searching => (self.tr("◌  connecting…", "◌  กำลังเชื่อมต่อ…").to_string(), Color32::from_rgb(0xFF, 0xDE, 0x7A)),
                    LinkState::Failed(_) => (self.tr("●  not connected", "●  ยังไม่เชื่อมต่อ").to_string(), Color32::from_rgb(0xFF, 0xC9, 0xC9)),
                    LinkState::Off => (self.tr("●  not connected", "●  ยังไม่เชื่อมต่อ").to_string(), Color32::from_white_alpha(235)),
                };
                p.text(hero.min + Vec2::new(26.0, 110.0), Align2::LEFT_TOP, stxt, FontId::proportional(16.0), scol);
            }

            // ---- Bluetooth devices (the main action)
            ui.add_space(14.0);
            let busy = self.bt_scanning || self.link.state == LinkState::Searching;
            let mut connect_id: Option<String> = None;
            // device list only until connected (then the function tiles take the screen)
            if !self.link.connected() {
            egui::Frame::new().fill(Color32::WHITE).corner_radius(16).inner_margin(16).show(ui, |ui| {
                ui.set_width(ui.available_width());
                ui.horizontal(|ui| {
                    ui.label(RichText::new(self.tr("Bluetooth OBD", "อุปกรณ์ OBD บลูทูธ")).strong().size(21.0).color(TEXT_DARK));
                    if self.bt_scanning {
                        ui.add_space(8.0);
                        ui.spinner();
                    }
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        let label = if self.bt_devices.is_empty() { self.tr("🔍 Scan", "🔍 สแกน") } else { self.tr("🔄 Rescan", "🔄 สแกนใหม่") };
                        if ui.add_enabled(!busy, blue_button(label, Vec2::new(140.0, 46.0))).clicked() {
                            self.bt_scanning = true;
                            self.bt_devices.clear();
                            self.link.send(Cmd::ScanDevices);
                        }
                    });
                });
                ui.add_space(6.0);
                if self.bt_devices.is_empty() {
                    let msg = if self.bt_scanning {
                        self.tr("Scanning for nearby adapters…", "กำลังค้นหาอุปกรณ์ใกล้ๆ…")
                    } else {
                        self.tr("Press Scan. Turn the ELM327 on and keep it close.", "กดสแกน เปิด ELM327 แล้ววางไว้ใกล้ๆ")
                    };
                    ui.label(RichText::new(msg).size(15.0).color(Color32::GRAY));
                }
                egui::ScrollArea::vertical().max_height(if car { 140.0 } else { 380.0 }).id_salt("devlist").show(ui, |ui| {
                    for d in &self.bt_devices {
                        ui.add_space(8.0);
                        let (r, resp) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 62.0), Sense::click());
                        let hover = resp.hovered() && !busy;
                        // state of the device the user tapped: 0 none, 1 connecting, 2 connected, 3 failed
                        let st = if self.bt_target.as_deref() == Some(d.id.as_str()) {
                            match self.link.state {
                                LinkState::Searching => 1,
                                LinkState::Connected(_) => 2,
                                LinkState::Failed(_) => 3,
                                LinkState::Off => 0,
                            }
                        } else {
                            0
                        };
                        let bg = if st == 1 {
                            Color32::from_rgb(0xFF, 0xF4, 0xD6)
                        } else if st == 2 {
                            Color32::from_rgb(0xD5, 0xF5, 0xDF)
                        } else if st == 3 {
                            Color32::from_rgb(0xFD, 0xE2, 0xE2)
                        } else if d.obd {
                            Color32::from_rgb(0xEA, 0xF7, 0xEE)
                        } else if hover {
                            Color32::from_rgb(0xE7, 0xF0, 0xFD)
                        } else {
                            Color32::from_rgb(0xF4, 0xF6, 0xFA)
                        };
                        ui.painter().rect_filled(r, 12.0, bg);
                        let pad = 16.0;
                        if d.obd {
                            let badge = Rect::from_min_size(r.min + Vec2::new(pad, 20.0), Vec2::new(54.0, 22.0));
                            ui.painter().rect_filled(badge, 6.0, GREEN);
                            ui.painter().text(badge.center(), Align2::CENTER_CENTER, "OBD", FontId::proportional(13.0), Color32::WHITE);
                        }
                        let tx = r.min.x + if d.obd { pad + 66.0 } else { pad };
                        let nm: &str = if d.name.is_empty() || d.name == "(unnamed)" { self.tr("(no name)", "(ไม่มีชื่อ)") } else { &d.name };
                        ui.painter().text(Pos2::new(tx, r.min.y + 12.0), Align2::LEFT_TOP, nm, FontId::proportional(18.0), TEXT_DARK);
                        ui.painter().text(Pos2::new(tx, r.min.y + 37.0), Align2::LEFT_TOP, &d.id, FontId::monospace(13.0), Color32::GRAY);
                        let (act, act_col) = match st {
                            1 => (self.tr("Connecting…", "กำลังเชื่อมต่อ…"), Color32::from_rgb(0xB2, 0x6A, 0x00)),
                            2 => (self.tr("● Connected", "● เชื่อมต่อแล้ว"), GREEN),
                            3 => (self.tr("Failed · tap to retry", "ไม่สำเร็จ · แตะลองใหม่"), Color32::from_rgb(0xC6, 0x28, 0x28)),
                            _ if busy => (self.tr("Connect ›", "เชื่อมต่อ ›"), Color32::GRAY),
                            _ => (self.tr("Connect ›", "เชื่อมต่อ ›"), ACCENT),
                        };
                        let right = r.right_center() - Vec2::new(14.0, 0.0);
                        let tr = ui.painter().text(right, Align2::RIGHT_CENTER, act, FontId::proportional(16.0), act_col);
                        if st == 1 {
                            // spinner left of the label; keep repainting while it turns
                            let c = Pos2::new(tr.min.x - 18.0, r.center().y);
                            let t = ui.input(|i| i.time) as f32;
                            let p = ui.painter();
                            for k in 0..8 {
                                let a = t * 6.0 + k as f32 * std::f32::consts::TAU / 8.0;
                                let alpha = (40 + k * 26) as u8;
                                p.circle_filled(c + Vec2::new(a.cos(), a.sin()) * 8.0, 2.4, Color32::from_rgba_unmultiplied(0xB2, 0x6A, 0x00, alpha));
                            }
                            ui.ctx().request_repaint();
                        }
                        if resp.clicked() && !busy && st != 2 {
                            connect_id = Some(d.id.clone());
                        }
                    }
                });
                // why the last attempt failed, under the list
                if let (Some(_), LinkState::Failed(why)) = (&self.bt_target, &self.link.state) {
                    ui.add_space(8.0);
                    ui.label(RichText::new(format!("{}: {why}", self.tr("Could not connect", "เชื่อมต่อไม่ได้"))).size(14.0).color(Color32::from_rgb(0xC6, 0x28, 0x28)));
                }
            });
            }
            if let Some(id) = connect_id {
                // immediate feedback: the worker may take seconds before it reports back
                self.bt_target = Some(id.clone());
                self.link.state = LinkState::Searching;
                self.link.send(Cmd::ConnectTo(id));
            }

            // ---- all functions: mosaic of flat tiles (some double width), reachable any time
            ui.add_space(14.0);
            // (label_en, label_th, target (None = Auto Scan), colour, draw_icon kind)
            let all: [(&str, &str, Option<Screen>, u32, u8); 9] = [
                ("Trouble Codes", "รหัสปัญหา", Some(Screen::Codes), 0x1546B5, 7),
                ("Live Data", "ข้อมูลสด", Some(Screen::LiveData), 0x2D7FE6, 2),
                ("Charge & Cool", "ชาร์จ & ทำความเย็น", Some(Screen::Charge), 0xC8331B, 8),
                ("Battery", "แบตเตอรี่", Some(Screen::Battery), 0x4FA32A, 6),
                ("Full check", "ตรวจรถทั้งคัน", None, 0x16848F, 0),
                ("VIN", "VIN รถ", Some(Screen::VinInput), 0xA3124A, 1),
                ("Report", "รายงาน", Some(Screen::Reports), 0x6A4FA8, 3),
                ("Bus Monitor", "ดักฟังบัส", Some(Screen::Sniff), 0x5A2DB5, 9),
                ("Settings", "ตั้งค่า", Some(Screen::Settings), 0x4A5568, 255),
            ];
            // Car: only Live Data, Charge & Cool, Battery, Full check (user cut the rest 2026-11-09).
            let items: Vec<_> = if car {
                all.iter().copied().filter(|it| matches!(it.2, Some(Screen::LiveData) | Some(Screen::Charge) | Some(Screen::Battery) | None)).collect()
            } else {
                all.to_vec()
            };
            // row layouts in thirds: 1 = narrow tile, 2 = double-width tile
            let rows: &[&[u8]] = if car { &[&[1, 2], &[2, 1]] } else { &[&[1, 2], &[2, 1], &[1, 1, 1], &[2, 1]] };
            let gap = 8.0;
            let total = ui.available_width();
            let third = (total - 2.0 * gap) / 3.0;
            let th = if car {
                // fill what is left: 4 tile rows + (when connected) disconnect + chain + credit
                let below = 10.0 + 22.0 + if self.link.connected() { 10.0 + 52.0 + 14.0 + 56.0 } else { 0.0 };
                ((ui.available_height() - below - rows.len() as f32 * gap) / rows.len() as f32).max(56.0)
            } else if total > 820.0 {
                120.0
            } else {
                132.0
            };
            let en = self.lang == Lang::En;
            let mut do_scan = false;
            let mut goto: Option<Screen> = None;
            let mut k = 0;
            for row in rows.iter() {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = gap;
                    for span in row.iter() {
                        let Some((le, lt, scr, col, icon)) = items.get(k) else { break };
                        k += 1;
                        let w = third * *span as f32 + gap * (*span as f32 - 1.0);
                        let (rect, resp) = ui.allocate_exact_size(Vec2::new(w, th), Sense::click());
                        let base = rgb(*col);
                        let [r, g, b, _] = base.to_array();
                        let lite = Color32::from_rgb(r.saturating_add(26), g.saturating_add(26), b.saturating_add(26));
                        let p = ui.painter();
                        p.rect_filled(rect.translate(Vec2::new(0.0, 3.0)), 6.0, Color32::from_black_alpha(35));
                        let rr = if resp.is_pointer_button_down_on() { rect.shrink(2.0) } else { rect };
                        rounded_gradient(p, rr, 6.0, lite, base);
                        draw_icon(p, *icon, Pos2::new(rr.center().x, rr.top() + th * 0.36), 20.0);
                        let label = if en { *le } else { *lt };
                        let mut job = egui::text::LayoutJob::simple(label.to_string(), FontId::proportional(16.0), Color32::WHITE, rr.width() - 16.0);
                        job.halign = Align::Center;
                        let galley = ui.fonts_mut(|f| f.layout_job(job));
                        p.galley(Pos2::new(rr.center().x, rr.top() + th * 0.62), galley, Color32::WHITE);
                        if resp.clicked() {
                            match scr {
                                Some(s) => goto = Some(*s),
                                None => do_scan = true,
                            }
                        }
                    }
                });
                ui.add_space(gap);
            }
            // ---- disconnect + App — OBD — Vehicle chain (when connected)
            if self.link.connected() {
                ui.add_space(10.0);
                let bw = ui.available_width();
                let (br, bresp) = ui.allocate_exact_size(Vec2::new(bw, 52.0), Sense::click());
                let red = if bresp.is_pointer_button_down_on() { Color32::from_rgb(0xC4, 0x12, 0x12) } else { Color32::from_rgb(0xE5, 0x1B, 0x1B) };
                ui.painter().rect_filled(br, 26.0, red);
                ui.painter().text(br.center(), Align2::CENTER_CENTER, self.tr("Disconnect", "ตัดการเชื่อมต่อ"), FontId::proportional(17.0), Color32::WHITE);
                if bresp.clicked() {
                    self.bt_target = None;
                    self.link.send(Cmd::Disconnect);
                }
                ui.add_space(14.0);
                let (cr, _) = ui.allocate_exact_size(Vec2::new(bw, 56.0), Sense::hover());
                let p = ui.painter();
                let on = Color32::from_rgb(0x1E, 0x4F, 0xC8);
                let off = Color32::from_gray(190);
                let veh = !self.live.pids.is_empty();
                let nodes = [(self.tr("App", "แอป"), true), ("OBD", true), (self.tr("Vehicle", "รถ"), veh)];
                let xs = [cr.left() + 30.0, cr.center().x, cr.right() - 30.0];
                let y = cr.top() + 16.0;
                for k in 0..2 {
                    let c = if nodes[k + 1].1 { on } else { off };
                    p.line_segment([Pos2::new(xs[k] + 16.0, y), Pos2::new(xs[k + 1] - 16.0, y)], Stroke::new(3.0, c));
                }
                for (k, (lbl, ok)) in nodes.iter().enumerate() {
                    let c = if *ok { on } else { off };
                    p.circle_stroke(Pos2::new(xs[k], y), 12.0, Stroke::new(2.0, c));
                    p.circle_filled(Pos2::new(xs[k], y), 5.0, c);
                    p.text(Pos2::new(xs[k], y + 26.0), Align2::CENTER_CENTER, *lbl, FontId::proportional(13.0), c);
                }
            }
            if do_scan {
                self.start_check();
            }
            if let Some(s) = goto {
                self.go(s);
            }
            ui.add_space(10.0);
            ui.vertical_centered(|ui| {
                ui.label(RichText::new(format!("{} M1TS", self.tr("Developed by", "พัฒนาโดย"))).size(13.0).color(Color32::GRAY));
            });
            if !car {
                ui.add_space(56.0); // keep the credit clear of the system navigation bar
            }
        }
    }

    /// Keep the OBD worker's background monitor in step: on while a charge recording is open, so
    /// it keeps reading / recording (and the Android foreground service runs) when the screen is
    /// off or the app is in the back.
    fn bg_sync(&mut self) {
        let want = self.real() && self.charge.rec.is_some();
        let path = if want { self.charge.rec.clone() } else { None };
        if (want, path.clone()) != self.bg_sent {
            self.link.send(Cmd::BgMonitor { on: want, csv: path.as_ref().map(|p| p.display().to_string()), req: charge_req() });
            self.bg_sent = (want, path);
        }
    }

    /// Start the full vehicle check (VIN, trouble codes, battery, battery detail, OBC + 12 V).
    fn start_check(&mut self) {
        self.check = Some(CheckState { started: clock_text(), ..Default::default() });
        self.batt = BattState::default();
        self.batt_detail = None;
        self.charge.obc = None;
        self.systems.clear();
        self.go(Screen::Check);
    }

    /// Send the next check step once the previous answer is in (nothing else is pending).
    fn check_tick(&mut self) {
        let idle = !self.link.busy
            && !self.vin_reading
            && !self.dtc_reading
            && !self.batt_pending
            && !self.batt_detail_pending
            && !self.charge_pending
            && !self.check_live_pending;
        let real = self.real();
        let Some(c) = &mut self.check else { return };
        if c.done || !real || !idle {
            return;
        }
        if c.waiting {
            c.waiting = false;
            c.step += 1;
        }
        if c.step >= CHECK_STEPS {
            c.done = true;
            self.check_report();
            return;
        }
        c.waiting = true;
        match c.step {
            0 => {
                self.vin_reading = true;
                self.link.send(Cmd::ReadVin);
            }
            1 => {
                self.codes_read = true;
                self.dtc_reading = true;
                self.link.send(Cmd::ReadDtc);
            }
            2 => {
                self.batt_pending = true;
                let req = BATT_DIDS.iter().map(|d| (0u8, Src::Did { hdr: 0x7E7, did: *d })).collect();
                self.link.send(Cmd::Live(req));
            }
            3 => {
                self.batt_detail_pending = true;
                self.batt_detail_t = self.now;
                self.link.send(batt_detail_req());
            }
            _ => {
                // OBC + PDC values, then the 12 V battery (adapter ATRV)
                if self.charge.obc.is_none() {
                    self.charge_pending = true;
                    self.link.send(Cmd::Live(charge_req()));
                    c.waiting = false; // come back for the 12 V read
                } else {
                    self.check_live_pending = true;
                    self.link.send(Cmd::Live(vec![(0, Src::Volt12)]));
                }
            }
        }
    }

    /// (level 0 ok / 1 attention / 2 problem, item, value) for every check line.
    fn check_findings(&self) -> Vec<(u8, String, String)> {
        let mut out = Vec::new();
        let tr = |en: &str, th: &str| self.tr(en, th).to_string();
        // VIN
        let vin = self.vin_text.trim().to_uppercase();
        let vin_v = match vin::decode_vin(&vin) {
            Some(i) => format!("{vin} · {} {}", i.maker, i.year.map(|y| y.to_string()).unwrap_or_default()),
            None if vin.is_empty() => "—".into(),
            None => vin.clone(),
        };
        out.push((if vin.len() == 17 { 0 } else { 1 }, tr("VIN", "VIN"), vin_v));
        // trouble codes
        let total: usize = self.systems.iter().map(|s| s.dtcs.len()).sum();
        let cur = |soft: bool| {
            self.systems
                .iter()
                .filter(|s| soft_module(s.name) == soft)
                .flat_map(|s| s.dtcs.iter())
                .filter(|d| matches!(d.status, DtcStatus::Current))
                .count()
        };
        let (current, current_soft) = (cur(false), cur(true));
        let with = self.systems.iter().filter(|s| !s.dtcs.is_empty()).count();
        // a current code in a multimedia-type module is only "attention"
        let lvl = if current > 0 { 2 } else if total > 0 { 1 } else { 0 };
        let mut v = format!("{total} {} · {current} {}", tr("codes", "รหัส"), tr("current", "ปัจจุบัน"));
        if current_soft > 0 {
            v += &format!(" (+{current_soft} {})", tr("multimedia", "มัลติมีเดีย"));
        }
        v += &format!(" · {with}/{} {}", self.systems.len(), tr("modules", "โมดูล"));
        out.push((lvl, tr("Trouble codes", "รหัสปัญหา"), v));
        // high-voltage battery
        let b = self.batt;
        if b.got {
            out.push((if b.soh.is_finite() && b.soh < 80.0 { 1 } else { 0 }, "SOH".into(), format!("{:.0} %", b.soh)));
            let spread = b.cmax - b.cmin;
            out.push((
                if spread.is_finite() && spread > 50.0 { 1 } else { 0 },
                tr("Cell voltage spread", "ผลต่างแรงดันเซลล์"),
                format!("{spread:.0} mV (#{:.0} {:.3} V · #{:.0} {:.3} V)", b.vmax_no, b.cmax / 1000.0, b.vmin_no, b.cmin / 1000.0),
            ));
            out.push((
                if (b.tmax - b.tmin).is_finite() && b.tmax - b.tmin > 8.0 { 1 } else { 0 },
                tr("Cell temperature", "อุณหภูมิเซลล์"),
                format!("{:.0}–{:.0} °C", b.tmin, b.tmax),
            ));
        }
        if let Some(d) = &self.batt_detail {
            let left = d.cap_now / d.cap_factory * 100.0;
            out.push((
                if left.is_finite() && left < 70.0 { 1 } else { 0 },
                tr("Capacity left", "ความจุคงเหลือ"),
                format!("{left:.1} % ({:.1} / {:.1} Ah)", d.cap_now, d.cap_factory),
            ));
            let bad = d.cell_bad.as_ref().map(|v| v.iter().filter(|x| **x).count());
            out.push((if bad.unwrap_or(0) > 0 { 2 } else { 0 }, tr("Cell sampling faults", "เซลล์วัดค่าผิดปกติ"), bad.map(|n| n.to_string()).unwrap_or("—".into())));
            out.push((if d.temp_bad.unwrap_or(0) > 0 { 2 } else { 0 }, tr("Temperature sensor faults", "เซนเซอร์อุณหภูมิผิดปกติ"), d.temp_bad.map(|n| n.to_string()).unwrap_or("—".into())));
            out.push((if d.insul_kohm.is_finite() && d.insul_kohm < 500.0 { 2 } else { 0 }, tr("Insulation resistance", "ความต้านทานฉนวน"), format!("{:.0} kΩ", d.insul_kohm)));
            out.push((0, tr("Lifetime energy", "พลังงานสะสม"), format!("{} {:.0} kWh · {} {:.0} kWh", tr("in", "ชาร์จเข้า"), d.chg_kwh, tr("out", "ใช้ไป"), d.dis_kwh)));
        }
        // 12 V and OBC
        if let Some(v) = self.check.as_ref().and_then(|c| c.v12) {
            out.push((if v < 11.8 { 2 } else if v < 12.2 { 1 } else { 0 }, tr("12 V battery", "แบต 12V"), format!("{v:.1} V")));
        }
        if let Some(o) = self.charge.obc {
            out.push((0, tr("On-board charger", "ตัวชาร์จในรถ (OBC)"), format!("{:.0} °C · {}", o.max_temp(), self.obc_state_text(o.state))));
        }
        out
    }

    /// Store the finished check in Reports.
    fn check_report(&mut self) {
        let f = self.check_findings();
        let worst = f.iter().map(|x| x.0).max().unwrap_or(0);
        let verdict = |l: u8| match l {
            0 => "OK",
            1 => "Attention",
            _ => "Problem",
        };
        let rows = f.iter().map(|(l, k, v)| (k.clone(), v.clone(), String::new(), verdict(*l).to_string())).collect();
        let started = self.check.as_ref().map(|c| c.started.clone()).unwrap_or_default();
        self.reports.insert(
            0,
            Report {
                time: started,
                vin: self.vin_text.trim().to_uppercase(),
                vehicle: self.tr("Full vehicle check", "ตรวจรถทั้งคัน").to_string(),
                summary: format!("{}: {}", self.tr("Result", "ผล"), verdict(worst)),
                rows,
            },
        );
        self.save_reports();
        let v12 = self.check.as_ref().and_then(|c| c.v12).unwrap_or(f64::NAN);
        self.record_snapshot("check", v12);
    }

    /// Keep the Reports list on disk (newest 50).
    fn save_reports(&mut self) {
        self.reports.truncate(50);
        let recs: Vec<history::ReportRec> = self.reports.iter().map(|r| (r.time.clone(), r.vin.clone(), r.vehicle.clone(), r.summary.clone(), r.rows.clone())).collect();
        let _ = history::save_reports(&out_dir(), &recs);
    }

    /// Append the current battery state to the history files ("check" or "battery").
    fn record_snapshot(&mut self, source: &str, v12: f64) {
        let b = self.batt;
        let d = self.batt_detail.as_ref();
        let nan = f64::NAN;
        let dtc = self.codes_read && source == "check";
        let snap = Snap {
            unix: history::now_unix(),
            time: clock_text(),
            vin: self.vin_text.trim().to_uppercase(),
            source: source.into(),
            soh: if b.got { b.soh } else { nan },
            cap_now: d.map(|d| d.cap_now).unwrap_or(nan),
            cap_factory: d.map(|d| d.cap_factory).unwrap_or(nan),
            spread_mv: if b.got { b.cmax - b.cmin } else { nan },
            tmin: if b.got { b.tmin } else { nan },
            tmax: if b.got { b.tmax } else { nan },
            insul_kohm: d.map(|d| d.insul_kohm).unwrap_or(nan),
            chg_kwh: d.map(|d| d.chg_kwh).unwrap_or(nan),
            dis_kwh: d.map(|d| d.dis_kwh).unwrap_or(nan),
            charges: if b.got { b.cycles } else { nan },
            v12,
            dtc_total: if dtc { self.systems.iter().map(|s| s.dtcs.len()).sum::<usize>() as f64 } else { nan },
            dtc_current: if dtc {
                self.systems.iter().flat_map(|s| s.dtcs.iter()).filter(|x| matches!(x.status, DtcStatus::Current)).count() as f64
            } else {
                nan
            },
            cell_faults: d.and_then(|d| d.cell_bad.as_ref()).map(|v| v.iter().filter(|x| **x).count() as f64).unwrap_or(nan),
        };
        let bal = d.map(|d| d.balance_h.clone());
        if history::append(&out_dir(), &snap, bal.as_deref()).is_ok() {
            if let Some(b) = bal {
                self.balance_hist.push((snap.unix, b));
            }
            self.history.push(snap);
        }
    }

    fn check_ui(&mut self, ui: &mut egui::Ui) {
        let Some(c) = &self.check else { return };
        let (step, done) = (c.step, c.done);
        let w = ui.available_width();
        ui.add_space(8.0);
        let steps = [
            self.tr("VIN", "อ่าน VIN"),
            self.tr("Trouble codes (all modules)", "รหัสปัญหา (ทุกโมดูล)"),
            self.tr("High-voltage battery", "แบตแรงสูง"),
            self.tr("Battery cells & capacity", "เซลล์และความจุแบต"),
            self.tr("12 V battery & on-board charger", "แบต 12V และตัวชาร์จ"),
        ];
        let findings = if done { self.check_findings() } else { Vec::new() };
        let worst = findings.iter().map(|x| x.0).max().unwrap_or(0);
        // summary card
        let (top, bot, title) = if !done {
            (Color32::from_rgb(0x1A, 0x6F, 0xE8), Color32::from_rgb(0x10, 0x3C, 0x9A), self.tr("Checking the car…", "กำลังตรวจรถ…"))
        } else if worst == 0 {
            (Color32::from_rgb(0x22, 0xB0, 0x5A), Color32::from_rgb(0x12, 0x80, 0x3E), self.tr("No problems found", "ไม่พบปัญหา"))
        } else if worst == 1 {
            (Color32::from_rgb(0xF2, 0xA1, 0x1C), Color32::from_rgb(0xC8, 0x78, 0x0A), self.tr("Some items need attention", "มีรายการที่ควรดู"))
        } else {
            (Color32::from_rgb(0xF0, 0x5A, 0x3C), Color32::from_rgb(0xC0, 0x2A, 0x1E), self.tr("Problems found", "พบปัญหา"))
        };
        let (card, _) = ui.allocate_exact_size(Vec2::new(w, 110.0), Sense::hover());
        rounded_gradient(ui.painter(), card, 18.0, top, bot);
        let p = ui.painter();
        p.text(card.left_top() + Vec2::new(22.0, 20.0), Align2::LEFT_TOP, title, FontId::proportional(22.0), Color32::WHITE);
        if !done {
            let k = step.min(CHECK_STEPS - 1);
            p.text(card.left_top() + Vec2::new(22.0, 56.0), Align2::LEFT_TOP, format!("{}/{} · {}", k + 1, CHECK_STEPS, steps[k]), FontId::proportional(15.0), Color32::from_white_alpha(220));
            let bar = Rect::from_min_size(card.left_bottom() + Vec2::new(22.0, -26.0), Vec2::new(w - 44.0, 8.0));
            p.rect_filled(bar, 4.0, Color32::from_white_alpha(60));
            let mut f = k as f32 / CHECK_STEPS as f32;
            if k == 1 {
                if let Some((d, t)) = self.dtc_progress {
                    f += d as f32 / t.max(1) as f32 / CHECK_STEPS as f32;
                }
            }
            p.rect_filled(Rect::from_min_size(bar.min, Vec2::new(bar.width() * f, bar.height())), 4.0, Color32::WHITE);
            ui.ctx().request_repaint();
        } else {
            let n_bad = findings.iter().filter(|x| x.0 > 0).count();
            p.text(card.left_top() + Vec2::new(22.0, 58.0), Align2::LEFT_TOP, format!("{} {} · {} {}", findings.len(), self.tr("items", "รายการ"), n_bad, self.tr("to look at", "ควรดู")), FontId::proportional(15.0), Color32::from_white_alpha(220));
        }
        ui.add_space(12.0);
        if done {
            egui::Frame::new().fill(Color32::WHITE).corner_radius(14).inner_margin(14).show(ui, |ui| {
                ui.set_width(ui.available_width());
                for (l, k, v) in &findings {
                    let col = match l {
                        0 => GREEN,
                        1 => Color32::from_rgb(0xF0, 0x85, 0x1F),
                        _ => RED,
                    };
                    ui.horizontal(|ui| {
                        let (dot, _) = ui.allocate_exact_size(Vec2::splat(14.0), Sense::hover());
                        ui.painter().circle_filled(dot.center(), 6.0, col);
                        ui.label(RichText::new(k).size(15.0).strong().color(TEXT_DARK));
                    });
                    ui.label(RichText::new(v).size(14.0).color(Color32::GRAY));
                    ui.add_space(6.0);
                }
            });
            ui.add_space(10.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add(blue_button(self.tr("Check again", "ตรวจอีกครั้ง"), Vec2::new(150.0, 44.0))).clicked() {
                    self.stack.pop();
                    self.start_check();
                }
                if ui.add(grey_button(self.tr("Trouble codes", "ดูรหัสปัญหา"), Vec2::new(150.0, 44.0))).clicked() {
                    self.go(Screen::Codes);
                }
                if ui.add(grey_button(self.tr("Battery", "ดูแบตเตอรี่"), Vec2::new(150.0, 44.0))).clicked() {
                    self.go(Screen::Battery);
                }
            });
        } else {
            for (i, sname) in steps.iter().enumerate() {
                let (txt, col) = if i < step {
                    (self.tr("done", "เสร็จ"), GREEN)
                } else if i == step {
                    (self.tr("reading…", "กำลังอ่าน…"), ACCENT)
                } else {
                    ("", Color32::GRAY)
                };
                ui.horizontal(|ui| {
                    ui.label(RichText::new(*sname).size(15.0).color(TEXT_DARK));
                    ui.label(RichText::new(txt).size(13.0).color(col));
                });
            }
        }
        ui.add_space(70.0);
    }

    fn open_vehicle(&mut self, vin: String, name: String) {
        self.current_vin = vin;
        self.vehicle = name;
        self.codes_read = false;
        self.systems = Vec::new();
        if self.real() {
            self.set_pids(Vec::new());
            self.link.send(Cmd::Discover);
        }
        self.go(Screen::Menu);
    }

    fn vin_input(&mut self, ui: &mut egui::Ui) {
        ui.add_space(12.0);
        let w = ui.available_width();
        // read from the car
        let can_read = self.link.connected() && !self.vin_reading && !self.link.busy;
        ui.horizontal(|ui| {
            if ui.add_enabled(can_read, blue_button(self.tr("Read VIN from car", "อ่าน VIN จากรถ"), Vec2::new(220.0, 46.0))).clicked() {
                self.vin_reading = true;
                self.link.send(Cmd::ReadVin);
            }
            if self.vin_reading {
                ui.spinner();
                ui.label(RichText::new(self.tr("Reading…", "กำลังอ่าน…")).color(Color32::GRAY));
            }
        });
        ui.add_space(10.0);
        // or type it
        let mut go = false;
        let hint = self.tr("or type the 17-character VIN", "หรือพิมพ์ VIN 17 ตัว").to_string();
        ui.horizontal(|ui| {
            let te = egui::TextEdit::singleline(&mut self.vin_text)
                .hint_text(hint)
                .desired_width((w - 130.0).clamp(160.0, 380.0))
                .font(FontId::monospace(18.0))
                .char_limit(17);
            let r = ui.add(te);
            if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                go = true;
            }
            if ui.add(grey_button(self.tr("Check", "ตรวจ"), Vec2::new(100.0, 36.0))).clicked() {
                go = true;
            }
        });
        if go {
            self.vin_text = self.vin_text.trim().to_uppercase();
        }
        let v = self.vin_text.trim().to_uppercase();
        if v.len() < 17 {
            return;
        }
        ui.add_space(12.0);
        let Some(info) = vin::decode_vin(&v) else {
            ui.label(RichText::new(self.tr("Not a valid VIN", "VIN ไม่ถูกต้อง")).size(15.0).color(RED));
            return;
        };
        // a name for the vehicle: the maker from the VIN
        let name = if info.maker.is_empty() { "Vehicle".to_string() } else { info.maker.to_string() };
        let mut open = false;
        egui::Frame::new().fill(Color32::WHITE).corner_radius(14).inner_margin(16).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(&v).monospace().size(22.0).strong().color(TEXT_DARK));
            ui.add_space(8.0);
            let unknown = self.tr("unknown", "ไม่ทราบ").to_string();
            let rows = [
                (self.tr("Maker", "ผู้ผลิต"), if info.maker.is_empty() { format!("{unknown} ({})", &v[..3]) } else { info.maker.to_string() }),
                (self.tr("Region", "ภูมิภาค"), if info.region.is_empty() { unknown.clone() } else { info.region.to_string() }),
                (self.tr("Model year", "ปีรุ่น"), info.year.map(|y| y.to_string()).unwrap_or(unknown)),
                (self.tr("Plant code", "รหัสโรงงาน"), info.plant.to_string()),
                (self.tr("Serial", "เลขลำดับ"), info.serial.clone()),
            ];
            egui::Grid::new("vininfo").num_columns(2).spacing([24.0, 8.0]).show(ui, |ui| {
                for (k, val) in rows {
                    ui.label(RichText::new(k).size(15.0).color(Color32::GRAY));
                    ui.label(RichText::new(val).size(16.0).color(TEXT_DARK));
                    ui.end_row();
                }
            });
            ui.add_space(12.0);
            if ui.add(blue_button(self.tr("Open diagnosis", "เปิดเมนูวินิจฉัย"), Vec2::new(220.0, 44.0))).clicked() {
                open = true;
            }
        });
        if open {
            self.open_vehicle(v, name);
        }
    }

    fn menu(&mut self, ui: &mut egui::Ui) {
        ui.add_space(10.0);
        ui.label(RichText::new(format!("{}   VIN: {}", self.vehicle, self.current_vin)).size(15.0).strong());
        ui.add_space(14.0);
        let items: [(&str, &str, Screen, Color32, Color32); 3] = [
            ("Read Trouble Codes", "อ่านรหัสข้อผิดพลาด", Screen::Codes, rgb(0x7741EC), rgb(0x5734D0)),
            ("Live Data", "ข้อมูลสด", Screen::LiveData, rgb(0xFD685B), rgb(0xF63B67)),
            ("Report", "รายงาน", Screen::Reports, rgb(0x5075FE), rgb(0x4B54FA)),
        ];
        let w = (ui.available_width() - 2.0 * 16.0) / 3.0;
        ui.horizontal(|ui| {
            for (i, (en, th, target, c1, c2)) in items.iter().enumerate() {
                let (r, resp) = ui.allocate_exact_size(Vec2::new(w, 140.0), Sense::click());
                rounded_gradient(ui.painter(), r, 14.0, *c1, *c2);
                draw_icon(ui.painter(), [1u8, 2, 3][i], Pos2::new(r.center().x, r.top() + 52.0), 26.0);
                ui.painter().text(
                    Pos2::new(r.center().x, r.bottom() - 30.0),
                    Align2::CENTER_CENTER,
                    if self.lang == Lang::En { *en } else { *th },
                    FontId::proportional(16.0),
                    Color32::WHITE,
                );
                if resp.clicked() {
                    if *target == Screen::Codes {
                        self.read_codes();
                    }
                    self.go(*target);
                }
            }
        });
        ui.add_space(14.0);
        if ui.add(blue_button(self.tr("⚡❄ Battery cooling & charge (BYD)", "⚡❄ ทำความเย็นแบต & ชาร์จ (BYD)"), Vec2::new(420.0, 42.0))).clicked() {
            self.go(Screen::Charge);
        }
    }

    fn read_codes(&mut self) {
        self.codes_read = true;
        if self.real() {
            self.dtc_reading = true;
            self.systems.clear(); // no stale/demo codes while reading
            self.link.send(Cmd::ReadDtc);
            return;
        }
        self.make_report();
    }

    fn make_report(&mut self) {
        let total_codes: usize = self.systems.iter().map(|s| s.dtcs.len()).sum();
        let with_codes = self.systems.iter().filter(|s| !s.dtcs.is_empty()).count();
        // Report summary wording.
        let summary = format!(
            "{} systems detected: {} systems related to fault codes, {} fault codes",
            self.systems.len(),
            with_codes,
            total_codes
        );
        let rows = self
            .systems
            .iter()
            .flat_map(|s| s.dtcs.iter().map(move |d| (s.name.to_string(), d.code(), d.text.to_string(), d.status.label().to_string())))
            .collect();
        self.reports.insert(
            0,
            Report {
                time: clock_text(),
                vin: self.current_vin.clone(),
                vehicle: self.vehicle.clone(),
                summary,
                rows,
            },
        );
        self.save_reports();
    }

    fn codes(&mut self, ui: &mut egui::Ui) {
        // opened straight from a home tile: read once automatically
        if self.real() && !self.codes_read && !self.dtc_reading && !self.link.busy {
            self.read_codes();
        }
        let total: usize = self.systems.iter().map(|s| s.dtcs.len()).sum();
        let with: Vec<usize> = (0..self.systems.len()).filter(|i| !self.systems[*i].dtcs.is_empty()).collect();
        let clean: Vec<&'static str> = self.systems.iter().filter(|s| s.dtcs.is_empty()).map(|s| s.name).collect();
        let busy = self.dtc_reading || self.dtc_progress.is_some();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let w = ui.available_width();
            ui.add_space(8.0);
            // ---- summary card
            let (top, bot) = if busy {
                (Color32::from_rgb(0x1A, 0x6F, 0xE8), Color32::from_rgb(0x10, 0x3C, 0x9A))
            } else if total == 0 {
                (Color32::from_rgb(0x22, 0xB0, 0x5A), Color32::from_rgb(0x12, 0x80, 0x3E))
            } else {
                (Color32::from_rgb(0xF0, 0x5A, 0x3C), Color32::from_rgb(0xC0, 0x2A, 0x1E))
            };
            let (card, _) = ui.allocate_exact_size(Vec2::new(w, 120.0), Sense::hover());
            rounded_gradient(ui.painter(), card, 18.0, top, bot);
            let p = ui.painter();
            let dim = Color32::from_white_alpha(215);
            if busy {
                let (d, t) = self.dtc_progress.unwrap_or((0, 0));
                p.text(card.left_top() + Vec2::new(22.0, 20.0), Align2::LEFT_TOP, self.tr("Reading trouble codes…", "กำลังอ่านรหัสปัญหา…"), FontId::proportional(20.0), Color32::WHITE);
                if t > 0 {
                    p.text(card.left_top() + Vec2::new(22.0, 54.0), Align2::LEFT_TOP, format!("{} {d}/{t}", self.tr("modules", "โมดูล")), FontId::proportional(15.0), dim);
                    let bar = Rect::from_min_size(card.left_bottom() + Vec2::new(22.0, -32.0), Vec2::new(w - 44.0, 8.0));
                    p.rect_filled(bar, 4.0, Color32::from_white_alpha(60));
                    let f = (d as f32 / t as f32).clamp(0.0, 1.0);
                    p.rect_filled(Rect::from_min_size(bar.min, Vec2::new(bar.width() * f, bar.height())), 4.0, Color32::WHITE);
                }
                ui.ctx().request_repaint();
            } else {
                p.text(card.left_center() + Vec2::new(22.0, -6.0), Align2::LEFT_CENTER, total.to_string(), FontId::proportional(52.0), Color32::WHITE);
                let msg = if total == 0 { self.tr("No trouble codes", "ไม่พบรหัสปัญหา") } else { self.tr("trouble codes", "รหัสปัญหา") };
                p.text(card.left_top() + Vec2::new(100.0, 34.0), Align2::LEFT_TOP, msg, FontId::proportional(20.0), Color32::WHITE);
                p.text(
                    card.left_top() + Vec2::new(100.0, 64.0),
                    Align2::LEFT_TOP,
                    format!("{} {} · {} {}", self.systems.len(), self.tr("modules answered", "โมดูลตอบกลับ"), with.len(), self.tr("with codes", "มีรหัส")),
                    FontId::proportional(14.0),
                    dim,
                );
            }
            ui.add_space(12.0);
            // ---- modules with codes
            for si in &with {
                let s = &self.systems[*si];
                egui::Frame::new().fill(Color32::WHITE).corner_radius(14).inner_margin(14).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(s.name).strong().size(16.0).color(TEXT_DARK));
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            ui.label(RichText::new(format!(" {} ", s.dtcs.len())).strong().color(Color32::WHITE).background_color(RED));
                        });
                    });
                    for d in &s.dtcs {
                        ui.add_space(6.0);
                        let (lbl, col) = match d.status {
                            DtcStatus::Current => (self.tr("Current", "ปัจจุบัน"), RED),
                            DtcStatus::Pending => (self.tr("Pending", "รอยืนยัน"), Color32::from_rgb(0xF0, 0x85, 0x1F)),
                            DtcStatus::History => (self.tr("History", "ประวัติ"), Color32::GRAY),
                        };
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(d.code()).monospace().strong().size(17.0).color(ACCENT));
                            ui.label(RichText::new(format!(" {lbl} ")).size(13.0).color(Color32::WHITE).background_color(col));
                        });
                        ui.label(RichText::new(d.text).size(14.5).color(TEXT_DARK));
                    }
                });
                ui.add_space(10.0);
            }
            // ---- modules without codes (folded)
            if !clean.is_empty() && !busy {
                let head = format!("{} {} ({})", if self.dtc_show_clean { "−" } else { "+" }, self.tr("Modules without codes", "โมดูลที่ไม่มีรหัส"), clean.len());
                if ui.add(egui::Button::new(RichText::new(head).size(15.0).color(GREEN)).frame(false)).clicked() {
                    self.dtc_show_clean = !self.dtc_show_clean;
                }
                if self.dtc_show_clean {
                    egui::Frame::new().fill(Color32::WHITE).corner_radius(14).inner_margin(14).show(ui, |ui| {
                        ui.set_width(ui.available_width());
                        for n in &clean {
                            ui.label(RichText::new(*n).size(14.0).color(TEXT_DARK));
                        }
                    });
                }
                ui.add_space(10.0);
            }
            // ---- actions
            ui.add_space(6.0);
            ui.horizontal_wrapped(|ui| {
                if ui.add_enabled(!busy, blue_button(self.tr("Read again", "อ่านใหม่"), Vec2::new(150.0, 46.0))).clicked() {
                    self.read_codes();
                }
                if total > 0 {
                    let b = egui::Button::new(RichText::new(self.tr("Clear codes", "ล้างรหัส")).color(Color32::WHITE).size(15.0))
                        .fill(Color32::from_rgb(0xE5, 0x1B, 0x1B))
                        .min_size(Vec2::new(150.0, 46.0))
                        .corner_radius(CornerRadius::same(23));
                    if ui.add_enabled(!busy, b).clicked() {
                        self.confirm_clear = true;
                    }
                }
            });
            ui.add_space(70.0);
        });
    }

    fn live_data(&mut self, ui: &mut egui::Ui) {
        let sel_count = self.live.selected.iter().filter(|s| **s).count();
        ui.add_space(6.0);
        let mut msg: Option<String> = None;
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing = Vec2::new(8.0, 8.0);
            let pause = if self.live.paused { self.tr("Continue", "ต่อ") } else { self.tr("Pause", "หยุดชั่วคราว") };
            if ui.add(grey_button(pause, Vec2::new(120.0, 36.0))).clicked() {
                self.live.paused = !self.live.paused;
            }
            // view switch: dashboard / table / graph
            let cur = if self.live.graph { 2 } else if self.live.table { 1 } else { 0 };
            for (k, en, th) in [(0u8, "Dashboard", "แดชบอร์ด"), (1, "Table", "ตาราง"), (2, "Graph", "กราฟ")] {
                let label = self.tr(en, th);
                let b = if cur == k { blue_button(label, Vec2::new(96.0, 36.0)) } else { grey_button(label, Vec2::new(96.0, 36.0)) };
                if ui.add(b).clicked() {
                    match k {
                        2 if sel_count == 0 => {
                            msg = Some(self.tr("Tap cards to choose values for the graph", "แตะการ์ดเพื่อเลือกค่าที่จะแสดงในกราฟ").into())
                        }
                        2 => self.live.graph = true,
                        1 => {
                            self.live.graph = false;
                            self.live.table = true;
                        }
                        _ => {
                            self.live.graph = false;
                            self.live.table = false;
                        }
                    }
                }
            }
            let rec = if self.live.recording.is_some() { self.tr("■ Stop recording", "■ หยุดบันทึก") } else { self.tr("● Record", "● บันทึก") };
            if ui.add(grey_button(rec, Vec2::new(130.0, 36.0))).clicked() {
                if let Some(rows) = self.live.recording.take() {
                    let header: Vec<&str> = self.live.pids.iter().map(|p| p.name).collect();
                    let text = format!("time,{}\n{}\n", header.join(","), rows.join("\n"));
                    msg = Some(match save_file("recording", "csv", &text) {
                        Ok(p) => format!("{} {p}", self.tr("Saved:", "บันทึกแล้ว:")),
                        Err(e) => format!("{} {e}", self.tr("Save failed:", "บันทึกไม่ได้:")),
                    });
                } else {
                    self.live.recording = Some(Vec::new());
                    msg = Some(self.tr("Recording…", "กำลังบันทึก…").into());
                }
            }
            if ui.add(grey_button(self.tr("Export CSV", "ส่งออก CSV"), Vec2::new(120.0, 36.0))).clicked() {
                let mut text = String::from("NO.,Name,Value,Min,Max,Unit\n");
                for (i, p) in self.live.pids.iter().enumerate() {
                    if sel_count == 0 || self.live.selected[i] {
                        let (v, u) = car::to_units(self.live.values[i], p.unit, self.units);
                        let (mn, mx) = self.live.ranges[i];
                        text += &format!("{},{},{v:.2},{mn},{mx},{u}\n", i + 1, p.name);
                    }
                }
                msg = Some(match save_file("livedata", "csv", &text) {
                    Ok(p) => format!("{} {p}", self.tr("Exported:", "ส่งออกแล้ว:")),
                    Err(e) => format!("{} {e}", self.tr("Export failed:", "ส่งออกไม่ได้:")),
                });
            }
            if sel_count > 0 {
                ui.label(RichText::new(format!("{} {sel_count}", self.tr("selected", "เลือก"))).color(ACCENT));
                if ui.small_button(self.tr("Clear", "ล้าง")).clicked() {
                    self.live.selected.iter_mut().for_each(|s| *s = false);
                    self.live.graph = false;
                }
            }
        });
        if let Some(m) = msg {
            self.toast(m);
        }
        ui.add_space(8.0);
        if self.live.graph {
            self.live_graph(ui);
        } else if self.live.table {
            self.live_table(ui);
        } else {
            self.live_dash(ui);
        }
    }

    /// Value and unit as shown on screen (gear letters, handbrake on/off, unit conversion).
    fn live_text(&self, i: usize) -> (String, &'static str) {
        let p = &self.live.pids[i];
        let v = self.live.values[i];
        if !v.is_finite() {
            return ("—".into(), if matches!(p.unit, "gear" | "onoff" | "obcstate") { "" } else { p.unit });
        }
        match p.unit {
            "gear" => (["?", "P", "R", "N", "D"].get(v as usize).copied().unwrap_or("?").into(), ""),
            "obcstate" => (self.obc_state_text(v).into(), ""),
            "onoff" => (if v >= 0.5 { self.tr("ON", "ดึง") } else { self.tr("OFF", "ปล่อย") }.into(), ""),
            _ => {
                let (cv, cu) = car::to_units(v, p.unit, self.units);
                (if cv.abs() >= 100.0 || p.unit.is_empty() { format!("{cv:.0}") } else { format!("{cv:.1}") }, cu)
            }
        }
    }

    /// Short display name for a live value (Thai/English).
    fn live_label(&self, name: &'static str) -> &'static str {
        let (en, th) = match name {
            "SOC (State of Charge)" => ("SOC", "SOC"),
            "Battery Pack Current Total Voltage" => ("Pack voltage", "แรงดันแพ็ก"),
            "Battery Pack Current Total Current (+ = discharge)" => ("Pack current", "กระแสแพ็ก"),
            "Battery temperature" => ("Battery temp", "อุณหภูมิแบต"),
            "Battery SOH" => ("Battery health (SOH)", "สุขภาพแบต (SOH)"),
            "Minimum Single Cell Voltage" => ("Lowest cell", "เซลล์ต่ำสุด"),
            "Maximum Single Cell Voltage" => ("Highest cell", "เซลล์สูงสุด"),
            "Number of Charges" => ("Charge count", "จำนวนครั้งที่ชาร์จ"),
            "Max cell temperature" => ("Max cell temp", "อุณหภูมิเซลล์สูงสุด"),
            "Min cell temperature" => ("Min cell temp", "อุณหภูมิเซลล์ต่ำสุด"),
            "Lowest cell number" => ("Lowest cell #", "เซลล์ต่ำสุด #"),
            "Highest cell number" => ("Highest cell #", "เซลล์สูงสุด #"),
            "Max allowed charge power" => ("Max charge power", "ชาร์จได้สูงสุด"),
            "Max allowed discharge power" => ("Max discharge power", "จ่ายไฟได้สูงสุด"),
            "Insulation resistance" => ("Insulation", "ความต้านทานฉนวน"),
            "Charged energy (lifetime)" => ("Charged (lifetime)", "ชาร์จเข้าสะสม"),
            "Discharged energy (lifetime)" => ("Discharged (lifetime)", "ใช้ไปสะสม"),
            "OBC internal temp 1" => ("OBC temp 1", "OBC อุณหภูมิ 1"),
            "OBC internal temp 2" => ("OBC temp 2", "OBC อุณหภูมิ 2"),
            "OBC internal temp 3" => ("OBC temp 3", "OBC อุณหภูมิ 3"),
            "OBC internal temp 4" => ("OBC temp 4", "OBC อุณหภูมิ 4"),
            "Charging port temp" => ("Charging port temp", "อุณหภูมิหัวชาร์จ"),
            "OBC cooling fan" => ("OBC fan", "พัดลม OBC"),
            "AC voltage" => ("AC voltage", "แรงดัน AC"),
            "AC current" => ("AC current", "กระแส AC"),
            "AC power" => ("AC power", "กำลังชาร์จ AC"),
            "OBC state" => ("OBC state", "สถานะ OBC"),
            "12V battery" => ("12V battery", "แบต 12V"),
            "Gear" => ("Gear", "เกียร์"),
            "Handbrake" => ("Handbrake", "เบรกมือ"),
            "A/C set temperature" => ("A/C set", "แอร์ตั้งไว้"),
            "A/C fan level" => ("A/C fan", "พัดลมแอร์"),
            "BMS thermal request" => ("BMS thermal request", "BMS ขอระบายความร้อน"),
            "Vehicle speed (bus)" => ("Speed", "ความเร็ว"),
            "Wheel speed FL" => ("Wheel FL", "ล้อหน้าซ้าย"),
            "Wheel speed FR" => ("Wheel FR", "ล้อหน้าขวา"),
            "Wheel speed RL" => ("Wheel RL", "ล้อหลังซ้าย"),
            "Wheel speed RR" => ("Wheel RR", "ล้อหลังขวา"),
            "Engine speed" => ("Engine speed", "รอบเครื่อง"),
            "Vehicle speed" => ("Speed", "ความเร็ว"),
            "Coolant temperature" => ("Coolant temp", "อุณหภูมิน้ำหล่อเย็น"),
            "Intake air temperature" => ("Intake air temp", "อุณหภูมิอากาศเข้า"),
            "Calculated engine load" => ("Engine load", "โหลดเครื่องยนต์"),
            "Throttle position" => ("Throttle", "ลิ้นปีกผีเสื้อ"),
            "Control module voltage" => ("Module voltage", "แรงดันกล่องควบคุม"),
            "MAF air flow rate" => ("Air flow (MAF)", "ปริมาณอากาศ (MAF)"),
            "Timing advance" => ("Timing advance", "องศาจุดระเบิด"),
            "Short term fuel trim B1" => ("Fuel trim B1", "ปรับน้ำมัน B1"),
            _ => return name,
        };
        if self.lang == Lang::Th { th } else { en }
    }

    /// Dark gauge dashboard: a 3-column grid (more columns in landscape) of black cells with
    /// thin borders — name on top, big value, unit below — plus a power gauge (kW = V x A)
    /// when the car reports pack voltage and current. Tap a cell to pick it for the graph.
    fn live_dash(&mut self, ui: &mut egui::Ui) {
        if self.live.pids.is_empty() {
            ui.add_space(30.0);
            ui.vertical_centered(|ui| {
                if self.link.busy {
                    ui.spinner();
                    ui.label(RichText::new(self.tr("Finding values this car reports…", "กำลังค้นหาค่าที่รถรองรับ…")).size(15.0).color(Color32::GRAY));
                } else {
                    ui.label(RichText::new(self.tr("No live values", "ไม่มีค่าข้อมูลสด")).size(15.0).color(Color32::GRAY));
                }
            });
            return;
        }
        let find = |k: &str| self.live.pids.iter().position(|p| p.name.starts_with(k));
        let vi = find("Battery Pack Current Total Voltage");
        let ai = find("Battery Pack Current Total Current");
        let power = match (vi, ai) {
            (Some(v), Some(a)) => {
                let now = self.live.values[v] * self.live.values[a] / 1000.0;
                let hist: Vec<f64> = self.live.history[v]
                    .iter()
                    .rev()
                    .zip(self.live.history[a].iter().rev())
                    .map(|(v, a)| v * a / 1000.0)
                    .filter(|x| x.is_finite())
                    .collect();
                Some((now, hist))
            }
            _ => None,
        };

        // ---- group the values into dashboard sections with a few combined tiles
        let n = self.live.pids.len();
        let mut used = vec![false; n];
        let take = |i: Option<usize>, used: &mut Vec<bool>| {
            if let Some(i) = i {
                used[i] = true;
            }
            i
        };
        let soc = take(find("SOC (State of Charge)"), &mut used);
        let t_max = take(find("Max cell temperature"), &mut used);
        let t_avg = take(find("Battery temperature"), &mut used);
        let t_min = take(find("Min cell temperature"), &mut used);
        let v_max = take(find("Maximum Single Cell Voltage"), &mut used);
        let v_min = take(find("Minimum Single Cell Voltage"), &mut used);
        let n_hi = take(find("Highest cell number"), &mut used);
        let n_lo = take(find("Lowest cell number"), &mut used);
        let hdr_of = |i: usize| match self.live.pids[i].src {
            Src::Did { hdr, .. } => hdr,
            _ => 0,
        };
        let mut bms: Vec<DashCell> = Vec::new();
        if let Some(i) = soc {
            bms.push(DashCell::Soc(i));
        }
        if power.is_some() {
            bms.push(DashCell::Power);
        }
        if t_max.is_some() || t_avg.is_some() || t_min.is_some() {
            bms.push(DashCell::Temps(t_max, t_avg, t_min));
        }
        if v_max.is_some() || v_min.is_some() {
            bms.push(DashCell::Volts(v_max, v_min, n_hi, n_lo));
        }
        bms.extend((0..n).filter(|i| !used[*i] && hdr_of(*i) == 0x7E7).map(DashCell::Num));
        let obc: Vec<DashCell> = (0..n).filter(|i| !used[*i] && hdr_of(*i) == obd::OBC).map(DashCell::Num).collect();
        let car: Vec<DashCell> = (0..n).filter(|i| !used[*i] && hdr_of(*i) != 0x7E7 && hdr_of(*i) != obd::OBC).map(DashCell::Num).collect();
        let sections = [
            (self.tr("Battery (BMS)", "แบตเตอรี่ (BMS)"), bms),
            (self.tr("On-board charger (OBC)", "ตัวชาร์จในรถ (OBC)"), obc),
            (self.tr("Vehicle", "รถ"), car),
        ];

        const LIME: Color32 = Color32::from_rgb(0xC6, 0xF0, 0x3C);
        let green = Color32::from_rgb(0x4C, 0xD9, 0x2E);
        let line = Stroke::new(1.5, Color32::from_gray(205));
        let mut toggle: Option<usize> = None;
        let val = |i: Option<usize>| i.map(|i| self.live.values[i]).filter(|v| v.is_finite());
        egui::Frame::new().fill(Color32::BLACK).show(ui, |ui| {
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                let w = ui.available_width();
                let cols = if w > 1000.0 { 5 } else if w > 700.0 { 4 } else { 3 };
                let cw = w / cols as f32;
                let ch = (cw * 1.08).clamp(130.0, 210.0);
                let big = (cw * 0.26).min(48.0);
                for (title, cells) in &sections {
                    if cells.is_empty() {
                        continue;
                    }
                    // section header bar
                    let (hr, _) = ui.allocate_exact_size(Vec2::new(w, 34.0), Sense::hover());
                    ui.painter().text(hr.left_center() + Vec2::new(8.0, 2.0), Align2::LEFT_CENTER, *title, FontId::proportional(17.0), HEAD_LIME);
                    let (area, _) = ui.allocate_exact_size(Vec2::new(w, ch * cells.len().div_ceil(cols) as f32), Sense::hover());
                    for (k, cell) in cells.iter().enumerate() {
                        let r = Rect::from_min_size(area.min + Vec2::new((k % cols) as f32 * cw, (k / cols) as f32 * ch), Vec2::new(cw, ch));
                        let resp = ui.interact(r, ui.scope_id().with((*title, k)), Sense::click());
                        let p = ui.painter();
                        p.rect_stroke(r, 0.0, line, StrokeKind::Inside);
                        let label = |p: &egui::Painter, ui: &egui::Ui, text: &str| {
                            let mut job = egui::text::LayoutJob::simple(text.to_string(), FontId::proportional(14.5), LIME, cw - 12.0);
                            job.halign = Align::Center;
                            let g = ui.fonts_mut(|f| f.layout_job(job));
                            p.galley(Pos2::new(r.center().x, r.top() + 10.0), g, LIME);
                        };
                        match *cell {
                            DashCell::Num(i) => {
                                if resp.clicked() {
                                    toggle = Some(i);
                                }
                                if self.live.selected[i] {
                                    p.rect_stroke(r.shrink(3.0), 0.0, Stroke::new(2.5, ACCENT), StrokeKind::Inside);
                                }
                                let v = self.live.values[i];
                                let (mn, mx) = self.live.ranges[i];
                                let (shown, cu) = self.live_text(i);
                                let out = v.is_finite() && (v < mn || v > mx);
                                label(p, ui, self.live_label(self.live.pids[i].name));
                                let size = if shown.chars().count() > 6 { big * 0.7 } else { big };
                                p.text(r.center() + Vec2::new(0.0, 4.0), Align2::CENTER_CENTER, shown, FontId::proportional(size), if out { Color32::from_rgb(0xFF, 0x7A, 0x3D) } else { LIME });
                                p.text(Pos2::new(r.center().x, r.bottom() - 14.0), Align2::CENTER_CENTER, cu, FontId::proportional(14.0), LIME);
                            }
                            DashCell::Soc(i) => {
                                if resp.clicked() {
                                    toggle = Some(i);
                                }
                                let v = self.live.values[i];
                                let c = r.center() + Vec2::new(0.0, 8.0);
                                let rad = (cw.min(ch) * 0.36).max(30.0);
                                let a0 = std::f32::consts::PI * 0.75;
                                let sweep = std::f32::consts::PI * 1.5;
                                draw_arc(p, c, rad, a0, sweep, Color32::from_gray(45), 9.0);
                                if v.is_finite() {
                                    draw_arc(p, c, rad, a0, sweep * (v / 100.0).clamp(0.0, 1.0) as f32, green, 9.0);
                                }
                                let t = if v.is_finite() { format!("{v:.0}") } else { "—".into() };
                                p.text(c - Vec2::new(0.0, 4.0), Align2::CENTER_CENTER, t, FontId::proportional((big * 0.9).max(22.0)), LIME);
                                p.text(c + Vec2::new(0.0, big * 0.45), Align2::CENTER_CENTER, "SOC %", FontId::proportional(12.5), LIME);
                            }
                            DashCell::Power => {
                                let Some((now, hist)) = &power else { continue };
                                let peak = hist.iter().fold(10.0f64, |m, x| m.max(x.abs()));
                                let scale = (peak * 1.2).max(10.0);
                                let c = r.center() + Vec2::new(0.0, 8.0);
                                let rad = (cw.min(ch) * 0.36).max(30.0);
                                let a0 = std::f32::consts::PI * 0.75;
                                let sweep = std::f32::consts::PI * 1.5;
                                draw_arc(p, c, rad, a0, sweep, Color32::from_gray(45), 9.0);
                                if now.is_finite() {
                                    draw_arc(p, c, rad, a0, sweep * (now.abs() / scale).clamp(0.0, 1.0) as f32, green, 9.0);
                                }
                                let t = if now.is_finite() { format!("{now:.1}") } else { "—".into() };
                                p.text(c - Vec2::new(0.0, 4.0), Align2::CENTER_CENTER, t, FontId::proportional((big * 0.8).max(20.0)), LIME);
                                p.text(c + Vec2::new(0.0, big * 0.45), Align2::CENTER_CENTER, format!("{} kW", self.tr("Power", "กำลังไฟ")), FontId::proportional(12.5), LIME);
                                if !hist.is_empty() {
                                    let mx = hist.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                                    let mn = hist.iter().cloned().fold(f64::INFINITY, f64::min);
                                    let avg = hist.iter().sum::<f64>() / hist.len() as f64;
                                    let small = FontId::proportional(11.5);
                                    p.text(r.left_top() + Vec2::new(6.0, 6.0), Align2::LEFT_TOP, format!("Max: {mx:.1}"), small.clone(), green);
                                    p.text(r.right_top() + Vec2::new(-6.0, 6.0), Align2::RIGHT_TOP, format!("Avg: {avg:.1}"), small.clone(), green);
                                    p.text(r.left_bottom() + Vec2::new(6.0, -6.0), Align2::LEFT_BOTTOM, format!("Min: {mn:.1}"), small, green);
                                }
                            }
                            DashCell::Temps(hi, avg, lo) => {
                                label(p, ui, self.tr("Cell temperature", "อุณหภูมิเซลล์"));
                                let t = |x: Option<f64>| x.map(|v| format!("{v:.0}")).unwrap_or("—".into());
                                p.text(r.center() + Vec2::new(0.0, 0.0), Align2::CENTER_CENTER, t(val(avg)), FontId::proportional(big), LIME);
                                p.text(r.center() + Vec2::new(0.0, big * 0.62), Align2::CENTER_CENTER, format!("°C {}", self.tr("avg", "เฉลี่ย")), FontId::proportional(12.5), LIME);
                                let row = format!("{} {}  ·  {} {}", self.tr("min", "ต่ำ"), t(val(lo)), self.tr("max", "สูง"), t(val(hi)));
                                p.text(Pos2::new(r.center().x, r.bottom() - 14.0), Align2::CENTER_CENTER, row, FontId::proportional(13.5), LIME);
                            }
                            DashCell::Volts(hi, lo, nhi, nlo) => {
                                label(p, ui, self.tr("Cell voltage", "แรงดันเซลล์"));
                                let spread = match (val(hi), val(lo)) {
                                    (Some(h), Some(l)) => format!("{:.0}", h - l),
                                    _ => "—".into(),
                                };
                                p.text(r.center() + Vec2::new(0.0, -6.0), Align2::CENTER_CENTER, spread, FontId::proportional(big), LIME);
                                p.text(r.center() + Vec2::new(0.0, big * 0.5), Align2::CENTER_CENTER, format!("mV {}", self.tr("spread", "ผลต่าง")), FontId::proportional(12.5), LIME);
                                let one = |v: Option<f64>, no: Option<f64>| match (v, no) {
                                    (Some(v), Some(n)) => format!("{:.3} #{n:.0}", v / 1000.0),
                                    (Some(v), None) => format!("{:.3}", v / 1000.0),
                                    _ => "—".into(),
                                };
                                let small = FontId::proportional(12.0);
                                p.text(Pos2::new(r.center().x, r.bottom() - 30.0), Align2::CENTER_CENTER, format!("{} {}", self.tr("max", "สูง"), one(val(hi), val(nhi))), small.clone(), LIME);
                                p.text(Pos2::new(r.center().x, r.bottom() - 13.0), Align2::CENTER_CENTER, format!("{} {}", self.tr("min", "ต่ำ"), one(val(lo), val(nlo))), small, LIME);
                            }
                        }
                    }
                    ui.add_space(10.0);
                }
                ui.add_space(70.0); // clear of the system navigation bar
            });
        });
        if let Some(i) = toggle {
            let count = self.live.selected.iter().filter(|s| **s).count();
            if !self.live.selected[i] && count >= MAX_SELECTED {
                let m = self.tr("Up to 8 values can be selected", "เลือกได้สูงสุด 8 ค่า").to_string();
                self.toast(m);
            } else {
                self.live.selected[i] = !self.live.selected[i];
            }
        }
    }

    fn live_table(&mut self, ui: &mut egui::Ui) {
        let mut toast = None;
        egui::Frame::new().fill(Color32::WHITE).corner_radius(8).inner_margin(8).show(ui, |ui| {
            ui.set_width(ui.available_width());
            egui::ScrollArea::vertical().show(ui, |ui| {
                egui::Grid::new("live").num_columns(7).striped(true).spacing([12.0, 8.0]).show(ui, |ui| {
                    for h in ["", "NO.", "Name", "Value", "Raw bytes  →  formula", "Range", "Unit"] {
                        ui.label(RichText::new(h).strong());
                    }
                    ui.end_row();
                    let count = self.live.selected.iter().filter(|s| **s).count();
                    for i in 0..self.live.pids.len() {
                        let p = &self.live.pids[i];
                        let mut s = self.live.selected[i];
                        if ui.checkbox(&mut s, "").changed() {
                            if s && count >= MAX_SELECTED {
                                toast = Some("Up to 8 PIDs can be selected".to_string());
                            } else {
                                self.live.selected[i] = s;
                            }
                        }
                        ui.label(format!("{}", i + 1));
                        ui.label(self.live_label(p.name));
                        let v = self.live.values[i];
                        let (mn, mx) = self.live.ranges[i];
                        let (shown, cu) = self.live_text(i);
                        let out = v < mn || v > mx;
                        ui.label(RichText::new(shown).monospace().size(15.0).color(if out { RED } else { TEXT_DARK }));
                        let (a, b) = self.live.raw[i];
                        let raw = match p.src {
                            Src::Did { hdr, did } => format!("{hdr:03X}: 62 {:02X} {:02X} {a:02X} {b:02X}", did >> 8, did & 0xFF),
                            Src::Mode01 if p.bytes == 2 => format!("41 {:02X} {a:02X} {b:02X}", p.pid),
                            Src::Mode01 => format!("41 {:02X} {a:02X}", p.pid),
                            Src::Volt12 => "ATRV".to_string(),
                            Src::Bus { id } => format!("{id:03X} (bus)"),
                        };
                        ui.label(RichText::new(format!("{raw}  →  {}", p.formula)).monospace().size(12.0).color(Color32::GRAY));
                        let (rmn, _) = car::to_units(mn, p.unit, self.units);
                        let (rmx, _) = car::to_units(mx, p.unit, self.units);
                        if ui.link(format!("{rmn:.1} – {rmx:.1}")).clicked() {
                            self.live.range_edit = Some((i, format!("{mn}"), format!("{mx}"), None));
                        }
                        ui.label(cu);
                        ui.end_row();
                    }
                });
            });
        });
        if let Some(t) = toast {
            self.toast(t);
        }
    }

    fn live_graph(&mut self, ui: &mut egui::Ui) {
        let chosen: Vec<usize> = (0..self.live.pids.len()).filter(|i| self.live.selected[*i]).take(MAX_GRAPH).collect();
        let sel_count = self.live.selected.iter().filter(|s| **s).count();
        if sel_count > MAX_GRAPH {
            ui.label(RichText::new("Up to 5 PIDs can be selected for one graph — showing the first 5").color(RED));
        }
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), Sense::hover());
        let p = ui.painter();
        p.rect_filled(rect, 8.0, Color32::WHITE);
        let plot = rect.shrink2(Vec2::new(20.0, 34.0));
        for k in 0..=4 {
            let y = plot.top() + plot.height() * k as f32 / 4.0;
            p.line_segment([Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)], Stroke::new(1.0, Color32::from_gray(230)));
        }
        let mut lx = rect.left() + 20.0;
        for (n, &i) in chosen.iter().enumerate() {
            let pid = &self.live.pids[i];
            let col = GRAPH_COLORS[n % GRAPH_COLORS.len()];
            let (mn, mx) = self.live.ranges[i];
            let span = (mx - mn).max(1e-6);
            let h = &self.live.history[i];
            let pts: Vec<Pos2> = h
                .iter()
                .enumerate()
                .map(|(k, v)| {
                    let x = plot.left() + plot.width() * k as f32 / (HISTORY - 1) as f32;
                    let t = ((v - mn) / span).clamp(-0.05, 1.05) as f32;
                    Pos2::new(x, plot.bottom() - t * plot.height())
                })
                .collect();
            if pts.len() > 1 {
                p.add(egui::Shape::line(pts, Stroke::new(2.0, col)));
            }
            let (cv, cu) = car::to_units(self.live.values[i], pid.unit, self.units);
            let label = format!("{}: {cv:.1} {cu}", pid.name);
            p.rect_filled(Rect::from_min_size(Pos2::new(lx, rect.top() + 10.0), Vec2::new(14.0, 4.0)), 2.0, col);
            let g = p.text(Pos2::new(lx + 20.0, rect.top() + 12.0), Align2::LEFT_CENTER, label, FontId::proportional(13.0), TEXT_DARK);
            lx = g.right() + 18.0;
        }
        p.text(Pos2::new(plot.left(), rect.bottom() - 12.0), Align2::LEFT_CENTER, "each line scaled to its own range", FontId::proportional(11.0), Color32::GRAY);
    }

    fn reports_ui(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        if self.reports.is_empty() {
            ui.label(RichText::new(self.tr("No reports yet", "ยังไม่มีรายงาน")).size(18.0).color(TEXT_DARK));
            ui.label(RichText::new(self.tr("Run a full check or read trouble codes.", "ตรวจรถทั้งคัน หรืออ่านรหัสปัญหา")).size(15.0).color(Color32::GRAY));
            return;
        }
        let mut save = None;
        let status = |st: &str| -> (&'static str, &'static str, Color32) {
            match st {
                "OK" => ("OK", "ปกติ", GREEN),
                "Attention" => ("Attention", "ควรดู", Color32::from_rgb(0xF0, 0x85, 0x1F)),
                "Problem" | "Current" => ("Problem", "มีปัญหา", RED),
                "Pending" => ("Pending", "รอยืนยัน", Color32::from_rgb(0xF0, 0x85, 0x1F)),
                "History" => ("History", "ประวัติ", Color32::GRAY),
                _ => ("", "", Color32::GRAY),
            }
        };
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            for (ri, r) in self.reports.iter().enumerate() {
                egui::Frame::new().fill(Color32::WHITE).corner_radius(14).inner_margin(14).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(&r.vehicle).strong().size(17.0).color(ACCENT));
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            if ui.add(grey_button(self.tr("Save file", "บันทึกไฟล์"), Vec2::new(110.0, 32.0))).clicked() {
                                save = Some(ri);
                            }
                        });
                    });
                    ui.label(RichText::new(format!("{}  ·  {}", r.time, r.vin)).size(13.5).color(Color32::GRAY));
                    ui.label(RichText::new(&r.summary).size(15.0).strong().color(TEXT_DARK));
                    for (a, b, c, d) in &r.rows {
                        ui.add_space(4.0);
                        let (en, th, col) = status(d);
                        ui.horizontal_wrapped(|ui| {
                            let (dot, _) = ui.allocate_exact_size(Vec2::splat(12.0), Sense::hover());
                            ui.painter().circle_filled(dot.center(), 5.0, col);
                            if c.is_empty() {
                                // full-check line: item, value
                                ui.label(RichText::new(a).size(14.5).strong().color(TEXT_DARK));
                                ui.label(RichText::new(b).size(14.0).color(Color32::DARK_GRAY));
                            } else {
                                // trouble code line: module, code, text
                                ui.label(RichText::new(b).monospace().strong().size(14.5).color(ACCENT));
                                ui.label(RichText::new(if self.lang == Lang::Th { th } else { en }).size(13.0).color(col));
                            }
                        });
                        if !c.is_empty() {
                            ui.label(RichText::new(format!("{c} · {a}")).size(13.5).color(Color32::DARK_GRAY));
                        }
                    }
                });
                ui.add_space(10.0);
            }
            ui.add_space(70.0);
        });
        if let Some(i) = save {
            let r = &self.reports[i];
            let mut text = format!("Time: {}\nVIN: {}\nVehicle: {}\n{}\n\n", r.time, r.vin, r.vehicle, r.summary);
            for (s, c, t, st) in &r.rows {
                text += &format!("{s}\t{c}\t{t}\t{st}\n");
            }
            let m = match save_file("report", "txt", &text) {
                Ok(p) => format!("{} {p}", self.tr("Saved:", "บันทึกแล้ว:")),
                Err(e) => format!("{} {e}", self.tr("Save failed:", "บันทึกไม่ได้:")),
            };
            self.toast(m);
        }
    }

    fn settings(&mut self, ui: &mut egui::Ui) {
        ui.add_space(10.0);
        egui::Frame::new().fill(Color32::WHITE).corner_radius(8).inner_margin(14).show(ui, |ui| {
            ui.set_width(ui.available_width());
            egui::Grid::new("set").num_columns(2).spacing([30.0, 14.0]).show(ui, |ui| {
                ui.label(RichText::new(self.tr("Language", "ภาษา")).strong());
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.lang, Lang::En, "English");
                    ui.selectable_value(&mut self.lang, Lang::Th, "ไทย");
                });
                ui.end_row();
                ui.label(RichText::new(self.tr("Unit", "หน่วย")).strong());
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.units, Units::Metric, "Metric Units");
                    ui.selectable_value(&mut self.units, Units::Imperial, "Imperial Units");
                    ui.selectable_value(&mut self.units, Units::Us, "U.S. Units");
                });
                ui.end_row();
                ui.label(RichText::new(self.tr("Battery module scan", "สแกนข้อมูลโมดูลแบต")).strong());
                ui.vertical(|ui| {
                    let can = self.link.connected() && !self.link.busy && self.did_scan.is_none();
                    if ui.add_enabled(can, grey_button(self.tr("Scan 7E7 (read only)", "สแกน 7E7 (อ่านอย่างเดียว)"), Vec2::new(240.0, 36.0))).clicked() {
                        self.did_found.clear();
                        self.did_scan = Some((0, 0, 0));
                        self.did_scan_ext = false;
                        self.link.send(Cmd::DidScan { hdr: 0x7E7, ranges: vec![(0x0000, 0x02FF), (0x1F00, 0x1FFF)], extended: false });
                    }
                    if ui.add_enabled(can, grey_button(self.tr("Scan 7E7 extended 0000-1FFF", "สแกน 7E7 extended 0000-1FFF"), Vec2::new(240.0, 36.0))).clicked() {
                        self.did_found.clear();
                        self.did_scan = Some((0, 0, 0));
                        self.did_scan_ext = true;
                        self.link.send(Cmd::DidScan { hdr: 0x7E7, ranges: vec![(0x0000, 0x1FFF)], extended: true });
                    }
                    if let Some((done, total, found)) = self.did_scan {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.label(format!("{done}/{total} · {} {found}", self.tr("found", "พบ")));
                        });
                    }
                    for (did, d) in self.did_found.iter().take(60) {
                        let hex: Vec<String> = d.iter().take(12).map(|b| format!("{b:02X}")).collect();
                        ui.label(RichText::new(format!("{did:04X}  [{}]  {}{}", d.len(), hex.join(" "), if d.len() > 12 { " …" } else { "" })).monospace().size(12.0));
                    }
                });
                ui.end_row();
                ui.label(RichText::new(self.tr("About", "เกี่ยวกับ")).strong());
                ui.label(format!("VoltaChill {} · {} M1TS", env!("CARGO_PKG_VERSION"), self.tr("developed by", "พัฒนาโดย")));
                ui.end_row();
            });
        });
    }

    // ---------------------------------------------------------------- dialogs

    fn dialogs(&mut self, ctx: &egui::Context) {
        let red = Color32::from_rgb(0xE5, 0x1B, 0x1B);
        if self.confirm_clear {
            let r = modal(ctx, "clear", self.tr("Clear trouble codes", "ล้างรหัสปัญหา"), red, self.tr("Clear", "ล้างรหัส"), self.tr("Cancel", "ยกเลิก"), |ui| {
                ui.label(
                    RichText::new(self.tr(
                        "Clear the codes in every module? Car parked, power ON. Codes that are still present come back.",
                        "ล้างรหัสในทุกโมดูล? รถจอดและเปิดสวิตช์ไว้ รหัสที่ยังเสียอยู่จะกลับมาอีก",
                    ))
                    .size(17.0)
                    .color(TEXT_DARK),
                );
            });
            if r == Some(true) && self.real() {
                self.dtc_reading = true;
                self.link.send(Cmd::ClearDtc);
            } else if r == Some(true) {
                for s in &mut self.systems {
                    // A fault that is still there comes back as Pending (demo rule, estimated).
                    s.dtcs.retain(|d| d.status == DtcStatus::Current);
                    for d in &mut s.dtcs {
                        d.status = DtcStatus::Pending;
                    }
                }
                self.toast("Clear: done. One fault is still present and returned as Pending.");
            }
            if r.is_some() {
                self.confirm_clear = false;
            }
        }
        if let Some((i, mn, mx, err)) = self.live.range_edit.clone() {
            let mut mn = mn;
            let mut mx = mx;
            let mut err = err;
            let name = self.live_label(self.live.pids[i].name).to_string();
            let unit = self.live.pids[i].unit;
            let (lo_t, hi_t) = (self.tr("Minimum", "ค่าต่ำสุด").to_string(), self.tr("Maximum", "ค่าสูงสุด").to_string());
            let r = modal(ctx, "range", &format!("{} — {name}", self.tr("Range", "ช่วงปกติ")), ACCENT, "OK", self.tr("Cancel", "ยกเลิก"), |ui| {
                ui.label(RichText::new(unit).size(15.0).color(Color32::GRAY));
                for (t, v) in [(&lo_t, &mut mn), (&hi_t, &mut mx)] {
                    ui.label(RichText::new(t).size(16.0).color(TEXT_DARK));
                    ui.add(egui::TextEdit::singleline(v).desired_width(f32::INFINITY).font(FontId::proportional(20.0)));
                    ui.add_space(4.0);
                }
                if let Some(e) = &err {
                    ui.label(RichText::new(e).size(15.0).color(RED));
                }
            });
            let mut done = r == Some(false);
            if r == Some(true) {
                match check_range(&mn, &mx) {
                    Ok(rng) => {
                        self.live.ranges[i] = rng;
                        done = true;
                    }
                    Err(e) => err = Some(e.to_string()),
                }
            }
            self.live.range_edit = if done { None } else { Some((i, mn, mx, err)) };
        }
        if let Some((m, until)) = &self.toast {
            if self.now > *until {
                self.toast = None;
            } else {
                let m = m.clone();
                egui::Area::new(egui::Id::unique("toast"))
                    .anchor(Align2::CENTER_BOTTOM, Vec2::new(0.0, -24.0))
                    .show(ctx, |ui| {
                        egui::Frame::new()
                            .fill(Color32::from_rgba_unmultiplied(20, 30, 45, 230))
                            .corner_radius(18)
                            .inner_margin(egui::Margin::symmetric(16, 8))
                            .show(ui, |ui| {
                                ui.label(RichText::new(m).color(Color32::WHITE));
                            });
                    });
            }
        }
    }
}

/// Android system back button: MainActivity.nativeBack() sets REQ when the app can go back
/// (more than one screen on the stack). The UI thread publishes the stack depth each frame
/// and processes the request. Returns true to the Activity so it consumes the back press.
#[cfg(target_os = "android")]
pub mod androidback {
    use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
    use std::sync::OnceLock;

    pub static DEPTH: AtomicUsize = AtomicUsize::new(1);
    pub static REQ: AtomicBool = AtomicBool::new(false);
    pub static CTX: OnceLock<eframe::egui::Context> = OnceLock::new();
    // System-bar / cutout insets in physical px (status bar, nav bar, camera cutout). The
    // GameActivity surface is full-screen, so the UI insets its content by these itself.
    pub static INSET_L: AtomicU32 = AtomicU32::new(0);
    pub static INSET_T: AtomicU32 = AtomicU32::new(0);
    pub static INSET_R: AtomicU32 = AtomicU32::new(0);
    pub static INSET_B: AtomicU32 = AtomicU32::new(0);
    /// Set by the `car` build flavor (DiLink head unit): smaller UI + keep clear of the
    /// head unit's bottom climate bar.
    pub static CAR: AtomicBool = AtomicBool::new(false);

    /// App-specific storage for saved logs: getExternalFilesDir(null)
    /// (= /sdcard/Android/data/com.tanawoot.scanlab/files, pullable with adb, no permission
    /// needed), falling back to the internal getFilesDir(). Android has no $HOME.
    pub fn files_dir() -> Option<std::path::PathBuf> {
        static DIR: OnceLock<Option<std::path::PathBuf>> = OnceLock::new();
        DIR.get_or_init(|| {
            use jni::objects::{JObject, JString, JValue};
            let ctx = ndk_context::android_context();
            let vm = unsafe { jni::JavaVM::from_raw(ctx.vm().cast()) }.ok()?;
            let mut env = vm.attach_current_thread().ok()?;
            let activity = unsafe { JObject::from_raw(ctx.context().cast()) };
            let null = JObject::null();
            let ext = env
                .call_method(&activity, "getExternalFilesDir", "(Ljava/lang/String;)Ljava/io/File;", &[JValue::Object(&null)])
                .and_then(|v| v.l())
                .ok()
                .filter(|f| !f.is_null());
            let _ = env.exception_clear();
            let file = match ext {
                Some(f) => f,
                None => env.call_method(&activity, "getFilesDir", "()Ljava/io/File;", &[]).and_then(|v| v.l()).ok()?,
            };
            let s = env.call_method(&file, "getAbsolutePath", "()Ljava/lang/String;", &[]).and_then(|v| v.l()).ok()?;
            let s: String = env.get_string(&JString::from(s)).ok()?.into();
            Some(std::path::PathBuf::from(s))
        })
        .clone()
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_MainActivity_nativeInsets(
        _env: jni::JNIEnv,
        _obj: jni::objects::JObject,
        l: jni::sys::jint,
        t: jni::sys::jint,
        r: jni::sys::jint,
        b: jni::sys::jint,
    ) {
        INSET_L.store(l.max(0) as u32, Ordering::Relaxed);
        INSET_T.store(t.max(0) as u32, Ordering::Relaxed);
        INSET_R.store(r.max(0) as u32, Ordering::Relaxed);
        INSET_B.store(b.max(0) as u32, Ordering::Relaxed);
        if let Some(c) = CTX.get() {
            c.request_repaint();
        }
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_MainActivity_nativeCarMode(
        _env: jni::JNIEnv,
        _obj: jni::objects::JObject,
        on: jni::sys::jboolean,
    ) {
        CAR.store(on != 0, Ordering::Relaxed);
        if let Some(c) = CTX.get() {
            c.request_repaint();
        }
    }

    #[no_mangle]
    pub extern "system" fn Java_com_tanawoot_scanlab_MainActivity_nativeBack(
        _env: jni::JNIEnv,
        _obj: jni::objects::JObject,
    ) -> jni::sys::jboolean {
        log::info!("[back] nativeBack depth={}", DEPTH.load(Ordering::Relaxed));
        if DEPTH.load(Ordering::Relaxed) > 1 {
            REQ.store(true, Ordering::Relaxed);
            if let Some(c) = CTX.get() {
                c.request_repaint();
            }
            1
        } else {
            0
        }
    }
}

impl eframe::App for ScanApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        #[cfg(target_os = "android")]
        {
            use std::sync::atomic::Ordering;
            let _ = androidback::CTX.set(ctx.clone());
            androidback::DEPTH.store(self.stack.len(), Ordering::Relaxed);
            if androidback::REQ.swap(false, Ordering::Relaxed) {
                self.back();
            }
        }
        // The UI is laid out for a wide desktop window (≥900 pt). On a phone the screen is
        // narrow and high-DPI, so scale the whole UI so the logical width ≈ the design width
        // and the existing layout fits instead of overlapping/clipping.
        #[cfg(target_os = "android")]
        {
            let cur = ctx.pixels_per_point();
            let phys_w = ui.max_rect().width() * cur;
            let phys_h = ui.max_rect().height() * cur;
            // Landscape (car head unit / tablet) → wide multi-column layout; portrait (phone) →
            // the narrower one-/two-column layout. One APK adapts to the screen it runs on.
            // (narrower logical width = everything bigger on the phone; user asked 2026-11-09)
            let landscape = phys_w > phys_h * 1.15;
            let want = if car_mode() && landscape {
                // Car: scale so the home page exactly fills the height between the status bar
                // and the climate bar; every other screen uses the same scale.
                use std::sync::atomic::Ordering;
                let t = androidback::INSET_T.load(Ordering::Relaxed) as f32;
                let b = androidback::INSET_B.load(Ordering::Relaxed).max(CAR_BAR_PX) as f32;
                (phys_h - t - b) / CAR_HOME_H
            } else {
                phys_w / if landscape { 1000.0 } else { 500.0 }
            }
            .clamp(0.75, 6.0);
            if (want - cur).abs() > 0.01 {
                ctx.set_pixels_per_point(want);
                ctx.request_repaint();
            }
        }
        let (now, dt) = ctx.input(|i| (i.time, i.stable_dt.min(0.1) as f64));
        self.now = now;
        self.frames += 1;
        self.tick(dt);

        // Safe-area insets (status bar, landscape nav bar, display cutout). The portrait window
        // already drops the content below the status bar, so the top inset is applied to the
        // header only in landscape; side insets always keep content clear of a side nav bar.
        #[cfg(target_os = "android")]
        let (il, it, ir, ib): (i8, i8, i8, i8) = {
            use std::sync::atomic::Ordering;
            let ppp = ctx.pixels_per_point().max(0.5);
            let px = |v: u32| (v as f32 / ppp).round().clamp(0.0, 120.0) as i8;
            let g = |a: &std::sync::atomic::AtomicU32| px(a.load(Ordering::Relaxed));
            // Bottom inset only on the car: the DiLink climate bar (90 px, y 990..1080) overlays
            // the app. Fall back to 90 px if the head unit doesn't report it as a system bar.
            let ib = if androidback::CAR.load(Ordering::Relaxed) {
                px(androidback::INSET_B.load(Ordering::Relaxed).max(CAR_BAR_PX))
            } else {
                0
            };
            (g(&androidback::INSET_L), g(&androidback::INSET_T), g(&androidback::INSET_R), ib)
        };
        #[cfg(not(target_os = "android"))]
        let (il, it, ir, ib): (i8, i8, i8, i8) = (0, 0, 0, 0);

        egui::Panel::top("header")
            .exact_size(58.0 + it as f32)
            .resizable(false)
            .show_separator_line(false)
            .frame(egui::Frame::new().inner_margin(egui::Margin { left: il, right: ir, top: it, bottom: 0 }))
            .show(ui, |ui| self.header(ui));

        let screen = self.screen();
        let bg = Color32::from_rgb(0xE9, 0xEE, 0xF5);
        let base: i8 = 16;
        let margin = egui::Margin { left: base + il, right: base + ir, top: base, bottom: base + ib };
        // Screens that need a connected car. Reachable any time, but when not connected they
        // show a "connect first" placeholder instead of demo/simulated data.
        let needs_car = matches!(
            screen,
            Screen::Menu | Screen::Codes | Screen::LiveData | Screen::Reports | Screen::Battery | Screen::Charge | Screen::Sniff | Screen::Check
        );
        egui::CentralPanel::default()
            .frame(egui::Frame::new().fill(bg).inner_margin(margin))
            .show(ui, |ui| {
                if needs_car && !self.real() {
                    self.not_connected(ui);
                    return;
                }
                match screen {
                    Screen::Connect => self.connect(ui),
                    Screen::Check => {
                        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| self.check_ui(ui));
                    }
                    Screen::VinInput => self.vin_input(ui),
                    Screen::Menu => self.menu(ui),
                    Screen::Codes => self.codes(ui),
                    Screen::LiveData => self.live_data(ui),
                    Screen::Reports => self.reports_ui(ui),
                    Screen::Settings => self.settings(ui),
                    Screen::Charge => {
                        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| self.charge_ui(ui));
                    }
                    Screen::Battery => self.battery_ui(ui),
                    Screen::Sniff => self.sniff_ui(ui),
                }
            });

        self.dialogs(&ctx);

        // --screenshot: draw ~120 frames, then save the window picture and quit.
        if let Some(path) = self.screenshot.clone() {
            ctx.request_repaint();
            if self.frames >= 120 && !self.shot_requested {
                self.shot_requested = true;
                let slot = self.shot.clone();
                let c2 = ctx.clone();
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::ScreenshotCallback::new(move |img| {
                    *slot.lock().unwrap() = Some(img);
                    c2.request_repaint();
                })));
            }
            let img = self.shot.lock().unwrap().take();
            if let Some(img) = img {
                if let Err(e) = save_png(&path, &img) {
                    eprintln!("screenshot failed: {e}");
                }
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            if self.frames > 600 {
                eprintln!("screenshot timed out");
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
        } else if matches!(screen, Screen::LiveData | Screen::Charge | Screen::Battery | Screen::Sniff | Screen::Check) || self.sniff.on || self.sniff.resume || self.toast.is_some() || self.link.busy || self.link.state == LinkState::Searching {
            ctx.request_repaint();
        }
    }
}

// ---------------------------------------------------------------- helpers

pub fn check_range(mn: &str, mx: &str) -> Result<(f64, f64), &'static str> {
    if mn.trim().is_empty() || mx.trim().is_empty() {
        return Err("The maximum/minimum value can not be empty");
    }
    let (Ok(a), Ok(b)) = (mn.trim().parse::<f64>(), mx.trim().parse::<f64>()) else {
        return Err("Please enter numbers");
    };
    if a >= b {
        return Err("The minimum value should not larger or equal to the maximum value, please try again");
    }
    Ok((a, b))
}


fn dtc_group(a: u8) -> &'static str {
    match a >> 6 {
        0 => "Powertrain fault (no description database)",
        1 => "Chassis fault (no description database)",
        2 => "Body fault (no description database)",
        _ => "Network / communication fault (no description database)",
    }
}

/// An OBD-II (J1962) connector glyph: the trapezoidal/D-shaped plug with two rows of pins.
fn draw_obd_connector(p: &egui::Painter, r: Rect) {
    let body = Color32::from_rgb(0x3C, 0x45, 0x55);
    let pin = Color32::from_rgb(0xDC, 0xE2, 0xEA);
    let w = r.width();
    let h = r.height();
    let cx = r.center().x;
    let top_w = w * 0.96;
    let bot_w = w * 0.60;
    let top = r.top() + h * 0.14;
    let bot = r.bottom() - h * 0.22;
    let pts = vec![
        Pos2::new(cx - top_w / 2.0, top),
        Pos2::new(cx + top_w / 2.0, top),
        Pos2::new(cx + bot_w / 2.0, bot),
        Pos2::new(cx - bot_w / 2.0, bot),
    ];
    p.add(egui::Shape::convex_polygon(pts, body, Stroke::new(2.0, body)));
    // a slim housing/cable tab under the plug
    let tab = Rect::from_center_size(Pos2::new(cx, bot + h * 0.11), Vec2::new(w * 0.30, h * 0.20));
    p.rect_filled(tab, 4.0, body);
    // 16 pins: 2 rows of 8, each row width following the trapezoid edges
    let rad = (w * 0.028).max(2.2);
    for row in 0..2 {
        let f = 0.30 + row as f32 * 0.34; // vertical position factor inside the body
        let y = top + (bot - top) * f;
        let rw = top_w + (bot_w - top_w) * f;
        let usable = rw * 0.82;
        for i in 0..8 {
            let t = i as f32 / 7.0;
            let x = cx - usable / 2.0 + t * usable;
            p.circle_filled(Pos2::new(x, y), rad, pin);
        }
    }
}

fn rgb(v: u32) -> Color32 {
    Color32::from_rgb((v >> 16) as u8, (v >> 8) as u8, v as u8)
}

fn chip(text: &str) -> egui::Button<'static> {
    egui::Button::new(RichText::new(text.to_string()).color(Color32::WHITE).size(13.0))
        .fill(Color32::from_white_alpha(45))
        .corner_radius(CornerRadius::same(12))
}

/// Modules whose trouble codes only count as "attention" in the full check, even when current
/// (not safety / drivetrain): multimedia head unit (741).
const CHECK_SOFT_MODULES: [u16; 1] = [0x741];

fn soft_module(name: &str) -> bool {
    obd::byd_modules().iter().any(|m| m.name == name && CHECK_SOFT_MODULES.contains(&m.req))
}

/// Steps of the full vehicle check (all read-only), run one after another.
const CHECK_STEPS: usize = 5;

#[derive(Default)]
struct CheckState {
    /// next step to send (0 VIN, 1 trouble codes, 2 battery, 3 battery detail, 4 OBC + 12 V)
    step: usize,
    /// a step's request is out, waiting for its answer
    waiting: bool,
    done: bool,
    v12: Option<f64>,
    /// app time the check started (for the report)
    started: String,
}

/// One tile of the Live Data dashboard.
#[derive(Clone, Copy)]
enum DashCell {
    /// one live value (index into LiveState)
    Num(usize),
    /// SOC dial (value index)
    Soc(usize),
    /// power dial, kW = pack V x A
    Power,
    /// cell temperature: max, average, min (value indices)
    Temps(Option<usize>, Option<usize>, Option<usize>),
    /// cell voltage: max, min, highest cell no, lowest cell no (value indices)
    Volts(Option<usize>, Option<usize>, Option<usize>, Option<usize>),
}

/// Section titles on the black Live Data dashboard.
const HEAD_LIME: Color32 = Color32::from_rgb(0x8C, 0xF5, 0xB0);

/// Arc starting at angle `a0` (radians, screen coordinates: 0 = right, +PI/2 = down) and
/// running clockwise for `sweep` radians.
fn draw_arc(p: &egui::Painter, c: Pos2, r: f32, a0: f32, sweep: f32, col: Color32, width: f32) {
    if sweep <= 0.0 {
        return;
    }
    let n = ((48.0 * sweep) as usize).max(2);
    let pts: Vec<Pos2> = (0..=n)
        .map(|k| {
            let a = a0 + sweep * k as f32 / n as f32;
            c + Vec2::new(a.cos(), a.sin()) * r
        })
        .collect();
    p.add(egui::Shape::line(pts, Stroke::new(width, col)));
}

/// Phone-friendly modal: dims and blocks the screen behind it, centred white card with a big
/// title, the body, and two large buttons (Cancel grey / primary in `accent`). Returns
/// Some(true) for the primary button, Some(false) for Cancel, None while still open.
fn modal(ctx: &egui::Context, id: &str, title: &str, accent: Color32, primary: &str, cancel: &str, body: impl FnOnce(&mut egui::Ui)) -> Option<bool> {
    let screen = ctx.content_rect();
    // dim layer that also swallows taps on the page behind
    egui::Area::new(egui::Id::unique(("modal-dim", id)))
        .order(egui::Order::Middle)
        .fixed_pos(screen.min)
        .show(ctx, |ui| {
            let (r, _) = ui.allocate_exact_size(screen.size(), Sense::click_and_drag());
            ui.painter().rect_filled(r, 0.0, Color32::from_black_alpha(150));
        });
    let w = (screen.width() - 32.0).min(460.0);
    let mut out = None;
    egui::Area::new(egui::Id::unique(("modal", id)))
        .order(egui::Order::Foreground)
        .anchor(Align2::CENTER_CENTER, Vec2::ZERO)
        .show(ctx, |ui| {
            egui::Frame::new()
                .fill(Color32::WHITE)
                .corner_radius(20)
                .inner_margin(22)
                .shadow(egui::epaint::Shadow { offset: [0, 6], blur: 24, spread: 0, color: Color32::from_black_alpha(90) })
                .show(ui, |ui| {
                    ui.set_width(w - 44.0);
                    // accent bar + title
                    let (bar, _) = ui.allocate_exact_size(Vec2::new(48.0, 5.0), Sense::hover());
                    ui.painter().rect_filled(bar, 3.0, accent);
                    ui.add_space(10.0);
                    ui.label(RichText::new(title).size(21.0).strong().color(TEXT_DARK));
                    ui.add_space(10.0);
                    body(ui);
                    ui.add_space(18.0);
                    let bw = (ui.available_width() - 12.0) / 2.0;
                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 12.0;
                        let c = egui::Button::new(RichText::new(cancel).size(18.0).color(TEXT_DARK))
                            .fill(Color32::from_rgb(0xE4, 0xE8, 0xEE))
                            .min_size(Vec2::new(bw, 54.0))
                            .corner_radius(CornerRadius::same(14));
                        if ui.add(c).clicked() {
                            out = Some(false);
                        }
                        let p = egui::Button::new(RichText::new(primary).size(18.0).strong().color(Color32::WHITE))
                            .fill(accent)
                            .min_size(Vec2::new(bw, 54.0))
                            .corner_radius(CornerRadius::same(14));
                        if ui.add(p).clicked() {
                            out = Some(true);
                        }
                    });
                });
        });
    out
}

/// A titled frame on the Charge screen (AC / DC): coloured header bar, tinted body, thick
/// border; `active` adds a "charging" pill to the header.
fn charge_section(ui: &mut egui::Ui, title: &str, accent: Color32, tint: Color32, active: bool, active_label: &str, body: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::new()
        .fill(tint)
        .stroke(Stroke::new(if active { 3.0 } else { 1.5 }, accent))
        .corner_radius(14)
        .inner_margin(egui::Margin { left: 10, right: 10, top: 0, bottom: 10 })
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            // header bar
            let (bar, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), 44.0), Sense::hover());
            let p = ui.painter();
            p.text(bar.left_center() + Vec2::new(4.0, 0.0), Align2::LEFT_CENTER, title, FontId::proportional(18.0), accent);
            if active {
                let pill = Rect::from_min_size(Pos2::new(bar.right() - 104.0, bar.center().y - 13.0), Vec2::new(100.0, 26.0));
                p.rect_filled(pill, 13.0, accent);
                p.text(pill.center(), Align2::CENTER_CENTER, active_label, FontId::proportional(13.5), Color32::WHITE);
            }
            p.line_segment([bar.left_bottom(), bar.right_bottom()], Stroke::new(1.0, accent.gamma_multiply(0.5)));
            ui.add_space(8.0);
            body(ui);
        });
}

fn blue_button(text: &str, size: Vec2) -> egui::Button<'static> {
    egui::Button::new(RichText::new(text.to_string()).color(Color32::WHITE).size(15.0))
        .fill(ACCENT)
        .min_size(size)
        .corner_radius(CornerRadius::same((size.y / 2.0) as u8))
}

fn grey_button(text: &str, size: Vec2) -> egui::Button<'static> {
    egui::Button::new(RichText::new(text.to_string()).color(TEXT_DARK).size(14.0))
        .fill(GREY_TOP)
        .min_size(size)
        .corner_radius(CornerRadius::same((size.y / 2.0) as u8))
}

fn lerp_color(a: Color32, b: Color32, t: f32) -> Color32 {
    let l = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t).round() as u8;
    Color32::from_rgb(l(a.r(), b.r()), l(a.g(), b.g()), l(a.b(), b.b()))
}

fn gradient_rect(p: &egui::Painter, r: Rect, top: Color32, bottom: Color32) {
    let mut m = egui::Mesh::default();
    m.colored_vertex(r.left_top(), top);
    m.colored_vertex(r.right_top(), top);
    m.colored_vertex(r.left_bottom(), bottom);
    m.colored_vertex(r.right_bottom(), bottom);
    m.add_triangle(0, 1, 2);
    m.add_triangle(1, 2, 3);
    p.add(egui::Shape::mesh(m));
}

/// Rounded rectangle with a vertical gradient.
fn rounded_gradient(p: &egui::Painter, r: Rect, radius: f32, top: Color32, bottom: Color32) {
    let rad = radius.min(r.height() / 2.0);
    let h = r.height();
    p.rect_filled(r, rad, top);
    p.rect_filled(
        Rect::from_min_max(Pos2::new(r.left(), r.center().y), r.max),
        CornerRadius { nw: 0, ne: 0, sw: rad as u8, se: rad as u8 },
        bottom,
    );
    let band = Rect::from_min_max(Pos2::new(r.left(), r.top() + rad), Pos2::new(r.right(), r.bottom() - rad));
    if band.height() > 0.0 {
        let t0 = rad / h;
        let t1 = 1.0 - rad / h;
        gradient_rect(p, band, lerp_color(top, bottom, t0), lerp_color(top, bottom, t1));
    }
}

/// Soft wave background for the home screen.

fn draw_icon(p: &egui::Painter, kind: u8, c: Pos2, s: f32) {
    let w = Stroke::new((s * 0.09).max(2.0), Color32::WHITE);
    match kind {
        0 => {
            // magnifier
            p.circle_stroke(c + Vec2::new(-s * 0.15, -s * 0.15), s * 0.55, w);
            p.line_segment([c + Vec2::new(s * 0.25, s * 0.25), c + Vec2::new(s * 0.8, s * 0.8)], Stroke::new(w.width * 1.6, Color32::WHITE));
        }
        1 => {
            // car
            let body = Rect::from_center_size(c + Vec2::new(0.0, s * 0.1), Vec2::new(s * 1.8, s * 0.6));
            p.rect_stroke(body, s * 0.15, w, StrokeKind::Middle);
            let roof = vec![
                c + Vec2::new(-s * 0.55, -s * 0.2),
                c + Vec2::new(-s * 0.3, -s * 0.65),
                c + Vec2::new(s * 0.35, -s * 0.65),
                c + Vec2::new(s * 0.6, -s * 0.2),
            ];
            p.add(egui::Shape::line(roof, w));
            p.circle_filled(c + Vec2::new(-s * 0.5, s * 0.45), s * 0.2, Color32::WHITE);
            p.circle_filled(c + Vec2::new(s * 0.5, s * 0.45), s * 0.2, Color32::WHITE);
        }
        2 => {
            // wave
            let pts: Vec<Pos2> = (0..=24)
                .map(|i| {
                    let t = i as f32 / 24.0;
                    c + Vec2::new((t - 0.5) * s * 1.9, -(t * 12.0).sin() * s * 0.35 * (1.0 - (t - 0.5).abs()))
                })
                .collect();
            p.add(egui::Shape::line(pts, w));
            p.line_segment([c + Vec2::new(-s * 0.95, s * 0.7), c + Vec2::new(s * 0.95, s * 0.7)], w);
        }
        3 => {
            // document
            let r = Rect::from_center_size(c, Vec2::new(s * 1.2, s * 1.6));
            p.rect_stroke(r, s * 0.1, w, StrokeKind::Middle);
            for k in 0..3 {
                let y = r.top() + s * 0.45 + k as f32 * s * 0.35;
                p.line_segment([Pos2::new(r.left() + s * 0.25, y), Pos2::new(r.right() - s * 0.25, y)], w);
            }
        }
        4 => {
            // update arrow in circle
            p.circle_stroke(c, s * 0.8, w);
            p.line_segment([c + Vec2::new(0.0, s * 0.45), c + Vec2::new(0.0, -s * 0.45)], w);
            p.line_segment([c + Vec2::new(0.0, -s * 0.45), c + Vec2::new(-s * 0.3, -s * 0.15)], w);
            p.line_segment([c + Vec2::new(0.0, -s * 0.45), c + Vec2::new(s * 0.3, -s * 0.15)], w);
        }
        6 => {
            // battery
            let body = Rect::from_center_size(c, Vec2::new(s * 1.5, s * 0.95));
            p.rect_stroke(body, s * 0.12, w, StrokeKind::Middle);
            // terminal cap on the right
            let cap = Rect::from_min_size(Pos2::new(body.right(), c.y - s * 0.22), Vec2::new(s * 0.14, s * 0.44));
            p.rect_filled(cap, s * 0.05, Color32::WHITE);
            // charge bars
            for k in 0..3 {
                let x = body.left() + s * 0.28 + k as f32 * s * 0.4;
                p.rect_filled(Rect::from_min_size(Pos2::new(x, c.y - s * 0.3), Vec2::new(s * 0.18, s * 0.6)), s * 0.03, Color32::WHITE);
            }
        }
        7 => {
            // warning triangle
            let t = vec![
                c + Vec2::new(0.0, -s * 0.8),
                c + Vec2::new(s * 0.85, s * 0.6),
                c + Vec2::new(-s * 0.85, s * 0.6),
                c + Vec2::new(0.0, -s * 0.8),
            ];
            p.add(egui::Shape::line(t, w));
            p.line_segment([c + Vec2::new(0.0, -s * 0.3), c + Vec2::new(0.0, s * 0.18)], Stroke::new(w.width * 1.3, Color32::WHITE));
            p.circle_filled(c + Vec2::new(0.0, s * 0.42), w.width * 0.9, Color32::WHITE);
        }
        8 => {
            // lightning bolt
            let b = vec![
                c + Vec2::new(s * 0.15, -s * 0.85),
                c + Vec2::new(-s * 0.45, s * 0.1),
                c + Vec2::new(s * 0.02, s * 0.1),
                c + Vec2::new(-s * 0.15, s * 0.85),
                c + Vec2::new(s * 0.5, -s * 0.15),
                c + Vec2::new(s * 0.02, -s * 0.15),
            ];
            p.add(egui::Shape::convex_polygon(b, Color32::WHITE, Stroke::NONE));
        }
        9 => {
            // broadcast / signal
            p.circle_filled(c + Vec2::new(0.0, s * 0.45), s * 0.22, Color32::WHITE);
            for k in 1..=2 {
                let rr = s * (0.28 + k as f32 * 0.3);
                p.add(egui::Shape::line(
                    (0..=14).map(|i| {
                        let a = std::f32::consts::PI * (1.15 + i as f32 / 14.0 * 0.7);
                        c + Vec2::new(0.0, s * 0.45) + Vec2::new(a.cos() * rr, a.sin() * rr)
                    }).collect(),
                    w,
                ));
            }
        }
        _ => {
            // gear
            p.circle_stroke(c, s * 0.5, w);
            p.circle_stroke(c, s * 0.2, w);
            for k in 0..8 {
                let a = k as f32 * std::f32::consts::TAU / 8.0;
                let d = Vec2::new(a.cos(), a.sin());
                p.line_segment([c + d * s * 0.5, c + d * s * 0.8], Stroke::new(w.width * 1.5, Color32::WHITE));
            }
        }
    }
}

/// DiLink climate bar height (physical px) when the head unit doesn't report it as an inset.
#[cfg(target_os = "android")]
const CAR_BAR_PX: u32 = 90;
/// Logical height of the car home page (header + margins + hero + Bluetooth card + 4 tile
/// rows of ~90 pt + credit). The car UI scale = available height / this, so the home page
/// fills the screen; the tiles flex to absorb the rest.
#[cfg(target_os = "android")]
const CAR_HOME_H: f32 = 802.0;

/// Car build flavor (DiLink head unit)?
fn car_mode() -> bool {
    #[cfg(target_os = "android")]
    return androidback::CAR.load(std::sync::atomic::Ordering::Relaxed);
    #[cfg(not(target_os = "android"))]
    false
}

fn install_fonts(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();
    let candidates = [
        "/System/Library/Fonts/Supplemental/Tahoma.ttf",
        "/Library/Fonts/Tahoma.ttf",
        "C:\\Windows\\Fonts\\tahoma.ttf",
        "/usr/share/fonts/truetype/tlwg/Loma.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansThai-Regular.ttf",
        "/system/fonts/NotoSansThaiUI-Regular.ttf",
        "/system/fonts/NotoSansThai-Regular.ttf",
    ];
    for path in candidates {
        if let Ok(bytes) = std::fs::read(path) {
            fonts.font_data.insert("thai".into(), Arc::new(egui::FontData::from_owned(bytes)));
            for fam in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                fonts.families.entry(fam).or_default().push("thai".into());
            }
            break;
        }
    }
    ctx.set_fonts(fonts);
}

fn out_dir() -> PathBuf {
    #[cfg(target_os = "android")]
    if let Some(d) = androidback::files_dir() {
        return d;
    }
    let home = std::env::var("HOME").unwrap_or_else(|_| ".".into());
    PathBuf::from(home).join("Documents").join("ScanLab")
}

fn save_file(stem: &str, ext: &str, text: &str) -> Result<String, String> {
    let dir = out_dir();
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let p = dir.join(format!("{stem}_{secs}.{ext}"));
    std::fs::write(&p, text).map_err(|e| e.to_string())?;
    Ok(p.display().to_string())
}

pub(crate) fn clock_text() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let days = secs / 86400;
    let (y, m, d) = civil_from_days(days as i64);
    let rem = secs % 86400;
    // "%.4d-%.2d-%.2d" (@DATE) and "%.2lu:%.2lu:%.2lu" (@TIME, seconds / 3600)
    format!("{y:04}-{m:02}-{d:02} {:02}:{:02}:{:02} UTC", rem / SECS_PER_HOUR, (rem % SECS_PER_HOUR) / 60, rem % 60)
}

const SECS_PER_HOUR: u64 = 3600;

fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

fn save_png(path: &std::path::Path, img: &egui::ColorImage) -> Result<(), String> {
    if let Some(dir) = path.parent() {
        if !dir.as_os_str().is_empty() {
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        }
    }
    let [w, h] = img.size;
    let mut buf = Vec::with_capacity(w * h * 4);
    for p in &img.pixels {
        let [r, g, b, _] = p.to_srgba_unmultiplied();
        buf.extend_from_slice(&[r, g, b, 255]);
    }
    image::save_buffer(path, &buf, w as u32, h as u32, image::ExtendedColorType::Rgba8).map_err(|e| e.to_string())
}

#[cfg(test)]
mod tests {
    use super::check_range;
    #[test]
    fn ranges() {
        assert!(check_range("", "5").is_err());
        assert!(check_range("5", "5").is_err());
        assert_eq!(check_range("1", "5"), Ok((1.0, 5.0)));
    }
}

#[cfg(test)]
mod batt_detail_tests {
    use super::BattDetail;
    #[test]
    fn parse_from_car_scan() {
        // bytes from this car's 7E7 scans (docs/did_scan_7E7_*.csv)
        let mut list = vec![
            (0x1FFC, vec![0x10, 0x18, 0xE9, 0x33]),
            (0x1FFE, vec![0x64, 0x19, 0x28, 0x3C]),
            (0x0040, vec![0xB2, 0x03]),
            (0x1100, vec![0x00, 0x02, 0x00, 0x00]), // cell 10 flagged
            (0x1101, vec![0, 0, 0, 0]),
            (0x1102, vec![0, 0, 0, 0]),
            (0x1103, vec![0, 0, 0, 0x80]), // bit 31 = cell 128 -> beyond 126, dropped
        ];
        list.push((0x00BD, vec![0xA2, 0x03]));
        let d = BattDetail::parse(&list);
        assert!((d.cap_now - 132.89).abs() < 1e-9);
        assert!((d.cap_factory - 154.0).abs() < 1e-9);
        assert_eq!(d.balance_h.len(), 126);
        assert_eq!(d.balance_h[0], 946.0);
        assert_eq!(d.balance_h[125], 930.0);
        let bad = d.cell_bad.unwrap();
        assert_eq!(bad.len(), 126);
        assert_eq!(bad.iter().filter(|b| **b).count(), 1);
        assert!(bad[9]);
        assert_eq!(d.temp_bad, None); // 110A/110B not in the list
    }
}
