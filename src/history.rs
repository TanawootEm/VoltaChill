//! Battery history and saved reports, kept as plain text files in the app's data folder so they
//! survive a restart (and can be pulled off the phone with adb):
//! - `history.csv`  one row per snapshot (full check or Battery-screen detail read)
//! - `balance.csv`  the 126 per-cell balancing hours of the same snapshots
//! - `reports.txt`  the Reports list (newest first)
//!
//! Missing / unreadable values are written as empty fields and read back as NaN.

use std::io::Write;
use std::path::Path;

pub const HISTORY_FILE: &str = "history.csv";
pub const BALANCE_FILE: &str = "balance.csv";
pub const REPORTS_FILE: &str = "reports.txt";

const HISTORY_HEADER: &str = "unix,time,vin,source,soh,cap_now_ah,cap_factory_ah,cell_spread_mv,temp_min_c,temp_max_c,insulation_kohm,charged_kwh,discharged_kwh,charges,v12,dtc_total,dtc_current,cell_faults";

/// One battery snapshot.
#[derive(Clone, Debug, PartialEq)]
pub struct Snap {
    pub unix: i64,
    pub time: String,
    pub vin: String,
    /// "check" (full vehicle check) or "battery" (Battery screen)
    pub source: String,
    pub soh: f64,
    pub cap_now: f64,
    pub cap_factory: f64,
    pub spread_mv: f64,
    pub tmin: f64,
    pub tmax: f64,
    pub insul_kohm: f64,
    pub chg_kwh: f64,
    pub dis_kwh: f64,
    pub charges: f64,
    pub v12: f64,
    pub dtc_total: f64,
    pub dtc_current: f64,
    pub cell_faults: f64,
}

impl Snap {
    pub fn cap_left_pct(&self) -> f64 {
        self.cap_now / self.cap_factory * 100.0
    }
}

pub fn now_unix() -> i64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

fn num(x: f64) -> String {
    if x.is_finite() {
        format!("{x}")
    } else {
        String::new()
    }
}

fn parse(x: &str) -> f64 {
    x.trim().parse().unwrap_or(f64::NAN)
}

/// Text fields are kept free of the separator.
fn clean(s: &str) -> String {
    s.replace([',', '\t', '\n', '\r'], " ")
}

pub fn snap_line(s: &Snap) -> String {
    [
        s.unix.to_string(),
        clean(&s.time),
        clean(&s.vin),
        clean(&s.source),
        num(s.soh),
        num(s.cap_now),
        num(s.cap_factory),
        num(s.spread_mv),
        num(s.tmin),
        num(s.tmax),
        num(s.insul_kohm),
        num(s.chg_kwh),
        num(s.dis_kwh),
        num(s.charges),
        num(s.v12),
        num(s.dtc_total),
        num(s.dtc_current),
        num(s.cell_faults),
    ]
    .join(",")
}

pub fn parse_snap(line: &str) -> Option<Snap> {
    let f: Vec<&str> = line.split(',').collect();
    if f.len() < 18 {
        return None;
    }
    Some(Snap {
        unix: f[0].trim().parse().ok()?,
        time: f[1].to_string(),
        vin: f[2].to_string(),
        source: f[3].to_string(),
        soh: parse(f[4]),
        cap_now: parse(f[5]),
        cap_factory: parse(f[6]),
        spread_mv: parse(f[7]),
        tmin: parse(f[8]),
        tmax: parse(f[9]),
        insul_kohm: parse(f[10]),
        chg_kwh: parse(f[11]),
        dis_kwh: parse(f[12]),
        charges: parse(f[13]),
        v12: parse(f[14]),
        dtc_total: parse(f[15]),
        dtc_current: parse(f[16]),
        cell_faults: parse(f[17]),
    })
}

fn append_line(path: &Path, header: &str, line: &str) -> std::io::Result<()> {
    let new = !path.exists();
    let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path)?;
    if new {
        writeln!(f, "{header}")?;
    }
    writeln!(f, "{line}")
}

/// Append one snapshot (+ its per-cell balancing hours, if read) to the history files.
pub fn append(dir: &Path, s: &Snap, balance_h: Option<&[f64]>) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    append_line(&dir.join(HISTORY_FILE), HISTORY_HEADER, &snap_line(s))?;
    if let Some(b) = balance_h {
        let header = std::iter::once("unix".to_string()).chain((1..=b.len()).map(|i| format!("cell{i}_h"))).collect::<Vec<_>>().join(",");
        let line = std::iter::once(s.unix.to_string()).chain(b.iter().map(|x| num(*x))).collect::<Vec<_>>().join(",");
        append_line(&dir.join(BALANCE_FILE), &header, &line)?;
    }
    Ok(())
}

/// Snapshots (oldest first) and balancing rows (unix, hours per cell).
pub fn load(dir: &Path) -> (Vec<Snap>, Vec<(i64, Vec<f64>)>) {
    let snaps = std::fs::read_to_string(dir.join(HISTORY_FILE))
        .map(|t| t.lines().skip(1).filter_map(parse_snap).collect())
        .unwrap_or_default();
    let bal = std::fs::read_to_string(dir.join(BALANCE_FILE))
        .map(|t| {
            t.lines()
                .skip(1)
                .filter_map(|l| {
                    let mut it = l.split(',');
                    let unix = it.next()?.trim().parse().ok()?;
                    Some((unix, it.map(parse).collect()))
                })
                .collect()
        })
        .unwrap_or_default();
    (snaps, bal)
}

/// Cells whose balancing time grew the most between the first and the last row:
/// (cell number 1-based, hours added), largest first.
pub fn balance_growth(rows: &[(i64, Vec<f64>)], top: usize) -> Vec<(usize, f64)> {
    let (Some(first), Some(last)) = (rows.first(), rows.last()) else { return Vec::new() };
    if rows.len() < 2 {
        return Vec::new();
    }
    let mut d: Vec<(usize, f64)> = first
        .1
        .iter()
        .zip(&last.1)
        .enumerate()
        .filter(|(_, (a, b))| a.is_finite() && b.is_finite())
        .map(|(i, (a, b))| (i + 1, b - a))
        .collect();
    d.sort_by(|x, y| y.1.total_cmp(&x.1));
    d.truncate(top);
    d
}

/// A saved report: time, VIN, title, summary, rows of (item, value, text, status).
pub type ReportRec = (String, String, String, String, Vec<(String, String, String, String)>);

/// reports.txt: a "R\t…" line per report then "-\t…" lines for its rows.
pub fn save_reports(dir: &Path, reports: &[ReportRec]) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let t = |s: &str| s.replace(['\t', '\n', '\r'], " ");
    let mut out = String::new();
    for (time, vin, title, summary, rows) in reports {
        out += &format!("R\t{}\t{}\t{}\t{}\n", t(time), t(vin), t(title), t(summary));
        for (a, b, c, d) in rows {
            out += &format!("-\t{}\t{}\t{}\t{}\n", t(a), t(b), t(c), t(d));
        }
    }
    std::fs::write(dir.join(REPORTS_FILE), out)
}

pub fn load_reports(dir: &Path) -> Vec<ReportRec> {
    let Ok(text) = std::fs::read_to_string(dir.join(REPORTS_FILE)) else { return Vec::new() };
    let mut out: Vec<ReportRec> = Vec::new();
    for l in text.lines() {
        let f: Vec<&str> = l.split('\t').collect();
        match (f.first(), f.len()) {
            (Some(&"R"), 5) => out.push((f[1].into(), f[2].into(), f[3].into(), f[4].into(), Vec::new())),
            (Some(&"-"), 5) => {
                if let Some(r) = out.last_mut() {
                    r.4.push((f[1].into(), f[2].into(), f[3].into(), f[4].into()));
                }
            }
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(unix: i64) -> Snap {
        Snap {
            unix,
            time: "09/11 22:18".into(),
            vin: "LGXCE4CB0P0000000".into(),
            source: "check".into(),
            soh: 89.0,
            cap_now: 132.89,
            cap_factory: 154.0,
            spread_mv: 5.0,
            tmin: 30.0,
            tmax: 31.0,
            insul_kohm: 4615.0,
            chg_kwh: 28508.0,
            dis_kwh: 28595.0,
            charges: 955.0,
            v12: f64::NAN,
            dtc_total: 5.0,
            dtc_current: 1.0,
            cell_faults: 0.0,
        }
    }

    #[test]
    fn roundtrip_files() {
        let dir = std::env::temp_dir().join(format!("voltachill-hist-{}", now_unix()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut b1 = vec![900.0; 126];
        let mut b2 = vec![900.0; 126];
        b1[40] = 950.0;
        b2[40] = 980.0; // cell 41 +30 h
        b2[7] = 912.0; // cell 8 +12 h
        append(&dir, &snap(100), Some(&b1)).unwrap();
        append(&dir, &snap(200), Some(&b2)).unwrap();
        let (s, b) = load(&dir);
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].unix, 100);
        assert!(s[1].v12.is_nan());
        assert!((s[1].cap_left_pct() - 86.29).abs() < 0.01);
        assert_eq!(b.len(), 2);
        let g = balance_growth(&b, 2);
        assert_eq!(g, vec![(41, 30.0), (8, 12.0)]);

        let reps: Vec<ReportRec> = vec![("t1".into(), "VIN".into(), "Full check".into(), "Result: OK".into(), vec![("SOH".into(), "89 %".into(), "".into(), "OK".into())])];
        save_reports(&dir, &reps).unwrap();
        assert_eq!(load_reports(&dir), reps);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
