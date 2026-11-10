//! VIN: offline ISO 3779 decode (maker from a small WMI table, region, model year, plant,
//! serial). No online VIN database is used.

/// What a VIN says by itself (ISO 3779), without any database.
pub struct VinInfo {
    pub maker: &'static str,
    pub region: &'static str,
    pub year: Option<u32>,
    pub plant: char,
    pub serial: String,
}

/// Decode a 17-character VIN offline: maker (small WMI table), region, model year
/// (position 10), plant (position 11) and serial (12–17). None if it is not a valid VIN.
pub fn decode_vin(vin: &str) -> Option<VinInfo> {
    let v: Vec<char> = vin.trim().to_uppercase().chars().collect();
    if v.len() != 17 || v.iter().any(|c| !c.is_ascii_alphanumeric() || matches!(c, 'I' | 'O' | 'Q')) {
        return None;
    }
    let wmi: String = v[..3].iter().collect();
    let maker = match wmi.as_str() {
        "LGX" | "LC0" => "BYD",
        "5YJ" | "7SA" | "LRW" | "XP7" => "Tesla",
        "JTD" | "JTE" | "JTM" | "JTN" | "MR0" | "MR1" | "MR2" => "Toyota",
        "MRH" | "JHM" => "Honda",
        "MNB" | "1FA" | "1FT" => "Ford",
        "WDB" | "WDD" | "W1K" => "Mercedes-Benz",
        "WBA" | "WBS" => "BMW",
        "WVW" | "WV1" | "WV2" => "Volkswagen",
        "LSG" => "SAIC-GM",
        "LVG" => "GAC Toyota",
        "LFV" => "FAW-Volkswagen",
        _ => "",
    };
    let region = match v[0] {
        'A'..='H' => "Africa",
        'J' => "Japan",
        'K' => "Korea",
        'L' => "China",
        'M' => "India / Indonesia / Thailand",
        'N' | 'P' | 'R' => "Asia",
        'S'..='Z' => "Europe",
        '1' | '4' | '5' => "USA",
        '2' => "Canada",
        '3' => "Mexico",
        '6' | '7' => "Oceania",
        '8' | '9' => "South America",
        _ => "",
    };
    // model-year code, 30-year cycle: A = 2010 … Y = 2030, 1–9 = 2001–2009
    const LETTERS: &str = "ABCDEFGHJKLMNPRSTVWXY";
    let year = match v[9] {
        d @ '1'..='9' => Some(2000 + d.to_digit(10)?),
        c => LETTERS.find(c).map(|i| 2010 + i as u32),
    };
    Some(VinInfo { maker, region, year, plant: v[10], serial: v[11..].iter().collect() })
}

#[cfg(test)]
mod decode_tests {
    use super::decode_vin;
    #[test]
    fn byd_vin() {
        let i = decode_vin("LGXCE4CB5P0123456").unwrap();
        assert_eq!(i.maker, "BYD");
        assert_eq!(i.region, "China");
        assert_eq!(i.year, Some(2023));
        assert_eq!(i.plant, '0');
        assert_eq!(i.serial, "123456");
        assert!(decode_vin("LGXCE4CB5P012345").is_none()); // 16 chars
        assert!(decode_vin("LGXCE4CB5P012345O").is_none()); // O is not allowed
    }
}
