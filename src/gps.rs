//! uBlox NEO-M8N GPS driver — NMEA-0183 line parser.
//!
//! The module is wired to **USART1** (PA9 TX / PA10 RX), which the ArduPilot
//! hwdef assigns as the GPS serial port (`DEFAULT_SERIAL1_PROTOCOL = GPS`).
//!
//! For bring-up we parse the module's *default* NMEA output (ASCII, emitted with
//! zero configuration at the factory baud) rather than UBX binary. That makes
//! "plug it in and watch the fix appear" work immediately. The richer UBX
//! NAV-PVT path (which also gives 3D velocity for the EKF) is a Phase-2 add-on;
//! the parser here already yields everything `GPS_RAW_INT` needs.
//!
//! Two sentences are decoded and merged into one [`GpsData`]:
//!   * **GGA** — fix quality, satellite count, lat/lon, MSL altitude, HDOP
//!   * **RMC** — ground speed, course over ground, validity
//!
//! Units are chosen to map straight onto MAVLink `GPS_RAW_INT` (#24).

/// Latest merged GPS solution. Units match MAVLink `GPS_RAW_INT`.
#[derive(Clone, Copy)]
pub struct GpsData {
    /// 0/1 = no fix, 2 = 2D, 3 = 3D (we report 0 or 3 from NMEA fix quality).
    pub fix_type: u8,
    /// Satellites used in the solution.
    pub sats: u8,
    /// Latitude, degrees * 1e7 (WGS-84).
    pub lat_e7: i32,
    /// Longitude, degrees * 1e7.
    pub lon_e7: i32,
    /// MSL altitude, millimetres.
    pub alt_mm: i32,
    /// Horizontal dilution of precision * 100 (65535 = unknown).
    pub eph: u16,
    /// Ground speed, cm/s.
    pub vel_cms: u16,
    /// Course over ground, centidegrees (0..35999, 65535 = unknown).
    pub cog_cdeg: u16,
    /// Count of complete sentences successfully parsed (liveness/debug).
    pub sentences: u32,
}

impl Default for GpsData {
    fn default() -> Self {
        Self {
            fix_type: 0,
            sats: 0,
            lat_e7: 0,
            lon_e7: 0,
            alt_mm: 0,
            eph: u16::MAX,
            vel_cms: 0,
            cog_cdeg: u16::MAX,
            sentences: 0,
        }
    }
}

const MAX_SENTENCE: usize = 100;

/// Byte-fed NMEA sentence assembler + decoder. Feed it raw UART bytes; it buffers
/// one `$...*CC<CR><LF>` sentence at a time, verifies the checksum, and updates
/// the running [`GpsData`].
pub struct NmeaParser {
    buf: [u8; MAX_SENTENCE],
    len: usize,
    active: bool,
    data: GpsData,
}

impl NmeaParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_SENTENCE],
            len: 0,
            active: false,
            data: GpsData {
                fix_type: 0,
                sats: 0,
                lat_e7: 0,
                lon_e7: 0,
                alt_mm: 0,
                eph: u16::MAX,
                vel_cms: 0,
                cog_cdeg: u16::MAX,
                sentences: 0,
            },
        }
    }

    pub fn data(&self) -> GpsData {
        self.data
    }

    /// Feed one received byte. Returns `true` when a complete, checksum-valid
    /// sentence was just decoded (caller may publish `data()`).
    pub fn push(&mut self, byte: u8) -> bool {
        match byte {
            b'$' => {
                // Start of a new sentence — reset the buffer.
                self.active = true;
                self.len = 0;
                false
            }
            b'\r' | b'\n' => {
                let complete = self.active && self.len > 0 && self.process();
                self.active = false;
                self.len = 0;
                complete
            }
            _ => {
                if self.active {
                    if self.len < MAX_SENTENCE {
                        self.buf[self.len] = byte;
                        self.len += 1;
                    } else {
                        // Overrun — drop this sentence.
                        self.active = false;
                        self.len = 0;
                    }
                }
                false
            }
        }
    }

    /// Validate checksum and dispatch the buffered sentence. Returns whether a
    /// known sentence was decoded.
    fn process(&mut self) -> bool {
        // Split body and "*CC" checksum.
        let star = self.buf[..self.len].iter().position(|&b| b == b'*');
        let body_end = match star {
            Some(i) => i,
            None => return false, // no checksum — reject
        };
        if body_end + 2 >= self.len {
            return false;
        }
        let mut sum: u8 = 0;
        for &b in &self.buf[..body_end] {
            sum ^= b;
        }
        let given = hex2(self.buf[body_end + 1], self.buf[body_end + 2]);
        if Some(sum) != given {
            return false;
        }

        // Sentence type is the 3 chars after the 2-char talker id.
        if body_end < 5 {
            return false;
        }
        // Split the borrows: the body slice reads `buf` while the parsers write
        // `data` — disjoint fields, so destructure to satisfy the borrow checker.
        let Self { buf, data, .. } = self;
        let kind = &buf[2..5];
        let body = &buf[..body_end];
        if kind == b"GGA" {
            parse_gga(data, body);
            data.sentences = data.sentences.wrapping_add(1);
            true
        } else if kind == b"RMC" {
            parse_rmc(data, body);
            data.sentences = data.sentences.wrapping_add(1);
            true
        } else {
            false
        }
    }
}

/// GGA: ...,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
fn parse_gga(data: &mut GpsData, body: &[u8]) {
    let f = Fields::new(body);
    let quality = f.get(6).and_then(parse_u32).unwrap_or(0);
    data.fix_type = if quality >= 1 { 3 } else { 0 };
    if let Some(s) = f.get(7).and_then(parse_u32) {
        data.sats = s as u8;
    }
    if let (Some(lat), Some(ns)) = (f.get(2), f.get(3)) {
        if let Some(v) = parse_lat_lon(lat, 2) {
            data.lat_e7 = if ns == b"S" { -v } else { v };
        }
    }
    if let (Some(lon), Some(ew)) = (f.get(4), f.get(5)) {
        if let Some(v) = parse_lat_lon(lon, 3) {
            data.lon_e7 = if ew == b"W" { -v } else { v };
        }
    }
    if let Some(alt) = f.get(9).and_then(parse_f32) {
        data.alt_mm = (alt * 1000.0) as i32;
    }
    if let Some(hdop) = f.get(8).and_then(parse_f32) {
        data.eph = (hdop * 100.0) as u16;
    }
}

/// RMC: ...,time,status,lat,N/S,lon,E/W,speed_knots,course,date,...
fn parse_rmc(data: &mut GpsData, body: &[u8]) {
    let f = Fields::new(body);
    if let Some(knots) = f.get(7).and_then(parse_f32) {
        // 1 knot = 51.4444 cm/s.
        data.vel_cms = (knots * 51.4444) as u16;
    }
    if let Some(course) = f.get(8).and_then(parse_f32) {
        data.cog_cdeg = (course * 100.0) as u16;
    }
}

/// Comma-separated field accessor over a sentence body (zero-copy).
struct Fields<'a> {
    body: &'a [u8],
}

impl<'a> Fields<'a> {
    fn new(body: &'a [u8]) -> Self {
        Self { body }
    }

    /// Field `n`, counting the sentence type as field 0.
    fn get(&self, n: usize) -> Option<&'a [u8]> {
        self.body.split(|&b| b == b',').nth(n).filter(|s| !s.is_empty())
    }
}

/// Two ASCII hex digits → byte.
fn hex2(hi: u8, lo: u8) -> Option<u8> {
    Some(hexval(hi)? << 4 | hexval(lo)?)
}

fn hexval(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'A'..=b'F' => Some(c - b'A' + 10),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    }
}

fn parse_u32(s: &[u8]) -> Option<u32> {
    let txt = core::str::from_utf8(s).ok()?;
    txt.parse::<u32>().ok()
}

fn parse_f32(s: &[u8]) -> Option<f32> {
    let txt = core::str::from_utf8(s).ok()?;
    txt.parse::<f32>().ok()
}

/// Parse an NMEA `ddmm.mmmm` (or `dddmm.mmmm`) angle into degrees * 1e7.
/// `deg_digits` is 2 for latitude, 3 for longitude.
fn parse_lat_lon(s: &[u8], deg_digits: usize) -> Option<i32> {
    if s.len() < deg_digits {
        return None;
    }
    let txt = core::str::from_utf8(s).ok()?;
    let deg: f32 = txt.get(..deg_digits)?.parse().ok()?;
    let min: f32 = txt.get(deg_digits..)?.parse().ok()?;
    Some(((deg + min / 60.0) * 1.0e7) as i32)
}
