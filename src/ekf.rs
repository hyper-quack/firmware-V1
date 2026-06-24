//! Navigation filter — loosely-coupled INS/GNSS Kalman filter.
//!
//! # What this is (and what it isn't)
//!
//! PX4's `ekf2` is a single, tightly-coupled 24-state error-state EKF that
//! estimates attitude, velocity, position, and all the sensor biases together.
//! It is powerful but large and numerically delicate. This firmware takes the
//! **loosely-coupled** approach instead, which is simpler, easier to verify, and
//! a very common architecture in practice:
//!
//! ```text
//!   IMU ─► Mahony AHRS (attitude.rs/ahrs.rs)  ─► attitude + world-frame accel
//!                                                      │
//!   GPS, baro, lidar, optical-flow ─────────────►  this EKF  ─► position + velocity
//! ```
//!
//! * **Attitude** is estimated by the [`crate::ahrs`] Mahony filter (gyro + accel
//!   + mag). We trust it and *use* it here — we do not re-estimate orientation.
//! * **Position & velocity** are estimated by the Kalman filter in this module,
//!   driven by the accelerometer (rotated into the world frame by the AHRS) and
//!   corrected by GPS / baro / lidar / optical-flow.
//!
//! # The key simplification: three independent axes
//!
//! A full position+velocity filter is 6 states with a 6×6 covariance. But with a
//! constant-acceleration motion model and direct position/velocity measurements,
//! **the three spatial axes do not couple** — North, East and Up evolve and are
//! measured independently. So the 6-state filter cleanly factorises into **three
//! identical 2-state filters** (one [`Axis1D`] each), with tiny 2×2 covariances
//! we can write out by hand. This is mathematically equivalent to the block-
//! diagonal 6-state filter and is far easier to read and debug.
//!
//! Each axis state is `x = [position, velocity]`.
//!
//! # World frame & the north-alignment caveat
//!
//! The world frame here matches the AHRS: **X, Y horizontal, Z up**, gravity
//! removed. For horizontal fusion to be correct, the AHRS heading (X axis) must
//! point to the same "north" the GPS uses — i.e. the **compass must be calibrated
//! (orientation + declination)**. Until then, *vertical* fusion (baro / lidar /
//! GPS-altitude + accel-Z) is fully valid; *horizontal* fusion is structurally
//! correct but rotated by the heading error. This is the same calibration
//! dependency documented for the compass in `docs/sensor-fusion.md`.
//!
//! Output messages use NED (Z down), so Up is negated on the way out.

const EARTH_RADIUS: f32 = 6_378_137.0; // m (WGS-84 equatorial)
const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
const RAD2DEG: f32 = 180.0 / core::f32::consts::PI;

// ---- Tuning (see docs/ekf.md §tuning) ---------------------------------------
/// Accelerometer process-noise PSD, (m/s²). Larger ⇒ trusts the IMU prediction
/// less and the aiding sensors more.
const Q_ACCEL: f32 = 0.5;
/// Accel-bias random-walk PSD, (m/s²)/√s. How fast the estimated bias is allowed
/// to wander. Small ⇒ bias is treated as nearly constant.
const Q_ACCEL_BIAS: f32 = 0.02;
/// GPS horizontal position measurement noise, metres (1σ), before HDOP scaling.
const R_GPS_POS: f32 = 2.5;
/// GPS horizontal velocity measurement noise, m/s (1σ).
const R_GPS_VEL: f32 = 0.5;
/// Barometric altitude measurement noise, metres (1σ).
const R_BARO: f32 = 1.5;
/// Lidar height measurement noise, metres (1σ) — precise near the ground.
const R_LIDAR: f32 = 0.05;
/// Optical-flow velocity measurement noise, m/s (1σ) at full quality.
const R_FLOW: f32 = 0.3;

/// One spatial axis: a **3-state** (position, velocity, accel-bias) Kalman filter
/// with a constant-acceleration motion model. The covariance `P` is the symmetric
/// 3×3 stored as its upper triangle `[p00,p01,p02; p11,p12; p22]`.
///
/// The bias state `ba` absorbs a slowly-varying world-frame acceleration error
/// (gravity-removal residual, accel scale/offset, small attitude error) so it
/// stops leaking into velocity between aiding updates. True acceleration used for
/// integration is `a − ba`.
///
/// All covariance updates use the symmetric form `P −= (P Hᵀ)(H P) / S`, which
/// keeps `P` symmetric exactly (no Joseph form / re-symmetrisation needed).
#[derive(Clone, Copy)]
pub struct Axis1D {
    pub pos: f32,
    pub vel: f32,
    pub bias: f32,
    p00: f32,
    p01: f32,
    p02: f32,
    p11: f32,
    p12: f32,
    p22: f32,
}

impl Axis1D {
    const fn new() -> Self {
        // Large pos/vel variance ("unknown"); modest bias variance.
        Self {
            pos: 0.0,
            vel: 0.0,
            bias: 0.0,
            p00: 100.0,
            p01: 0.0,
            p02: 0.0,
            p11: 100.0,
            p12: 0.0,
            p22: 1.0,
        }
    }

    /// Predict forward by `dt` under measured acceleration `a` (m/s²).
    ///
    /// Motion model with `F = [[1, dt, −½dt²], [0, 1, −dt], [0, 0, 1]]` (the bias
    /// columns subtract the estimated bias from the integrated acceleration) and
    /// control `B = [½dt², dt, 0]`. Covariance `P' = F P Fᵀ + Q`, where `Q` is the
    /// accelerometer white noise on pos/vel plus a bias random walk on the bias.
    fn predict(&mut self, a: f32, dt: f32, sigma_a: f32, sigma_b: f32) {
        let c = -0.5 * dt * dt; // ∂pos/∂bias
        let d = -dt; // ∂vel/∂bias
        let acc = a - self.bias;

        // State.
        self.pos += self.vel * dt + 0.5 * acc * dt * dt;
        self.vel += acc * dt;
        // bias unchanged (random walk handled by Q).

        // Covariance P' = F P Fᵀ. M = F P (only the rows we need for the upper
        // triangle: row0 and row1; row2 of M is the bottom row of P unchanged).
        let (p00, p01, p02, p11, p12, p22) =
            (self.p00, self.p01, self.p02, self.p11, self.p12, self.p22);
        let m00 = p00 + dt * p01 + c * p02;
        let m01 = p01 + dt * p11 + c * p12;
        let m02 = p02 + dt * p12 + c * p22;
        let m11 = p11 + d * p12;
        let m12 = p12 + d * p22;
        let m22 = p22;
        // P' = M Fᵀ (columns of Fᵀ are rows of F).
        let np00 = m00 + dt * m01 + c * m02;
        let np01 = m01 + d * m02;
        let np02 = m02;
        let np11 = m11 + d * m12;
        let np12 = m12;
        let np22 = m22;

        // + Q.
        let sa = sigma_a * sigma_a;
        let dt2 = dt * dt;
        self.p00 = np00 + sa * dt2 * dt2 * 0.25;
        self.p01 = np01 + sa * dt2 * dt * 0.5;
        self.p02 = np02;
        self.p11 = np11 + sa * dt2;
        self.p12 = np12;
        self.p22 = np22 + sigma_b * sigma_b * dt;
    }

    /// Fuse a direct position measurement `z` with variance `r` (H = [1,0,0]).
    fn update_pos(&mut self, z: f32, r: f32) {
        // v = P Hᵀ = column 0 of P; H P Hᵀ = p00.
        let v = [self.p00, self.p01, self.p02];
        self.update_scalar(z - self.pos, self.p00, r, v);
    }

    /// Fuse a direct velocity measurement `z` with variance `r` (H = [0,1,0]).
    fn update_vel(&mut self, z: f32, r: f32) {
        // v = P Hᵀ = column 1 of P; H P Hᵀ = p11.
        let v = [self.p01, self.p11, self.p12];
        self.update_scalar(z - self.vel, self.p11, r, v);
    }

    /// Shared scalar Kalman update. `innov = z − H x`, `hph = H P Hᵀ` (the measured
    /// state's variance), `v = P Hᵀ` (the selected covariance column), `r` the
    /// measurement variance.
    fn update_scalar(&mut self, innov: f32, hph: f32, r: f32, v: [f32; 3]) {
        let s = hph + r;
        if s <= 0.0 {
            return;
        }
        let k = [v[0] / s, v[1] / s, v[2] / s];
        self.pos += k[0] * innov;
        self.vel += k[1] * innov;
        self.bias += k[2] * innov;
        // P −= K vᵀ  (symmetric since K = v/S and v is a column of symmetric P).
        self.p00 -= k[0] * v[0];
        self.p01 -= k[0] * v[1];
        self.p02 -= k[0] * v[2];
        self.p11 -= k[1] * v[1];
        self.p12 -= k[1] * v[2];
        self.p22 -= k[2] * v[2];
    }
}

/// Fused navigation solution, world frame (X north, Y east, Z up).
#[derive(Clone, Copy, Default)]
pub struct NavSolution {
    /// Position relative to the GPS origin, metres [north, east, up].
    pub pos: [f32; 3],
    /// Velocity, m/s [north, east, up].
    pub vel: [f32; 3],
    /// True once a GPS origin is set and horizontal variance has converged.
    pub converged: bool,
    /// 1σ horizontal position uncertainty, metres (for the UI / health).
    pub pos_std: f32,
    /// Absolute position reconstructed from the origin (for GLOBAL_POSITION_INT).
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// MSL altitude (mm) = origin altitude + Up.
    pub alt_mm: i32,
    /// Altitude above launch (mm) = Up.
    pub rel_alt_mm: i32,
    /// Estimated world-frame accelerometer bias, m/s² [north, east, up].
    pub accel_bias: [f32; 3],
}

/// The navigation filter: three [`Axis1D`] (N, E, Up) + the GPS local-tangent
/// origin used to convert lat/lon ↔ metres.
pub struct Ekf {
    n: Axis1D,
    e: Axis1D,
    u: Axis1D,
    origin_set: bool,
    lat0: f32,
    lon0: f32,
    alt0: f32,
    cos_lat0: f32,
}

impl Ekf {
    pub const fn new() -> Self {
        Self {
            n: Axis1D::new(),
            e: Axis1D::new(),
            u: Axis1D::new(),
            origin_set: false,
            lat0: 0.0,
            lon0: 0.0,
            alt0: 0.0,
            cos_lat0: 1.0,
        }
    }

    /// Strapdown prediction. `accel_world` is the gravity-removed acceleration in
    /// the world frame (m/s², [north, east, up]); `dt` seconds.
    pub fn predict(&mut self, accel_world: [f32; 3], dt: f32) {
        self.n.predict(accel_world[0], dt, Q_ACCEL, Q_ACCEL_BIAS);
        self.e.predict(accel_world[1], dt, Q_ACCEL, Q_ACCEL_BIAS);
        self.u.predict(accel_world[2], dt, Q_ACCEL, Q_ACCEL_BIAS);
    }

    /// Set the local-tangent origin from the first good GPS fix. Subsequent
    /// positions are metres relative to this point.
    pub fn set_origin(&mut self, lat_deg: f32, lon_deg: f32, alt_m: f32) {
        self.lat0 = lat_deg;
        self.lon0 = lon_deg;
        self.alt0 = alt_m;
        self.cos_lat0 = libm::cosf(lat_deg * DEG2RAD);
        self.origin_set = true;
        // Anchor the state at the origin with modest uncertainty.
        self.n.pos = 0.0;
        self.e.pos = 0.0;
        self.u.pos = 0.0;
    }

    pub fn origin_set(&self) -> bool {
        self.origin_set
    }

    /// Convert a lat/lon (deg) to local north/east metres via the equirectangular
    /// (flat-earth) projection — accurate to centimetres over the kilometre
    /// scales a small UAV flies.
    pub fn gps_to_local(&self, lat_deg: f32, lon_deg: f32) -> (f32, f32) {
        let north = (lat_deg - self.lat0) * DEG2RAD * EARTH_RADIUS;
        let east = (lon_deg - self.lon0) * DEG2RAD * EARTH_RADIUS * self.cos_lat0;
        (north, east)
    }

    /// Inverse of [`gps_to_local`] — local metres back to lat/lon (deg).
    pub fn local_to_gps(&self, north: f32, east: f32) -> (f32, f32) {
        let lat = self.lat0 + (north / EARTH_RADIUS) * RAD2DEG;
        let lon = self.lon0 + (east / (EARTH_RADIUS * self.cos_lat0)) * RAD2DEG;
        (lat, lon)
    }

    pub fn origin_alt(&self) -> f32 {
        self.alt0
    }

    /// Fuse a GPS horizontal position fix (local metres). `hdop` scales the noise.
    pub fn fuse_gps_pos(&mut self, north: f32, east: f32, hdop: f32) {
        let sd = R_GPS_POS * hdop.max(1.0);
        let r = sd * sd;
        self.n.update_pos(north, r);
        self.e.update_pos(east, r);
    }

    /// Fuse a GPS horizontal velocity (north/east m/s).
    pub fn fuse_gps_vel(&mut self, vn: f32, ve: f32) {
        let r = R_GPS_VEL * R_GPS_VEL;
        self.n.update_vel(vn, r);
        self.e.update_vel(ve, r);
    }

    /// Fuse a barometric altitude (metres above the origin / launch).
    pub fn fuse_baro(&mut self, up_m: f32) {
        self.u.update_pos(up_m, R_BARO * R_BARO);
    }

    /// Fuse a lidar height-above-ground (metres). Precise, so it dominates Z when
    /// in range and the ground is level.
    pub fn fuse_lidar(&mut self, up_m: f32) {
        self.u.update_pos(up_m, R_LIDAR * R_LIDAR);
    }

    /// Fuse an optical-flow horizontal velocity (north/east m/s). `quality`
    /// (0..1) inflates the noise as the surface texture worsens.
    pub fn fuse_flow_vel(&mut self, vn: f32, ve: f32, quality: f32) {
        let q = quality.clamp(0.05, 1.0);
        let sd = R_FLOW / q;
        let r = sd * sd;
        self.n.update_vel(vn, r);
        self.e.update_vel(ve, r);
    }

    /// Current fused solution, including the absolute lat/lon reconstructed from
    /// the local-tangent origin (valid only once `converged`).
    pub fn solution(&self) -> NavSolution {
        let pos_var = self.n.p00.max(self.e.p00);
        let pos_std = libm::sqrtf(pos_var.max(0.0));
        let (lat, lon) = self.local_to_gps(self.n.pos, self.e.pos);
        NavSolution {
            pos: [self.n.pos, self.e.pos, self.u.pos],
            vel: [self.n.vel, self.e.vel, self.u.vel],
            // Converged once anchored and horizontal 1σ is within a few metres.
            converged: self.origin_set && pos_std < 5.0,
            pos_std,
            lat_e7: (lat as f64 * 1.0e7) as i32,
            lon_e7: (lon as f64 * 1.0e7) as i32,
            alt_mm: ((self.alt0 + self.u.pos) * 1000.0) as i32,
            rel_alt_mm: (self.u.pos * 1000.0) as i32,
            accel_bias: [self.n.bias, self.e.bias, self.u.bias],
        }
    }
}
