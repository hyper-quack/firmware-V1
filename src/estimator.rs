//! Sensor-fusion front end: takes the two filtered IMU channels, brings them
//! into a common body frame, combines them, and drives the attitude filter.
//!
//! This is the analogue of PX4's `sensors` module (per-sensor rotation +
//! voting/combining) feeding the attitude estimator. See `docs/sensor-fusion.md`.

use crate::ahrs::{rotate_body_to_world, Attitude, Mahony};
use crate::compass::MagCal;
use crate::imu::{Health, ImuOut};

const GRAVITY: f32 = 9.806_65; // m/s²
const DEG2RAD: f32 = core::f32::consts::PI / 180.0;

/// Mount-rotation correction. The hwdef declares IMU1 = `ROTATION_ROLL_180` and
/// IMU2 = `ROTATION_PITCH_180`; applying these aligns both sensors to the shared
/// body frame so they can be fused without fighting each other.
#[derive(Clone, Copy)]
pub enum Rotation {
    /// 180° about X: (x, -y, -z).
    Roll180,
    /// 180° about Y: (-x, y, -z).
    Pitch180,
}

impl Rotation {
    #[inline]
    pub fn apply(self, v: [f32; 3]) -> [f32; 3] {
        match self {
            Rotation::Roll180 => [v[0], -v[1], -v[2]],
            Rotation::Pitch180 => [-v[0], v[1], -v[2]],
        }
    }
}

pub struct Estimator {
    ahrs: Mahony,
    rot1: Rotation,
    rot2: Rotation,
    /// Compass calibration (mount rotation + hard/soft-iron), applied to the raw
    /// magnetometer before fusion.
    magcal: MagCal,
    /// Magnetic declination (deg, east-positive): converts the magnetic heading
    /// the compass measures into a true-north heading, so the EKF (true-north,
    /// from GPS) and the AHRS agree. Default 0 — set it for your location.
    declination_deg: f32,
    /// Gravity-removed acceleration in the **world** frame (true-north, Z up),
    /// from the last `update`. The EKF strapdown prediction consumes this.
    accel_world: [f32; 3],
}

impl Estimator {
    pub fn new(kp: f32, ki: f32, rot1: Rotation, rot2: Rotation) -> Self {
        Self {
            ahrs: Mahony::new(kp, ki),
            rot1,
            rot2,
            magcal: MagCal::identity(),
            declination_deg: 0.0,
            accel_world: [0.0; 3],
        }
    }

    /// Install a compass calibration.
    pub fn set_mag_cal(&mut self, cal: MagCal) {
        self.magcal = cal;
    }

    /// Set magnetic declination (deg, east-positive).
    pub fn set_declination(&mut self, deg: f32) {
        self.declination_deg = deg;
    }

    /// Mutable access to the compass calibration — e.g. to drive the online
    /// hard-iron collector (`start_collection` / `finish_collection`).
    pub fn mag_cal_mut(&mut self) -> &mut MagCal {
        &mut self.magcal
    }

    /// World-frame, gravity-removed acceleration (m/s²) from the most recent
    /// `update`, for the EKF prediction step.
    pub fn accel_world(&self) -> [f32; 3] {
        self.accel_world
    }

    /// Run one fusion step over the latest filtered IMU outputs and return the
    /// updated attitude. `mag` is the body-frame magnetometer field (Gauss) when
    /// a healthy compass is present — it makes yaw absolute. `dt` is the step
    /// period in seconds.
    pub fn update(
        &mut self,
        imu1: &ImuOut,
        imu2: &ImuOut,
        mag_raw: Option<[f32; 3]>,
        dt: f32,
    ) -> Attitude {
        let ok1 = matches!(imu1.health, Health::Ok(_));
        let ok2 = matches!(imu2.health, Health::Ok(_));

        // Rotate each sensor into the body frame.
        let g1 = self.rot1.apply(imu1.gyro);
        let a1 = self.rot1.apply(imu1.accel);
        let g2 = self.rot2.apply(imu2.gyro);
        let a2 = self.rot2.apply(imu2.accel);

        // Combine: average healthy sensors, fall back to whichever is alive.
        let (gyro, accel) = match (ok1, ok2) {
            (true, true) => (avg(g1, g2), avg(a1, a2)),
            (true, false) => (g1, a1),
            (false, true) => (g2, a2),
            (false, false) => ([0.0; 3], [0.0, 0.0, 1.0]), // hold level-ish
        };

        // Calibrate the magnetometer (mount rotation + hard/soft-iron) before
        // fusing it for heading.
        let mag = mag_raw.map(|m| self.magcal.apply(m));

        self.ahrs.update(gyro, accel, mag, dt);
        let mut a = self.ahrs.attitude(gyro);

        // Magnetic → true heading via declination.
        if self.declination_deg != 0.0 {
            a.yaw = wrap180(a.yaw + self.declination_deg);
        }

        // World-frame, gravity-removed acceleration for the EKF. Rotate the body
        // accel (in g) by the attitude quaternion, subtract gravity (world Z up),
        // then rotate the horizontal part by declination into the true-north frame
        // the GPS uses.
        let aw = rotate_body_to_world(
            a.q,
            [accel[0] * GRAVITY, accel[1] * GRAVITY, accel[2] * GRAVITY],
        );
        let g_removed = [aw[0], aw[1], aw[2] - GRAVITY];
        self.accel_world = if self.declination_deg != 0.0 {
            let r = self.declination_deg * DEG2RAD;
            let (s, c) = (libm::sinf(r), libm::cosf(r));
            [
                g_removed[0] * c - g_removed[1] * s,
                g_removed[0] * s + g_removed[1] * c,
                g_removed[2],
            ]
        } else {
            g_removed
        };

        a
    }
}

/// Wrap an angle (deg) into −180..180.
#[inline]
fn wrap180(mut deg: f32) -> f32 {
    while deg > 180.0 {
        deg -= 360.0;
    }
    while deg < -180.0 {
        deg += 360.0;
    }
    deg
}

#[inline]
fn avg(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ]
}
