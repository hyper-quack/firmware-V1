//! Sensor-fusion front end: takes the two filtered IMU channels, brings them
//! into a common body frame, combines them, and drives the attitude filter.
//!
//! This is the analogue of PX4's `sensors` module (per-sensor rotation +
//! voting/combining) feeding the attitude estimator. See `docs/sensor-fusion.md`.

use crate::ahrs::{Attitude, Mahony};
use crate::imu::{Health, ImuOut};

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
}

impl Estimator {
    pub fn new(kp: f32, ki: f32, rot1: Rotation, rot2: Rotation) -> Self {
        Self {
            ahrs: Mahony::new(kp, ki),
            rot1,
            rot2,
        }
    }

    /// Run one fusion step over the latest filtered IMU outputs and return the
    /// updated attitude. `mag` is the body-frame magnetometer field (Gauss) when
    /// a healthy compass is present — it makes yaw absolute. `dt` is the step
    /// period in seconds.
    pub fn update(
        &mut self,
        imu1: &ImuOut,
        imu2: &ImuOut,
        mag: Option<[f32; 3]>,
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

        self.ahrs.update(gyro, accel, mag, dt);
        self.ahrs.attitude(gyro)
    }
}

#[inline]
fn avg(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    ]
}
