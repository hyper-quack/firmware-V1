//! Attitude estimation: a quaternion **Mahony complementary filter** with gyro
//! bias estimation.
//!
//! This is the same class of estimator as PX4's `attitude_estimator_q` (a
//! nonlinear complementary filter on SO(3)): the gyro is integrated for the
//! high-frequency attitude, while the accelerometer's gravity direction
//! corrects low-frequency drift in roll/pitch. The correction is fed back both
//! proportionally (Kp) and through an integral term (Ki) that estimates and
//! removes gyro bias — the integral path is what makes a complementary filter
//! behave like a steady-state Kalman filter.
//!
//! **Yaw:** with no magnetometer fused (this board has no onboard compass), yaw
//! is *gyro-integrated only*. It is smooth and correct over short horizons but
//! has no absolute reference, so it slowly drifts. Roll and pitch are fully
//! observable from gravity and do not drift. See `docs/sensor-fusion.md`.

const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
const RAD2DEG: f32 = 180.0 / core::f32::consts::PI;

/// Fused attitude output, body frame.
#[derive(Clone, Copy)]
pub struct Attitude {
    /// Orientation quaternion `[w, x, y, z]`.
    pub q: [f32; 4],
    /// Euler angles in degrees.
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    /// Body angular rates in deg/s (filtered, bias-corrected).
    pub rates: [f32; 3],
    /// Estimated gyro bias in deg/s.
    pub bias: [f32; 3],
}

impl Default for Attitude {
    fn default() -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rates: [0.0; 3],
            bias: [0.0; 3],
        }
    }
}

/// Mahony complementary filter state + gains.
pub struct Mahony {
    q: [f32; 4],
    /// Integral error accumulator (estimated gyro bias, rad/s).
    integral_fb: [f32; 3],
    two_kp: f32,
    two_ki: f32,
    initialized: bool,
}

impl Mahony {
    /// `kp` drives how hard accel pulls roll/pitch toward gravity (responsiveness
    /// vs. noise immunity); `ki` sets gyro-bias learning rate.
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            integral_fb: [0.0; 3],
            two_kp: 2.0 * kp,
            two_ki: 2.0 * ki,
            initialized: false,
        }
    }

    /// One update step. `gyro_dps` in deg/s, `accel_g` in g (any consistent unit;
    /// it is normalized), `mag` (any consistent unit; normalized) optional, `dt`
    /// in seconds.
    ///
    /// With `mag = Some(..)` this is the full 9-DOF Mahony AHRS: the
    /// magnetometer corrects heading about the gravity axis, so yaw becomes
    /// **absolute** (no drift). With `mag = None` it degrades to the 6-DOF
    /// accel-only filter, where yaw is gyro-integrated and drifts.
    pub fn update(
        &mut self,
        gyro_dps: [f32; 3],
        accel_g: [f32; 3],
        mag: Option<[f32; 3]>,
        dt: f32,
    ) {
        let mut gx = gyro_dps[0] * DEG2RAD;
        let mut gy = gyro_dps[1] * DEG2RAD;
        let mut gz = gyro_dps[2] * DEG2RAD;

        let (ax, ay, az) = (accel_g[0], accel_g[1], accel_g[2]);
        let anorm = libm::sqrtf(ax * ax + ay * ay + az * az);

        // On the first valid accel reading, snap the quaternion to the measured
        // gravity so we don't wait for the filter to converge from identity.
        if !self.initialized && anorm > 0.5 {
            self.set_from_accel(ax / anorm, ay / anorm, az / anorm);
            self.initialized = true;
        }

        // Only apply accel correction when the accel reading is usable
        // (non-degenerate). This is the gating PX4 also does.
        if anorm > 1.0e-6 {
            let ax = ax / anorm;
            let ay = ay / anorm;
            let az = az / anorm;

            let [q0, q1, q2, q3] = self.q;
            // Gravity direction estimated from the current quaternion.
            let vx = 2.0 * (q1 * q3 - q0 * q2);
            let vy = 2.0 * (q0 * q1 + q2 * q3);
            let vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

            // Error = measured gravity x estimated gravity (roll/pitch).
            let mut ex = ay * vz - az * vy;
            let mut ey = az * vx - ax * vz;
            let mut ez = ax * vy - ay * vx;

            // Magnetometer correction (heading). Adds the cross-product error
            // between the measured field and the field direction predicted from
            // the current quaternion — the standard Mahony 9-DOF term.
            if let Some(m) = mag {
                let mnorm = libm::sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
                if mnorm > 1.0e-6 {
                    let mx = m[0] / mnorm;
                    let my = m[1] / mnorm;
                    let mz = m[2] / mnorm;

                    // Earth-frame field from the current attitude, then folded
                    // back to a horizontal reference (bx) + vertical (bz) so the
                    // correction only constrains heading, not tilt.
                    let hx = 2.0
                        * (mx * (0.5 - q2 * q2 - q3 * q3)
                            + my * (q1 * q2 - q0 * q3)
                            + mz * (q1 * q3 + q0 * q2));
                    let hy = 2.0
                        * (mx * (q1 * q2 + q0 * q3)
                            + my * (0.5 - q1 * q1 - q3 * q3)
                            + mz * (q2 * q3 - q0 * q1));
                    let bx = libm::sqrtf(hx * hx + hy * hy);
                    let bz = 2.0
                        * (mx * (q1 * q3 - q0 * q2)
                            + my * (q2 * q3 + q0 * q1)
                            + mz * (0.5 - q1 * q1 - q2 * q2));

                    // Predicted field direction in the body frame.
                    let wx = 2.0 * (bx * (0.5 - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
                    let wy = 2.0 * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3));
                    let wz = 2.0 * (bx * (q0 * q2 + q1 * q3) + bz * (0.5 - q1 * q1 - q2 * q2));

                    ex += my * wz - mz * wy;
                    ey += mz * wx - mx * wz;
                    ez += mx * wy - my * wx;
                }
            }

            // Integral feedback => gyro bias estimate.
            if self.two_ki > 0.0 {
                self.integral_fb[0] += self.two_ki * ex * dt;
                self.integral_fb[1] += self.two_ki * ey * dt;
                self.integral_fb[2] += self.two_ki * ez * dt;
                gx += self.integral_fb[0];
                gy += self.integral_fb[1];
                gz += self.integral_fb[2];
            }

            // Proportional feedback.
            gx += self.two_kp * ex;
            gy += self.two_kp * ey;
            gz += self.two_kp * ez;
        }

        // Integrate the quaternion: q_dot = 0.5 * q ⊗ (0, gx, gy, gz).
        let [q0, q1, q2, q3] = self.q;
        let half_dt = 0.5 * dt;
        let n0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
        let n1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_dt;
        let n2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_dt;
        let n3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_dt;

        // Normalize.
        let recip = inv_sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
        self.q = [n0 * recip, n1 * recip, n2 * recip, n3 * recip];
    }

    /// Build the current [`Attitude`] (Euler angles + bias) from filter state.
    /// `gyro_dps` is the raw (pre-bias-removal) gyro, used only to report the
    /// bias-corrected body rates.
    pub fn attitude(&self, gyro_dps: [f32; 3]) -> Attitude {
        let [q0, q1, q2, q3] = self.q;

        // ZYX (yaw-pitch-roll) Euler extraction.
        let roll = libm::atan2f(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
        let sinp = 2.0 * (q0 * q2 - q3 * q1);
        let pitch = if libm::fabsf(sinp) >= 1.0 {
            libm::copysignf(core::f32::consts::FRAC_PI_2, sinp) // gimbal lock
        } else {
            libm::asinf(sinp)
        };
        let yaw = libm::atan2f(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

        let bias = [
            self.integral_fb[0] * RAD2DEG,
            self.integral_fb[1] * RAD2DEG,
            self.integral_fb[2] * RAD2DEG,
        ];

        Attitude {
            q: self.q,
            roll: roll * RAD2DEG,
            pitch: pitch * RAD2DEG,
            yaw: yaw * RAD2DEG,
            rates: [
                gyro_dps[0] + bias[0],
                gyro_dps[1] + bias[1],
                gyro_dps[2] + bias[2],
            ],
            bias,
        }
    }

    /// Initialize the quaternion from a normalized gravity vector (roll/pitch
    /// only; yaw is left at 0).
    fn set_from_accel(&mut self, ax: f32, ay: f32, az: f32) {
        let roll = libm::atan2f(ay, az);
        let pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az));
        let (cr, sr) = (libm::cosf(roll * 0.5), libm::sinf(roll * 0.5));
        let (cp, sp) = (libm::cosf(pitch * 0.5), libm::sinf(pitch * 0.5));
        // yaw = 0 => cy = 1, sy = 0
        self.q = [cr * cp, sr * cp, cr * sp, -sr * sp];
    }
}

#[inline]
fn inv_sqrt(x: f32) -> f32 {
    if x > 0.0 {
        1.0 / libm::sqrtf(x)
    } else {
        0.0
    }
}

/// Rotate a body-frame vector into the world frame using attitude quaternion
/// `q = [w, x, y, z]` (the AHRS convention: world Z is *up*, gravity reads
/// `+1 g` on body Z when level). `world = R(q) · body`.
///
/// Used by the navigation EKF to turn body-frame accelerometer readings into a
/// world-frame acceleration for the strapdown prediction step.
pub fn rotate_body_to_world(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q;
    // Rotation matrix R (body -> world).
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - w * z);
    let r02 = 2.0 * (x * z + w * y);
    let r10 = 2.0 * (x * y + w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - w * x);
    let r20 = 2.0 * (x * z - w * y);
    let r21 = 2.0 * (y * z + w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}
