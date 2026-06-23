//! Flow + lidar navigation: height-above-ground from the MTF-01 lidar and a
//! horizontal velocity/position estimate from its optical flow.
//!
//! This is **dead-reckoning**, the same idea PX4/INAV use for an optical-flow
//! position hold: optical flow gives an *angular* rate of ground motion, which
//! becomes a *linear* velocity once scaled by height (`v = ω · h`). Integrating
//! that velocity gives a relative position. With no GPS correction it drifts over
//! time — fusing it against GPS/accel in an EKF is the next step; for now it is a
//! standalone estimate so the flow and lidar can be verified end to end.
//!
//! See [mtf01.rs](mtf01.rs) for the sensor driver.

use crate::ahrs::Attitude;
use crate::mtf01::Mtf01Data;

const DEG2RAD: f32 = core::f32::consts::PI / 180.0;

/// Raw MTF-01 optic-flow unit → rad/s. **Calibration constant** — the true value
/// must be measured on hardware (translate the craft a known distance at a known
/// height and match integrated position). This is a documented placeholder.
const FLOW_SCALE: f32 = 1.0e-4;

/// Reject flow below this MSP quality (0..255).
const FLOW_MIN_QUALITY: u8 = 30;
/// Valid lidar working range, metres.
const HEIGHT_MIN_M: f32 = 0.05;
const HEIGHT_MAX_M: f32 = 8.0;

/// Fused flow/lidar navigation state, earth frame (NED-ish: x north, y east).
#[derive(Clone, Copy, Default)]
pub struct NavState {
    /// Height above ground, metres (tilt-compensated lidar). Valid only when
    /// `height_valid`.
    pub height_m: f32,
    pub height_valid: bool,
    /// Horizontal velocity, m/s (earth frame).
    pub vx: f32,
    pub vy: f32,
    /// Integrated horizontal position, m (earth frame, relative to power-on).
    pub px: f32,
    pub py: f32,
    /// Last optical-flow quality seen (0..255).
    pub flow_quality: u8,
}

/// Flow/lidar dead-reckoning integrator.
#[derive(Default)]
pub struct Nav {
    state: NavState,
}

impl Nav {
    pub const fn new() -> Self {
        Self {
            state: NavState {
                height_m: 0.0,
                height_valid: false,
                vx: 0.0,
                vy: 0.0,
                px: 0.0,
                py: 0.0,
                flow_quality: 0,
            },
        }
    }

    pub fn state(&self) -> NavState {
        self.state
    }

    /// One navigation step. `att` supplies tilt (for height), heading (for the
    /// body→earth rotation) and body rates (to de-rotate the flow). `dt` seconds.
    pub fn update(&mut self, sensor: &Mtf01Data, att: &Attitude, dt: f32) {
        // --- Height: project the lidar slant range onto the vertical axis. ---
        let roll = att.roll * DEG2RAD;
        let pitch = att.pitch * DEG2RAD;
        let tilt_cos = libm::cosf(roll) * libm::cosf(pitch);
        if sensor.dist_valid {
            let h = (sensor.dist_mm as f32 / 1000.0) * tilt_cos;
            if (HEIGHT_MIN_M..=HEIGHT_MAX_M).contains(&h) {
                self.state.height_m = h;
                self.state.height_valid = true;
            } else {
                self.state.height_valid = false;
            }
        } else {
            self.state.height_valid = false;
        }

        self.state.flow_quality = sensor.flow_quality;

        // --- Horizontal velocity from optical flow. ---
        // Need a valid height to convert angular flow to linear velocity.
        if sensor.flow_valid
            && sensor.flow_quality >= FLOW_MIN_QUALITY
            && self.state.height_valid
        {
            // Angular ground-motion rate (rad/s) reported by the sensor.
            let fx = sensor.flow_x as f32 * FLOW_SCALE;
            let fy = sensor.flow_y as f32 * FLOW_SCALE;

            // De-rotate: body pitch/roll rates induce apparent flow that is not
            // translation. att.rates are bias-corrected body rates in deg/s.
            // (Axis pairing is sensor-mount dependent — verify signs on hardware.)
            let wx = att.rates[0] * DEG2RAD; // roll rate
            let wy = att.rates[1] * DEG2RAD; // pitch rate
            let flow_x = fx - wy;
            let flow_y = fy + wx;

            // Linear body velocity: v = ω · h.
            let h = self.state.height_m;
            let vbx = flow_x * h;
            let vby = flow_y * h;

            // Rotate body velocity into the earth frame by heading.
            let yaw = att.yaw * DEG2RAD;
            let (s, c) = (libm::sinf(yaw), libm::cosf(yaw));
            self.state.vx = vbx * c - vby * s;
            self.state.vy = vbx * s + vby * c;

            // Integrate to a relative position.
            self.state.px += self.state.vx * dt;
            self.state.py += self.state.vy * dt;
        } else {
            // No usable flow — hold position, zero the velocity estimate.
            self.state.vx = 0.0;
            self.state.vy = 0.0;
        }
    }
}
