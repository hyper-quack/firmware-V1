//! Digital pre-filters for the IMU signal chain.
//!
//! These mirror the building blocks PX4 uses in its sensor pipeline
//! (`mathlib/math/filter`): a 2nd-order Butterworth low-pass and a 2nd-order
//! biquad notch. They run on every raw sample *before* fusion, exactly like
//! PX4's `SensorGyro`/`SensorAccel` filtering stages.
//!
//! See `docs/sensor-fusion.md` for the math and where each filter sits.

use core::f32::consts::PI;

/// 2nd-order Butterworth low-pass — byte-for-byte the algorithm in PX4's
/// `LowPassFilter2p`. Direct-form-I with two delay elements.
#[derive(Clone, Copy)]
pub struct LowPassFilter2p {
    a1: f32,
    a2: f32,
    b0: f32,
    b1: f32,
    b2: f32,
    d1: f32,
    d2: f32,
    enabled: bool,
}

impl LowPassFilter2p {
    /// `cutoff_freq <= 0` (or >= Nyquist) disables filtering (pass-through).
    pub fn new(sample_freq: f32, cutoff_freq: f32) -> Self {
        let mut f = Self {
            a1: 0.0,
            a2: 0.0,
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            d1: 0.0,
            d2: 0.0,
            enabled: false,
        };
        f.set_cutoff(sample_freq, cutoff_freq);
        f
    }

    pub fn set_cutoff(&mut self, sample_freq: f32, cutoff_freq: f32) {
        if cutoff_freq <= 0.0 || cutoff_freq >= sample_freq * 0.5 {
            // Disabled: pass-through.
            self.enabled = false;
            self.b0 = 1.0;
            self.b1 = 0.0;
            self.b2 = 0.0;
            self.a1 = 0.0;
            self.a2 = 0.0;
            return;
        }
        let fr = sample_freq / cutoff_freq;
        let ohm = libm::tanf(PI / fr);
        let c = 1.0 + 2.0 * libm::cosf(PI / 4.0) * ohm + ohm * ohm;
        self.b0 = ohm * ohm / c;
        self.b1 = 2.0 * self.b0;
        self.b2 = self.b0;
        self.a1 = 2.0 * (ohm * ohm - 1.0) / c;
        self.a2 = (1.0 - 2.0 * libm::cosf(PI / 4.0) * ohm + ohm * ohm) / c;
        self.enabled = true;
    }

    pub fn apply(&mut self, sample: f32) -> f32 {
        if !self.enabled {
            return sample;
        }
        let d0 = sample - self.d1 * self.a1 - self.d2 * self.a2;
        let output = d0 * self.b0 + self.d1 * self.b1 + self.d2 * self.b2;
        self.d2 = self.d1;
        self.d1 = d0;
        output
    }

    /// Pre-load the filter state so it starts at `sample` instead of zero
    /// (avoids a startup transient). Provided for callers that reset filters on
    /// arming; not used by the current always-on pipeline.
    #[allow(dead_code)]
    pub fn reset(&mut self, sample: f32) {
        // Steady-state delay value for a constant input.
        let dval = sample / (1.0 + self.a1 + self.a2);
        self.d1 = dval;
        self.d2 = dval;
    }
}

/// 2nd-order biquad notch (transposed direct-form II), matching PX4's
/// `NotchFilter`. Used to reject a narrow band (e.g. prop/motor vibration).
///
/// Not wired into the live pipeline yet — it needs a noise frequency to target
/// (normally driven by ESC RPM / a dynamic-notch tracker), which this firmware
/// doesn't have until motor telemetry exists. Provided + documented so the slot
/// is ready. See `docs/sensor-fusion.md`.
#[allow(dead_code)] // wired in once a dynamic-notch / ESC-RPM source exists
#[derive(Clone, Copy)]
pub struct NotchFilter {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
    enabled: bool,
}

#[allow(dead_code)] // see note on the struct above
impl NotchFilter {
    pub fn new(sample_freq: f32, notch_freq: f32, bandwidth: f32) -> Self {
        let mut f = Self {
            b0: 1.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            z1: 0.0,
            z2: 0.0,
            enabled: false,
        };
        f.set(sample_freq, notch_freq, bandwidth);
        f
    }

    pub fn set(&mut self, sample_freq: f32, notch_freq: f32, bandwidth: f32) {
        if notch_freq <= 0.0 || bandwidth <= 0.0 || notch_freq >= sample_freq * 0.5 {
            self.enabled = false;
            self.b0 = 1.0;
            self.b1 = 0.0;
            self.b2 = 0.0;
            self.a1 = 0.0;
            self.a2 = 0.0;
            return;
        }
        let alpha = libm::tanf(PI * bandwidth / sample_freq);
        let beta = -libm::cosf(2.0 * PI * notch_freq / sample_freq);
        let a0_inv = 1.0 / (alpha + 1.0);
        self.b0 = a0_inv;
        self.b1 = 2.0 * beta * a0_inv;
        self.b2 = a0_inv;
        self.a1 = self.b1;
        self.a2 = (1.0 - alpha) * a0_inv;
        self.enabled = true;
    }

    pub fn apply(&mut self, x: f32) -> f32 {
        if !self.enabled {
            return x;
        }
        let output = x * self.b0 + self.z1;
        self.z1 = x * self.b1 - output * self.a1 + self.z2;
        self.z2 = x * self.b2 - output * self.a2;
        output
    }
}

/// Per-IMU low-pass bank: one filter per gyro axis and per accel axis.
/// Gyro and accel get separate cutoffs (accel is low-passed harder since the
/// gravity reference we fuse is essentially DC).
#[derive(Clone, Copy)]
pub struct ImuLpf {
    gyro: [LowPassFilter2p; 3],
    accel: [LowPassFilter2p; 3],
}

impl ImuLpf {
    pub fn new(sample_freq: f32, gyro_cutoff: f32, accel_cutoff: f32) -> Self {
        Self {
            gyro: [LowPassFilter2p::new(sample_freq, gyro_cutoff); 3],
            accel: [LowPassFilter2p::new(sample_freq, accel_cutoff); 3],
        }
    }

    /// Filter one sample (gyro in dps, accel in g). Returns the filtered pair.
    pub fn apply(&mut self, gyro_dps: [f32; 3], accel_g: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        let mut g = [0.0f32; 3];
        let mut a = [0.0f32; 3];
        for i in 0..3 {
            g[i] = self.gyro[i].apply(gyro_dps[i]);
            a[i] = self.accel[i].apply(accel_g[i]);
        }
        (g, a)
    }
}
