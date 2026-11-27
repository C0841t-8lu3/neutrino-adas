//! Exact geometric & coordinate conventions shared 1:1 with the QNX 8.0 C++ Safety Kernel.
//!
//! QNX World Frame:
//!   - +x: East (Right when heading == 0)
//!   - +y: Elevation (0 on flat roads)
//!   - +z: North (Forward when heading == 0)
//!   - heading (psi): radians clockwise from +Z (forward = (sin psi, cos psi), right = (cos psi, -sin psi))
//!   - signed curvature (kappa): + = clockwise / right turn, - = counter-clockwise / left turn

use std::f32::consts::{PI, TAU};

#[inline]
pub fn normalize_angle_rad(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= TAU;
    }
    while angle < -PI {
        angle += TAU;
    }
    angle
}

/// Forward unit vector `(fx, fz)` for QNX heading `psi` (rad).
#[inline]
pub fn forward_vec(heading: f32) -> (f32, f32) {
    (heading.sin(), heading.cos())
}

/// Right-hand unit vector `(rx, rz)` perpendicular to forward `(fx, fz)`.
#[inline]
pub fn right_vec(heading: f32) -> (f32, f32) {
    (heading.cos(), -heading.sin())
}

/// Project `(target_x, target_z)` into ego vehicle local `(fwd, lat)` meters.
/// Positive `fwd` is ahead; positive `lat` is to the right of ego.
/// Mirrors `ForwardRadar.cpp` and `BlindSpotMonitor.cpp` identically.
#[inline]
pub fn project_relative(
    ego_x: f32,
    ego_z: f32,
    ego_heading: f32,
    target_x: f32,
    target_z: f32,
) -> (f32, f32) {
    let dx = target_x - ego_x;
    let dz = target_z - ego_z;
    let sin_h = ego_heading.sin();
    let cos_h = ego_heading.cos();
    let fwd = dx * sin_h + dz * cos_h;
    let lat = dx * cos_h - dz * sin_h;
    (fwd, lat)
}

/// Lateral centerline curve offset at forward range `fwd`, matching `ForwardRadar.cpp`.
#[inline]
pub fn curved_corridor_offset(curvature: f32, fwd: f32) -> f32 {
    if curvature.abs() > 0.002 {
        (1.0 - (curvature * fwd).cos()) / curvature
    } else {
        0.5 * curvature * fwd * fwd
    }
}

/// Blind-spot zone check matching `BlindSpotMonitor.cpp` (`-12m..+4m`, `|lat| in [1.3, 4.8]`).
#[inline]
pub fn is_in_bsm_zone(fwd: f32, lat_rel: f32) -> (bool, bool) {
    if (-12.0..=4.0).contains(&fwd) {
        let left = (-4.8..=-1.3).contains(&lat_rel);
        let right = (1.3..=4.8).contains(&lat_rel);
        (left, right)
    } else {
        (false, false)
    }
}

/// Deterministic fast PRNG (SplitMix64 + Box-Muller Gaussian) for traffic/noise reproducibility.
#[derive(Debug, Clone)]
pub struct DeterministicRng {
    state: u64,
    spare_gauss: Option<f32>,
}

impl DeterministicRng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
            spare_gauss: None,
        }
    }

    pub fn reseed(&mut self, seed: u64) {
        self.state = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        self.spare_gauss = None;
    }

    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    #[inline]
    pub fn next_f32(&mut self) -> f32 {
        let bits = (self.next_u64() >> 40) as u32;
        (bits as f32) / ((1u32 << 24) as f32)
    }

    #[inline]
    pub fn gen_range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.next_f32() * (hi - lo)
    }

    #[inline]
    pub fn gen_usize(&mut self, bound: usize) -> usize {
        if bound == 0 {
            return 0;
        }
        (self.next_u64() as usize) % bound
    }

    #[inline]
    pub fn gauss(&mut self, mean: f32, sigma: f32) -> f32 {
        if let Some(z) = self.spare_gauss.take() {
            return mean + sigma * z;
        }
        let u1 = self.next_f32().max(1e-6);
        let u2 = self.next_f32() * TAU;
        let mag = (-2.0 * u1.ln()).sqrt();
        self.spare_gauss = Some(mag * u2.sin());
        mean + sigma * mag * u2.cos()
    }
}
