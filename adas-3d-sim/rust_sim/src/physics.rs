//! Dynamic bicycle vehicle dynamics model with wheelbase L = 2.75m,
//! 80 ms first-order steering actuator lag, steering rate limits,
//! aerodynamic/rolling drag, and DRY (0.95) vs WET (0.55) tire friction grip.

use crate::config::WHEELBASE_M;
use crate::coordinate_mapper::normalize_angle_rad;

#[derive(Debug, Clone)]
pub struct VehiclePhysics {
    pub x: f32,
    pub z: f32,
    pub heading: f32,
    pub speed: f32,
    pub steer_angle: f32, // Actual front wheel steering angle (rad, +=right, -=left)
    pub yaw_rate: f32,
    pub accel_lon: f32,
    pub wheelbase: f32,
}

impl VehiclePhysics {
    pub fn new(x: f32, z: f32, heading: f32) -> Self {
        Self {
            x,
            z,
            heading: normalize_angle_rad(heading),
            speed: 0.0,
            steer_angle: 0.0,
            yaw_rate: 0.0,
            accel_lon: 0.0,
            wheelbase: WHEELBASE_M,
        }
    }

    pub fn reset(&mut self, x: f32, z: f32, heading: f32) {
        self.x = x;
        self.z = z;
        self.heading = normalize_angle_rad(heading);
        self.speed = 0.0;
        self.steer_angle = 0.0;
        self.yaw_rate = 0.0;
        self.accel_lon = 0.0;
    }

    /// Advance dynamic bicycle model by `dt` seconds under surface friction `mu`.
    /// Returns forward distance traveled `ds` in this step.
    pub fn step(
        &mut self,
        steer_cmd_rad: f32,
        throttle: f32,
        brake: f32,
        mu: f32,
        dt: f32,
    ) -> f32 {
        let dt_safe = dt.clamp(0.001, 0.05);
        let eff_mu = mu.clamp(0.25, 1.2);
        let max_tire_accel = eff_mu * 9.81;

        // 1. Steering actuator dynamics: 80 ms first-order lag + 2.4 rad/s mechanical rate limit
        let target_steer = steer_cmd_rad.clamp(-0.65, 0.65);
        let tau = 0.080f32;
        let desired_rate = (target_steer - self.steer_angle) / tau;
        let max_rate = 2.4f32;
        let ds_wheel = desired_rate.clamp(-max_rate, max_rate) * dt_safe;
        self.steer_angle = (self.steer_angle + ds_wheel).clamp(-0.65, 0.65);

        // 2. Longitudinal drive / brake / drag forces capped by tire friction circle
        let thr = throttle.clamp(0.0, 1.0);
        let brk = brake.clamp(0.0, 1.0);

        let drive_accel = thr * 3.8;
        let brake_decel = brk * 9.0;
        let drag_decel = if self.speed > 0.02 {
            0.014 * 9.81 + 0.00035 * self.speed * self.speed
        } else {
            0.0
        };

        let raw_accel = drive_accel - brake_decel - drag_decel;
        let net_accel = raw_accel.clamp(-max_tire_accel, max_tire_accel * 0.85);
        self.accel_lon = net_accel;

        let new_speed = if self.speed <= 0.05 && thr <= 0.01 && brk > 0.1 {
            0.0
        } else {
            (self.speed + net_accel * dt_safe).max(0.0)
        };
        let v_avg = 0.5 * (self.speed + new_speed);
        self.speed = new_speed;

        // 3. Bicycle yaw kinematics with wet/dry cornering traction ceiling
        let yaw_ideal = (v_avg / self.wheelbase) * self.steer_angle.tan();
        let a_lat_req = (v_avg * yaw_ideal).abs();
        let a_lat_cap = max_tire_accel * 0.94;
        let yaw_rate = if a_lat_req > a_lat_cap && a_lat_req > 1e-4 {
            yaw_ideal * (a_lat_cap / a_lat_req)
        } else {
            yaw_ideal
        };
        self.yaw_rate = yaw_rate;

        // 4. Midpoint pose integration for zero radial drift around curves
        let dpsi = yaw_rate * dt_safe;
        let h_mid = normalize_angle_rad(self.heading + 0.5 * dpsi);
        let ds = v_avg * dt_safe;
        self.x += ds * h_mid.sin();
        self.z += ds * h_mid.cos();
        self.heading = normalize_angle_rad(self.heading + dpsi);

        ds
    }
}
