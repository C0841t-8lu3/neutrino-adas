//! Pure-Rust interactive navigation map viewport math (world <-> screen, pan, zoom,
//! and Point A -> Point B click snapping) consumed by the `bevy_egui` map UI & tests.

use crate::road_network::{PlannedRoute, RoadNetwork, RoadSnap};
use crate::simulation::SimWorld;

#[derive(Debug, Clone)]
pub struct MapViewport {
    pub center_x: f32,
    pub center_z: f32,
    pub zoom_m_per_px: f32,
    pub screen_w: f32,
    pub screen_h: f32,
    pub dragging: bool,
    pub drag_last_mouse: (f32, f32),
    pub drag_moved_px: f32,
    pub preview: Option<PlannedRoute>,
    pub snap_hint: Option<RoadSnap>,
    pub last_click_time_s: f64,
    pub last_click_pos: (f32, f32),
}

impl MapViewport {
    pub fn fit_network(net: &RoadNetwork, screen_w: f32, screen_h: f32) -> Self {
        let (min_x, max_x, min_z, max_z) = net.bounds;
        let span_x = (max_x - min_x + 120.0).max(120.0);
        let span_z = (max_z - min_z + 140.0).max(120.0);
        let zoom_x = span_x / (screen_w - 80.0).max(200.0);
        let zoom_z = span_z / (screen_h - 140.0).max(200.0);
        Self {
            center_x: 0.5 * (min_x + max_x),
            center_z: 0.5 * (min_z + max_z),
            zoom_m_per_px: zoom_x.max(zoom_z).clamp(0.08, 8.0),
            screen_w: screen_w.max(320.0),
            screen_h: screen_h.max(240.0),
            dragging: false,
            drag_last_mouse: (0.0, 0.0),
            drag_moved_px: 0.0,
            preview: None,
            snap_hint: None,
            last_click_time_s: -10.0,
            last_click_pos: (0.0, 0.0),
        }
    }

    pub fn update_screen_size(&mut self, w: f32, h: f32) {
        self.screen_w = w.max(320.0);
        self.screen_h = h.max(240.0);
    }

    #[inline]
    pub fn world_to_screen(&self, wx: f32, wz: f32) -> (f32, f32) {
        let sx = 0.5 * self.screen_w + (wx - self.center_x) / self.zoom_m_per_px;
        let sy = 0.5 * self.screen_h - (wz - self.center_z) / self.zoom_m_per_px;
        (sx, sy)
    }

    #[inline]
    pub fn screen_to_world(&self, sx: f32, sy: f32) -> (f32, f32) {
        let wx = self.center_x + (sx - 0.5 * self.screen_w) * self.zoom_m_per_px;
        let wz = self.center_z - (sy - 0.5 * self.screen_h) * self.zoom_m_per_px;
        (wx, wz)
    }

    pub fn zoom_at(&mut self, sx: f32, sy: f32, factor: f32) {
        let (wx_before, wz_before) = self.screen_to_world(sx, sy);
        self.zoom_m_per_px = (self.zoom_m_per_px * factor).clamp(0.06, 8.0);
        self.center_x = wx_before - (sx - 0.5 * self.screen_w) * self.zoom_m_per_px;
        self.center_z = wz_before + (sy - 0.5 * self.screen_h) * self.zoom_m_per_px;
    }

    pub fn pan_by_pixels(&mut self, dx: f32, dy: f32) {
        self.center_x -= dx * self.zoom_m_per_px;
        self.center_z += dy * self.zoom_m_per_px;
    }

    pub fn preview_click(&mut self, world: &SimWorld, sx: f32, sy: f32) -> Option<PlannedRoute> {
        let (wx, wz) = self.screen_to_world(sx, sy);
        let snap = world.net.find_snap(wx, wz);
        self.snap_hint = snap;
        let route = world.preview_point_to_point(wx, wz);
        self.preview = route.clone();
        route
    }
}
