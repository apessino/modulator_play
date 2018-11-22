//!
//! # 2d Cubic Bezier curves using Shemanarev metrics
//!
//! Eventually this will be extended with curve split/join operations, maybe made generic in
//! the sample type (even if it seems overkill), and then moved into its own crate. For now
//! just a module that lives here in the modulator testbed app
//!
//! Copyright© 2018 Ready At Dawn Studios

use std::f32;
use std::f32::consts::*;

use cgmath::prelude::*;
use cgmath::{vec2, Vector2};

type Vec2 = Vector2<f32>;

/// Maximum precision normalization or 0-vector
pub fn vec2_norm_or_zero(v: Vec2) -> Vec2 {
    let sz = 1.0 / v[0].hypot(v[1]);
    if sz.is_finite() {
        v * sz
    } else {
        vec2(0.0, 0.0)
    }
}

/// Test if the given bounds enclose the point `v`
pub fn vec2_in_bounds(v: Vec2, bnds: &[Vec2; 2]) -> bool {
    v.x >= bnds[0].x && v.x <= bnds[1].x && v.y >= bnds[0].y && v.y <= bnds[1].y
}

/// Calculate and return the closest point on line segment `l0..l1` to point `p`.
/// Returns the point and its weight
pub fn vec2_closest_on_line(p: Vec2, l0: Vec2, l1: Vec2) -> (Vec2, f32) {
    let ln = l1 - l0;
    let t = (p - l0).dot(ln);

    if t <= 0.0 {
        return (l0, 0.0);
    }

    let den = ln.magnitude2();
    if t >= den {
        return (l1, 1.0);
    }

    let t = t / den;
    (l0 + ln * t, t)
}

/// Extended data stored per sample
#[derive(Copy, Clone, Debug)]
pub struct SampleData {
    pub t: f32,  // normalized sample time along the curve
    pub l: f32,  // linear distance to the next sample
    pub n: Vec2, // normal to the curve at the sample
}

impl SampleData {
    pub fn new(t: f32) -> Self {
        Self {
            t,
            l: 0.0,
            n: vec2(0.0, 1.0),
        }
    }
}

/// A 2d Bezier-style cubic curve using Shemanarev metrics for termination of recursion
#[derive(Clone)]
pub struct CubicCurve {
    controls: [Vec2; 4], // curve control points
    dc: [Vec2; 3],       // controls derivative coefficients

    approx_scale: f32,    // screen space approx. scale (increases with accuracy)
    angle_tolerance: f32, // the smaller, the smoother the turns, 0 to disable

    smp: Vec<Vec2>,       // cached curve samples
    smd: Vec<SampleData>, // symmetrical cached sample data

    tolerance_squared: f32, // internal state for recursion
    total_length: f32,      // cached total length of the curve, from samples
    bounds: [Vec2; 2],      // bounding box (as a l,h quad) (around samples only)

    cached: bool, // state of the sample cache, all data above depends on this
}

const MAX_RECURSION: u32 = 32;
const ANGLE_EPSILON: f32 = 0.01;

impl CubicCurve {
    /// Make a new cubic curve
    pub fn new() -> Self {
        let approx_scale = 1.0;
        CubicCurve {
            controls: [vec2(0.0, 0.0); 4],
            dc: [vec2(0.0, 0.0); 3],
            approx_scale,
            angle_tolerance: 15.0_f32.to_radians(),
            smp: vec![],
            smd: vec![],
            tolerance_squared: 0.25 / (approx_scale * approx_scale),
            total_length: 0.0,
            bounds: [vec2(0.0, 0.0); 2],
            cached: true,
        }
    }

    /// Access the cubic curve control points
    pub fn control_points(&self) -> &[Vec2; 4] {
        &self.controls
    }
    /// Set the cubic curve control points
    pub fn set_control_points(&mut self, controls: &[Vec2; 4]) {
        self.controls = *controls;

        self.dc[0] = (self.controls[1] - self.controls[0]) * 3.0;
        self.dc[1] = (self.controls[2] - self.controls[1]) * 3.0;
        self.dc[2] = (self.controls[3] - self.controls[2]) * 3.0;

        self.invalidate();
    }

    /// Screen space approximation scale (increases with accuracy)
    pub fn approx_scale(&self) -> f32 {
        self.approx_scale
    }
    /// Set the screen space approximation scale (increases with accuracy)
    pub fn set_approx_scale(&mut self, scale: f32) {
        self.approx_scale = f32::max(scale, 0.0001);
        self.tolerance_squared = 0.25 / (self.approx_scale * self.approx_scale);

        self.invalidate();
    }

    /// Angle tolerance - the smaller it is, the smoother the turns, 0 to disable
    pub fn angle_tolerance(&self) -> f32 {
        self.angle_tolerance
    }
    /// Set angle tolerance - the smaller it is, the smoother the turns, 0 to disable
    pub fn set_angle_tolerance(&mut self, scale: f32) {
        self.angle_tolerance = scale;
        self.invalidate();
    }

    /// Borrow the cached curve samples
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn samples(&self) -> &Vec<Vec2> {
        assert!(self.valid());
        &self.smp
    }
    /// Borrow the cached curve additional sample data
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn sampledata(&self) -> &Vec<SampleData> {
        assert!(self.valid());
        &self.smd
    }

    /// Total linear length of the curve, same magnitude of error as the samples
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn length(&self) -> f32 {
        assert!(self.valid());
        self.total_length
    }
    /// AABB of sample points (no control points), extruded by some value (pass 0.0 for exact)
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn bounds(&self, extrude: f32) -> [Vec2; 2] {
        assert!(self.valid());
        [
            vec2(self.bounds[0].x - extrude, self.bounds[0].y - extrude),
            vec2(self.bounds[1].x + extrude, self.bounds[1].y + extrude),
        ]
    }
    /// Test if the given point is inside the bounds of the curve, with optional extrusion
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn in_bounds(&self, pt: Vec2, extrude: f32) -> bool {
        let bn = self.bounds(extrude);
        pt.x >= bn[0].x && pt.x <= bn[1].x && pt.y >= bn[0].y && pt.y <= bn[1].y
    }

    /// Point on curve at normalized time t (exact, not using samples)
    pub fn curve_at(&self, t: f32) -> Vec2 {
        let t2 = t * t;
        let t3 = t * t * t;

        let mt = 1.0 - t;
        let mt2 = mt * mt;
        let mt3 = mt * mt * mt;

        let c1 = 3.0 * mt2 * t;
        let c2 = 3.0 * mt * t2;

        vec2(
            mt3 * self.controls[0].x
                + c1 * self.controls[1].x
                + c2 * self.controls[2].x
                + t3 * self.controls[3].x,
            mt3 * self.controls[0].y
                + c1 * self.controls[1].y
                + c2 * self.controls[2].y
                + t3 * self.controls[3].y,
        )
    }

    /// Tangent to curve at curve time t (exact, not using samples)
    pub fn tangent_at(&self, t: f32) -> Vec2 {
        let t2 = t * t;

        let mt = 1.0 - t;
        let mt2 = mt * mt;

        let c1 = 2.0 * mt * t;

        vec2_norm_or_zero(vec2(
            mt2 * self.dc[0].x + c1 * self.dc[1].x + t2 * self.dc[2].x,
            mt2 * self.dc[0].y + c1 * self.dc[1].y + t2 * self.dc[2].y,
        ))
    }
    /// Normal to curve at curve time t (exact, not using samples)
    pub fn normal_at(&self, t: f32) -> Vec2 {
        self.normal_at_(self.tangent_at(t))
    }
    /// Get tangent if you have the normal (exact, not using samples)
    pub fn tangent_at_(&self, n: Vec2) -> Vec2 {
        vec2(n.y, -n.x)
    }
    /// Get normal if you have the tangent (exact, not using samples)
    pub fn normal_at_(&self, t: Vec2) -> Vec2 {
        vec2(-t.y, t.x)
    }

    /// Closest point on curve to `pt` (linear approximation, from samples), returns
    /// `(p, t = curve time [0..1], d = dist from pt to p, k = index of sample containing t)`
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn closest_to(&self, pt: Vec2) -> (Option<usize>, Vec2, f32, f32) {
        assert!(self.valid());

        let mut p = vec2(f32::INFINITY, f32::INFINITY);
        let mut t = f32::INFINITY;
        let mut d = f32::INFINITY;
        let mut k = None;

        let n = self.smp.len();
        let mut i = 0;
        for l0 in &self.smp {
            let not_last = i < n - 1;

            let l1 = if not_last { &self.smp[i + 1] } else { l0 };
            let (p_, t_) = vec2_closest_on_line(pt, *l0, *l1);

            let d_ = (pt - p_).magnitude2();
            if d_ < d {
                p = p_;
                t = if not_last {
                    self.smd[i].t + (self.smd[i + 1].t - self.smd[i].t) * t_
                } else {
                    self.smd[i].t
                };
                d = d_;
                k = Some(i);
            }
            i += 1;
        }

        (k, p, t, d.sqrt())
    }

    /// Closest sample to `pt`, returns (closest sample index, its distance to pt)
    ///
    /// # Panics
    ///
    /// Curve must be `valid` or this method will panic
    pub fn closest_sample_to(&self, pt: Vec2) -> (Option<usize>, f32) {
        assert!(self.valid());

        let mut d = f32::INFINITY;
        let mut k = None;

        let mut i = 0;
        for sp in &self.smp {
            let d_ = (pt - *sp).magnitude2();
            if d_ < d {
                d = d_;
                k = Some(i);
            }
            i += 1;
        }

        (k, d.sqrt())
    }

    /// Closest handle point to `pt` (a handle is the line between a pair of control points,
    /// from the point on curve to the one off curve), returns (handle 0 = c0..c1 1 = c3..c2,
    /// point on handle, time along handle of closest point [0.0..1.0], distance to pt)
    pub fn closest_handle_to(&self, pt: Vec2) -> (usize, Vec2, f32, f32) {
        let (p0, t0) = vec2_closest_on_line(pt, self.controls[0], self.controls[1]);
        let (p1, t1) = vec2_closest_on_line(pt, self.controls[3], self.controls[2]);

        let d0 = (pt - p0).magnitude2();
        let d1 = (pt - p1).magnitude2();

        if d1 < d0 {
            (1, p1, t1, d1.sqrt())
        } else {
            (0, p0, t0, d0.sqrt())
        }
    }

    /// Closest control point to pt, returns (control point index [0..3], distance to pt)
    pub fn closest_control_point_to(&self, pt: Vec2) -> (usize, f32) {
        let mut d = f32::INFINITY;
        let mut k = 0;

        for i in 0..4 {
            let d_ = (pt - self.controls[i]).magnitude2();
            if d_ < d {
                d = d_;
                k = i;
            }
        }

        (k, d.sqrt())
    }

    /// If true the cached curve data is valid
    pub fn valid(&self) -> bool {
        self.cached
    }
    /// Force the cached curve data to be recomputed
    pub fn invalidate(&mut self) {
        self.cached = false;
    }

    /// Validate the cached curved data - MUST be called to ensure the curve is valid before
    /// accessing any of the curve caches
    pub fn validate(&mut self) {
        if self.valid() == false {
            self.smp.clear();
            self.smd.clear();

            let t_ = self.initial_times(); // initial curve times
            let c_ = self.controls;

            self.smp.push(c_[0]); // c0 is always the first sample
            self.smd.push(SampleData::new(t_[0]));

            self.bezier(c_, t_, 0);

            self.smp.push(c_[3]); // c3 is always the last
            self.smd.push(SampleData::new(t_[3]));

            self.cache_sample_data();
            self.cached = true;
        }
    }

    /// Compute the initial curve times for the recursion
    fn initial_times(&self) -> [f32; 4] {
        let c01 = self.controls[1] - self.controls[0];
        let c12 = self.controls[2] - self.controls[1];
        let c23 = self.controls[3] - self.controls[2];

        let l01 = c01.magnitude();
        let l12 = c12.magnitude();
        let l23 = c23.magnitude();

        let l = l01 + l12 + l23;

        let t1 = l01 / l;
        let t2 = l12 / l;

        [
            0.0,
            if t1.is_finite() { t1 } else { 0.0 },
            if t2.is_finite() { t2 } else { 0.0 },
            1.0,
        ]
    }

    /// Recursive bezier evaluation using Shemanarev metrics
    fn bezier(&mut self, c_: [Vec2; 4], t_: [f32; 4], level: u32) {
        #![allow(non_snake_case)]

        if level > MAX_RECURSION {
            return;
        }

        let c__01 = (c_[0] + c_[1]) * 0.5;
        let c__12 = (c_[1] + c_[2]) * 0.5;
        let c__23 = (c_[2] + c_[3]) * 0.5;
        let c_012 = (c__01 + c__12) * 0.5;
        let c_123 = (c__12 + c__23) * 0.5;
        let c0123 = (c_012 + c_123) * 0.5;

        let t__01 = (t_[0] + t_[1]) * 0.5;
        let t__12 = (t_[1] + t_[2]) * 0.5;
        let t__23 = (t_[2] + t_[3]) * 0.5;
        let t_012 = (t__01 + t__12) * 0.5;
        let t_123 = (t__12 + t__23) * 0.5;
        let t0123 = (t_012 + t_123) * 0.5;

        let d = vec2(c_[3][0] - c_[0][0], c_[3][1] - c_[0][1]);
        let dsq = d.magnitude2() * self.tolerance_squared;

        let d2 = ((c_[1].x - c_[3].x) * d.y - (c_[1].y - c_[3].y) * d.x).abs();
        let d3 = ((c_[2].x - c_[3].x) * d.y - (c_[2].y - c_[3].y) * d.x).abs();

        let has_d2 = d2 > f32::EPSILON;
        let has_d3 = d3 > f32::EPSILON;

        if has_d3 == false && has_d2 == false {
            // all collinear, c0 == c3
            if self.collinear(c_, t_, d) == false {
                return;
            }
        } else if has_d3 == false && has_d2 {
            // c0, c1 and c3 collinear, c2 significant
            if d3 * d3 <= dsq && self.onlyc2c1(c_, t_, true) == false {
                return;
            }
        } else if has_d3 && has_d2 == false {
            // c0, c2 and c3 collinear, c1 significant
            if d2 * d2 <= dsq && self.onlyc2c1(c_, t_, false) == false {
                return;
            }
        } else if (d2 + d3) * (d2 + d3) <= dsq && self.curve(c_, t_) == false {
            return;
        }

        // recurse down each side of the curve
        self.bezier(
            [c_[0], c__01, c_012, c0123],
            [t_[0], t__01, t_012, t0123],
            level + 1,
        );
        self.bezier(
            [c0123, c_123, c__23, c_[3]],
            [t0123, t_123, t__23, t_[3]],
            level + 1,
        );
    }

    /// Local curve coefficients are collinear
    fn collinear(&mut self, c_: [Vec2; 4], t_: [f32; 4], d: Vec2) -> bool {
        let k = 1.0 / d.magnitude2();

        let mut d2;
        let mut d3;

        if k.is_finite() == false {
            d2 = (c_[0] - c_[1]).magnitude2();
            d3 = (c_[3] - c_[2]).magnitude2();
        } else {
            d2 = k * ((c_[1].x - c_[0].x) * d.x + (c_[1].y - c_[0].y) * d.y);
            d3 = k * ((c_[2].x - c_[0].x) * d.x + (c_[2].y - c_[0].y) * d.y);

            if d2 > 0.0 && d2 < 1.0 && d3 > 0.0 && d3 < 1.0 {
                return false; // simple 1-2-3-4 collinear case, stop recursion
            }

            if d2 <= 0.0 {
                d2 = (c_[0] - c_[1]).magnitude2();
            } else if d2 >= 1.0 {
                d2 = (c_[3] - c_[1]).magnitude2();
            } else {
                d2 = (vec2(c_[0].x + d2 * d.x, c_[0].y + d2 * d.y) - c_[1]).magnitude2();
            }

            if d3 <= 0.0 {
                d3 = (c_[0] - c_[2]).magnitude2();
            } else if d3 >= 1.0 {
                d3 = (c_[3] - c_[2]).magnitude2();
            } else {
                d3 = (vec2(c_[0].x + d3 * d.x, c_[0].y + d3 * d.y) - c_[2]).magnitude2();
            }
        }

        if d2 > d3 {
            if d2 < self.tolerance_squared {
                self.smp.push(c_[1]);
                self.smd.push(SampleData::new(t_[1]));

                return false;
            }
        } else if d3 < self.tolerance_squared {
            self.smp.push(c_[2]);
            self.smd.push(SampleData::new(t_[2]));

            return false;
        }

        true
    }

    /// Only one of the mid controls is meaningful
    fn onlyc2c1(&mut self, c_: [Vec2; 4], t_: [f32; 4], onlyc2: bool) -> bool {
        if self.angle_tolerance < ANGLE_EPSILON {
            self.smp.push((c_[1] + c_[2]) * 0.5);
            self.smd.push(SampleData::new((t_[1] + t_[2]) * 0.5));

            return false;
        }

        let mut da = if onlyc2 {
            ((c_[3].y - c_[2].y).atan2(c_[3].x - c_[2].x)
                - (c_[2].y - c_[1].y).atan2(c_[2].x - c_[1].x)).abs()
        } else {
            ((c_[2].y - c_[1].y).atan2(c_[2].x - c_[1].x)
                - (c_[1].y - c_[0].y).atan2(c_[1].x - c_[0].x)).abs()
        };

        if da >= PI {
            da = PI * 2.0 - da;
        }

        if da < self.angle_tolerance {
            self.smp.push(c_[1]);
            self.smd.push(SampleData::new(t_[1]));

            self.smp.push(c_[2]);
            self.smd.push(SampleData::new(t_[2]));

            return false;
        }

        true
    }

    /// The control points fully define a curve
    fn curve(&mut self, c_: [Vec2; 4], t_: [f32; 4]) -> bool {
        if self.angle_tolerance < ANGLE_EPSILON {
            self.smp.push((c_[1] + c_[2]) * 0.5);
            self.smd.push(SampleData::new((t_[1] + t_[2]) * 0.5));

            return false;
        }

        let d21 = (c_[2].y - c_[1].y).atan2(c_[2].x - c_[1].x);
        let mut da1 = (d21 - (c_[1].y - c_[0].y).atan2(c_[1].x - c_[0].x)).abs();
        let mut da2 = ((c_[3].y - c_[2].y).atan2(c_[3].x - c_[2].x) - d21).abs();

        if da1 >= PI {
            da1 = PI * 2.0 - da1;
        }
        if da2 >= PI {
            da2 = PI * 2.0 - da2;
        }

        if da1 + da2 < self.angle_tolerance {
            self.smp.push((c_[1] + c_[2]) * 0.5);
            self.smd.push(SampleData::new((t_[1] + t_[2]) * 0.5));

            return false;
        }

        true
    }

    /// Compute the additional data cached for the curve samples
    fn cache_sample_data(&mut self) {
        self.total_length = 0.0;
        self.bounds = [
            vec2(f32::INFINITY, f32::INFINITY),
            vec2(f32::NEG_INFINITY, f32::NEG_INFINITY),
        ];

        let n = self.smp.len();
        debug_assert!(n == self.smd.len());

        for i in 0..n {
            self.smd[i].n = self.normal_at(self.smd[i].t);
            if i < n - 1 {
                self.smd[i].l = (self.smp[i + 1] - self.smp[i]).magnitude();
            }

            let x = self.smp[i].x;
            let y = self.smp[i].y;

            if x < self.bounds[0].x {
                self.bounds[0].x = x;
            }
            if x > self.bounds[1].x {
                self.bounds[1].x = x;
            }
            if y < self.bounds[0].y {
                self.bounds[0].y = y;
            }
            if y > self.bounds[1].y {
                self.bounds[1].y = y;
            }

            self.total_length += self.smd[i].l;
        }
    }
}
