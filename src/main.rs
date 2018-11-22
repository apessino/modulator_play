//!
//! # Modulator Play
//!
//! An environment to visualize and test the modulator crate and to experiment with
//! expressive 2d primitive rendering. Based on Piston Window, this application is
//! meant to be both a test bed for the Modulator crate and its included source types,
//! and a minimal friction environment to experiment with Rust coding.
//!
//! Copyright© 2018 Ready At Dawn Studios

extern crate cgmath;
extern crate find_folder;
extern crate modulator;
extern crate num_complex;
extern crate rand;

extern crate draw_state;
extern crate gfx_device_gl;
extern crate gfx_graphics;
extern crate gfx_texture;
extern crate graphics;
extern crate piston_window;
extern crate shader_version;
extern crate shaders_graphics2d as shaders;

#[macro_use]
extern crate gfx;

pub mod bezier;
pub mod prims;

use modulator::sources::{
    Newtonian, ScalarGoalFollower, ScalarSpring, ShiftRegister, ShiftRegisterInterp, Wave,
};
use modulator::{Modulator, ModulatorEnv};

use piston_window::character::CharacterCache;
use piston_window::*;

use rand::prelude::*;
use std::{collections::HashMap, collections::VecDeque, f32, time::Instant};

use cgmath::prelude::*;
use cgmath::{vec2, Basis2, Decomposed, Rad, Vector2};

type Vec2 = Vector2<f32>;
type Bas2 = Basis2<f32>;
type Dec2 = Decomposed<Vec2, Bas2>;

use bezier::CubicCurve;
use prims::{PrimGraphics, Prims2d};

/// Application main loop
fn main() {
    let mut window: PistonWindow = WindowSettings::new("modulator_play", (1920, 1080))
        .samples(16)
        .exit_on_esc(true)
        .build()
        .unwrap_or_else(|e| panic!("error creating main window: {}", e));

    let mut data = DrawData::new(&mut window); // all data related to drawing
    let mut state = StateData::new(); // all other data

    setup_modulators(&mut state);

    let mut earlier = Instant::now();
    while let Some(e) = window.next() {
        let dt = time_delta(&mut earlier); // `earlier` updated in place
        update(&e, dt, &mut state, &mut data);

        if let Some(args) = e.render_args() {
            window.window.make_current();
            clear(data.theme[0], &mut data.draw_context(&mut window)); // clear window

            let c = Context::new_viewport(args.viewport()); // make a context, cache dimensions
            data.dims = c.get_view_size();
            data.hdims = [data.dims[0] * 0.5, data.dims[1] * 0.5];

            dispatch(&mut state, &mut data, &c, &mut window); // main update driver

            window.draw_2d(&e, |_, _| {}); // just so that window.g2d is flushed if needed
            window.encoder.flush(&mut window.device);
        };
    }
}

/// Populate the modulator environments with modulators
fn setup_modulators(st: &mut StateData) {
    // Make a wave modulator
    let wave = Wave::new(2.0, 0.5); // start with 2.0 amplitude and 0.5Hz frequency

    st.m1.take("waveform", Box::new(wave));
    st.bufs.insert("waveform".to_string(), VecDeque::new());

    set_wave_shape(WaveShape::Sine, st);

    // Make a random wave modulator
    let wave = Wave::new(2.0, 0.1).wave(Box::new(|w, _| {
        let n = w.value + thread_rng().gen_range(-w.frequency, w.frequency);
        f32::min(f32::max(n, -w.amplitude), w.amplitude)
    }));

    st.m1.take("rnd_wave", Box::new(wave));
    st.bufs.insert("rnd_wave".to_string(), VecDeque::new());

    toggle_modulator("rnd_wave", st); // start it off

    // Make a goal follower using a spring, use to modulate the amp of the sin wave
    let mut amp_mod = Box::new(ScalarGoalFollower::new(Box::new(ScalarSpring::new(
        1.0, 0.0, 1.0,
    ))));
    amp_mod.regions.push([0.0, 12.0]);
    st.m2.take("amp_mod", amp_mod); // modulates amp of waveform

    // Make a goal follower using a spring
    let mut fol = Box::new(ScalarGoalFollower::new(Box::new(ScalarSpring::new(
        0.3, 5.0, 0.0,
    ))));

    fol.regions.push([-3.0, -2.0]);
    fol.regions.push([-1.0, 1.0]);
    fol.regions.push([2.0, 3.0]);

    fol.random_region = true;
    fol.pause_range = [0, 100_000]; // short pause between goals

    st.m1.take("follow_s", fol);
    st.bufs.insert("follow_s".to_string(), VecDeque::new());

    toggle_modulator("follow_s", st); // start it off

    // Make a follower using a Newtonian
    let mut fol = Box::new(ScalarGoalFollower::new(Box::new(Newtonian::new(
        [2.0, 12.0],
        [4.0, 24.0],
        [4.0, 24.0],
        0.0,
    ))));

    fol.regions.push([-3.0, 3.0]);

    st.m1.take("follow_n", fol);
    st.bufs.insert("follow_n".to_string(), VecDeque::new());

    toggle_modulator("follow_n", st); // start it off

    // Make a shift register modulator
    let mut sreg = ShiftRegister::new(6, [-3.0, 3.0], 0.05, 1.0, ShiftRegisterInterp::None);
    sreg.age_range = [5, 30];

    st.m1.take("shift_rg", Box::new(sreg));
    st.bufs.insert("shift_rg".to_string(), VecDeque::new());

    toggle_modulator("shift_rg", st); // start it off

    // One more follower with spring for title animation
    let mut fol = Box::new(ScalarGoalFollower::new(Box::new(ScalarSpring::new(
        0.4, 5.0, 0.0,
    ))));
    fol.regions.push([-8.0, 8.0]);

    st.m2.take("follow", fol);

    // Bunch of followers with springs over 0.0/1.0
    for i in 1..=7 {
        let mut f = Box::new(ScalarGoalFollower::new(Box::new(ScalarSpring::new(
            1.0,
            0.0,
            rand::thread_rng().gen_range(0.0, 1.0),
        ))));

        f.regions.push([0.0, 1.0]);
        f.pause_range = [0, 1000000];

        st.m2.take(&format!("follow{}", i), f);
    }

    // One more wave modulator
    let wave = Wave::new(1.0, 0.3).wave(Box::new(|w, t| {
        (t * w.frequency * f32::consts::PI * 2.0).sin() * w.amplitude
    }));
    st.m2.take("waveform2", Box::new(wave));

    // Populate m3 with followers that pursue normalized coordinates
    for i in 0..8 {
        let mut f = Box::new(ScalarGoalFollower::new(Box::new(Newtonian::new(
            [2.0, 10.0],
            [2.0, 12.0],
            [2.0, 12.0],
            rand::thread_rng().gen_range(0.2, 0.8),
        ))));

        f.regions.push([0.0, 1.0]);
        f.pause_range = [0, 1000000];

        st.m3.take(&format!("follow{}", i), f);
    }
}

/// Set the current shape for the wave modulator
fn set_wave_shape(shape: WaveShape, st: &mut StateData) {
    let md = if let Some(sw) = st.m1.get_mut("waveform") {
        if let Some(ss) = sw.as_any().downcast_mut::<Wave>() {
            ss
        } else {
            return;
        }
    } else {
        return;
    };

    st.shape = shape;
    match st.shape {
        WaveShape::Sine => {
            md.set_enabled(true);
            md.set_wave(Box::new(|w, t| {
                (t * w.frequency * f32::consts::PI * 2.0).sin() * w.amplitude
            }));
        }
        WaveShape::Triangle => {
            md.set_enabled(true);
            md.set_wave(Box::new(|w, t| {
                let t_p = t * w.frequency;
                (2.0 * f32::abs(2.0 * (t_p - f32::floor(t_p + 0.5))) - 1.0) * w.amplitude
            }));
        }
        WaveShape::Square => {
            md.set_enabled(true);
            md.set_wave(Box::new(|w, t| {
                f32::signum((t * w.frequency * f32::consts::PI * 2.0).sin()) * w.amplitude
            }));
        }
        WaveShape::Saw => {
            md.set_enabled(true);
            md.set_wave(Box::new(|w, t| {
                let t_p = t * w.frequency;
                (2.0 * (t_p - f32::floor(t_p + 0.5))) * w.amplitude
            }));
        }
        WaveShape::None => {
            md.set_enabled(false);
            md.set_wave(Box::new(|_, _| 0.0));
        }
    }
}

/// Cycle the closure/activation state used for the wave modulator
fn cycle_wave_shape(st: &mut StateData) {
    match st.shape {
        WaveShape::Sine => set_wave_shape(WaveShape::Triangle, st),
        WaveShape::Triangle => set_wave_shape(WaveShape::Square, st),
        WaveShape::Square => set_wave_shape(WaveShape::Saw, st),
        WaveShape::Saw => set_wave_shape(WaveShape::None, st),
        WaveShape::None => set_wave_shape(WaveShape::Sine, st),
    }
}

/// Cycle the follow spring undmping mode and activation
fn cycle_follow_spring(key: &str, st: &mut StateData) {
    if let Some(sw) = st.m1.get_mut(key) {
        if let Some(ss) = sw.as_any().downcast_mut::<ScalarGoalFollower>() {
            let mut enabled = ss.enabled();

            if enabled {
                if let Some(sp) = ss.follower.as_any().downcast_mut::<ScalarSpring>() {
                    if sp.undamp == 0.0 {
                        enabled = false;
                        sp.undamp = 5.0;
                    } else {
                        sp.undamp = 0.0;
                    }
                }
            } else {
                enabled = true;
            }

            ss.set_enabled(enabled);
        }
    }
}

/// Cycle the closure/activation state used for the wave modulator
fn cycle_shift_reg_interpolation(key: &str, st: &mut StateData) {
    if let Some(sw) = st.m1.get_mut(key) {
        if let Some(ss) = sw.as_any().downcast_mut::<ShiftRegister>() {
            if ss.enabled() {
                match ss.interp {
                    ShiftRegisterInterp::Quadratic => ss.set_enabled(false),
                    ShiftRegisterInterp::Linear => ss.interp = ShiftRegisterInterp::Quadratic,
                    ShiftRegisterInterp::None => ss.interp = ShiftRegisterInterp::Linear,
                }
            } else {
                ss.set_enabled(true);
                ss.interp = ShiftRegisterInterp::None;
            }
        }
    }
}

/// Toggle activation status for the given modulator
fn toggle_modulator(key: &str, st: &mut StateData) {
    let m = if let Some(s) = st.m1.get_mut(key) {
        s
    } else if let Some(s) = st.m2.get_mut(key) {
        s
    } else {
        return;
    };

    let state = m.enabled();
    m.set_enabled(!state);
}

/// Try to get a ref to the give modulator - searches in m1 first, m2 if not found
fn get_modulator<'a>(
    key: &str,
    st: &'a StateData,
) -> Option<&'a Box<dyn modulator::Modulator<f32>>> {
    return if let Some(s) = st.m1.get(key) {
        Some(s)
    } else if let Some(s) = st.m2.get(key) {
        Some(s)
    } else {
        None
    };
}

/// ###
/// ### Menus
/// ###

enum MenuPages {
    MainMenu,

    Modulators,
    Primitives,
    CubicCurves,
}

/// Process and dispatch the current menu
fn dispatch(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    match st.menu_page {
        MenuPages::MainMenu => main_menu(st, d, c, w),

        MenuPages::Modulators => vis_mod_waves(st, d, c, w),
        MenuPages::Primitives => primitives(st, d, c, w),
        MenuPages::CubicCurves => cubic_curves(st, d, c, w),
    }

    match st.entered_text.as_ref() {
        " " => {
            st.paused = !st.paused;
            st.entered_text = String::new();
        }
        _ => (),
    }

    if st.paused {
        let at = vec2(d.dims[0] as f32 - 148.0, d.dims[1] as f32 - 18.0);
        draw_text("[PAUSED]", at, d.theme[9], 24, d, c, w);
    }
}

/// Draw a menu display
fn draw_menu(
    menu: &[String],
    col: types::Color,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let mut at = vec2(16.0, 32.0);
    for m in menu {
        draw_text(m, at, col, 16, d, c, w);
        at.y += 20.0;
    }
}

/// Draw a menu display
fn draw_info(
    info: &[String],
    col: types::Color,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let n = info.len();
    let mut at = vec2(16.0, d.dims[1] as f32 - n as f32 * 18.0);

    for m in info {
        draw_text(m, at, col, 14, d, c, w);
        at.y += 18.0;
    }
}

/// Main Menu
fn main_menu(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    main_menu_exe(st, d, c, w);

    // Draw the menu
    let menu = [
        "MODULATOR PLAY - app for Modulator trait, extended primitives testing".to_string(),
        "".to_string(),
        "[ 1 ] Modulators".to_string(),
        "[ 2 ] Primitives".to_string(),
        "[ 3 ] Cubic Curves".to_string(),
    ];
    draw_menu(&menu, d.theme[1], d, c, w);

    let info = [
        "By Andrea Pessino & Jason Hise".to_string(),
        "Copyright © 2018 Ready At Dawn Studios".to_string(),
    ];
    draw_info(&info, d.theme[6], d, c, w);

    // Process the menu
    match st.entered_text.as_ref() {
        "1" => {
            st.menu_page = MenuPages::Modulators;
            st.entered_text = String::new();
        }
        "2" => {
            st.menu_page = MenuPages::Primitives;
            st.entered_text = String::new();
        }
        "3" => {
            st.menu_page = MenuPages::CubicCurves;
            st.entered_text = String::new();
        }
        _ => (),
    }
}

fn main_menu_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    penta_dance(st, d, c, w);
    tri_dance(st, d, c, w);
}

/// Visualize some of the modulator waves
fn vis_mod_waves(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    vis_mod_waves_exe(st, d, c, w);

    // Draw the menu
    let menu = [
        "MODULATORS - demo of types implementing the Modulator trait".to_string(),
        "".to_string(),
        "[ 1 ] Wave: sin > tri > square > saw > off".to_string(),
        "[ 2 ] Follow|Spring: undamped > damped > off".to_string(),
        "[ 3 ] Follow|Newtonian: on > off".to_string(),
        "[ 4 ] Wave|Random Walk: on > off".to_string(),
        "[ 5 ] Shift Register: near > lin > quad > off".to_string(),
        "".to_string(),
        "[ 0 ] Back".to_string(),
    ];
    draw_menu(&menu, d.theme[1], d, c, w);

    // Process the menu
    match st.entered_text.as_ref() {
        "0" => {
            st.menu_page = MenuPages::MainMenu;
            st.entered_text = String::new();
        }
        "1" => {
            cycle_wave_shape(st);
            st.entered_text = String::new();
        }
        "2" => {
            cycle_follow_spring("follow_s", st);
            st.entered_text = String::new();
        }
        "3" => {
            toggle_modulator("follow_n", st);
            st.entered_text = String::new();
        }
        "4" => {
            toggle_modulator("rnd_wave", st);
            st.entered_text = String::new();
        }
        "5" => {
            cycle_shift_reg_interpolation("shift_rg", st);
            st.entered_text = String::new();
        }
        _ => (),
    }
}

fn vis_mod_waves_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    draw_ruler(d, c, w);

    let mut ord = 0;

    draw_modulator("waveform", &mut ord, 1, st, d, c, w);
    draw_modulator("follow_s", &mut ord, 9, st, d, c, w);
    draw_modulator("follow_n", &mut ord, 2, st, d, c, w);
    draw_modulator("rnd_wave", &mut ord, 10, st, d, c, w);
    draw_modulator("shift_rg", &mut ord, 6, st, d, c, w);
}

/// Testing of prim drawing, lin e segments
fn primitives(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    primitives_exe(st, d, c, w);

    // Draw the menu
    let menu = [
        "PRIMITIVES - special prims provided by prims.rs".to_string(),
        "".to_string(),
        "[ 1 ] Toggle n-gons filled".to_string(),
        "[ 2 ] Toggle connectors".to_string(),
        "[ 3 ] Toggle debug quads".to_string(),
        "[ 4 ] Toggle debug wireframe".to_string(),
        "".to_string(),
        "[ 0 ] Back".to_string(),
    ];
    draw_menu(&menu, d.theme[1], d, c, w);

    // Process the menu
    match st.entered_text.as_ref() {
        "1" => {
            d.prim_filled = !d.prim_filled;
            st.entered_text = String::new();
        }
        "2" => {
            d.prim_no_connect = !d.prim_no_connect;
            st.entered_text = String::new();
        }
        "3" => {
            d.prim_face_debug = !d.prim_face_debug;
            st.entered_text = String::new();
        }
        "4" => {
            d.prim_debug = !d.prim_debug;
            st.entered_text = String::new();
        }
        "0" => {
            st.menu_page = MenuPages::MainMenu;
            st.entered_text = String::new();
        }
        _ => (),
    }
}

fn primitives_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    single_lines_exe(st, d, c, w);
    complex_sweeps_exe(st, d, c, w);
    n_gons_exe(st, d, c, w);
}

fn single_lines_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let mut lp = prims::DrawParms::new();

    let rot = st.m2.value("follow") * 0.4;
    let rst = f32::consts::PI / 3.0;

    let center = vec2(450.0, 600.0);
    let halfl = 500.0 * 0.5;

    // First line
    lp.color = [d.theme[8], d.theme[6]];
    lp.thickness = [6.0, 42.0];
    lp.cap = [prims::CapStyle::Round(16), prims::CapStyle::Round(32)];

    let (sn, cs) = rot.sin_cos();
    let dir = vec2(cs - sn, cs + sn);
    let l0 = [
        vec2(center.x - dir.x * halfl, center.y - dir.y * halfl),
        vec2(center.x + dir.x * halfl, center.y + dir.y * halfl),
    ];
    d.prim_context_debug(w).draw_line(l0, &lp, c);

    // Second line
    lp.color = [d.theme[1], d.theme[2]];
    lp.color[0][3] = 0.5;
    lp.color[1][3] = 0.5;
    lp.thickness = [20.0, (st.m2.value("waveform2") * 18.0).abs()];
    lp.cap = [prims::CapStyle::Square(0.0), prims::CapStyle::Round(32)];

    let (sn, cs) = (rot + rst).sin_cos();
    let dir = vec2(cs - sn, cs + sn);
    let l1 = [
        vec2(center.x - dir.x * halfl, center.y - dir.y * halfl),
        vec2(center.x + dir.x * halfl, center.y + dir.y * halfl),
    ];
    d.prim_context_debug(w).draw_line(l1, &lp, c);

    // Third line
    lp.color = [d.theme[9], d.theme[11]];
    lp.color[0][3] = 0.5;
    lp.color[1][3] = 0.5;
    lp.cap = [prims::CapStyle::Round(16), prims::CapStyle::Square(0.0)];
    lp.thickness = [st.m2.value("follow4") * 60.0, st.m2.value("follow3") * 60.0];

    let (sn, cs) = (rot + rst * 2.0).sin_cos();
    let dir = vec2(cs - sn, cs + sn);
    let l1 = [
        vec2(center.x - dir.x * halfl, center.y - dir.y * halfl),
        vec2(center.x + dir.x * halfl, center.y + dir.y * halfl),
    ];
    d.prim_context_debug(w).draw_line(l1, &lp, c);
}

fn complex_sweeps_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let mut lp0 = prims::DrawParms::new();

    lp0.thickness = [30.0, 30.0];
    if d.prim_no_connect {
        lp0.connect_subs = 0;
    }

    let o0 = vec2(1300.0, 280.0);
    let o1 = o0;

    let r = st.m2.elapsed_us("waveform2") as f32 / 1_000_000 as f32;

    let r0 = cgmath::Matrix2::from_angle(Rad(r * 0.3));
    let r1 = cgmath::Matrix2::from_angle(Rad(r * -0.6));

    lp0.thickness[0] = 30.0 + st.m2.value("follow").abs() * 4.0;
    lp0.thickness[1] = 30.0;

    let line0 = [
        o0 + r0 * (vec2(o0[0] - 280.0, o0[1]) - o0),
        o0 + r0 * (vec2(o0[0] - 80.0, o0[1]) - o0),
    ];
    let line1 = [
        o1 + r1 * (vec2(o0[0], o0[1] + 80.0) - o1),
        o1 + r1 * (vec2(o0[0], o0[1] + 400.0) - o1),
    ];

    lp0.color = [d.theme[2], d.theme[6]];
    let lp1 = prims::DrawParms {
        thickness: [30.0, 30.0 + st.m2.value("waveform2") * 3.0],
        color: [d.theme[3], d.theme[7]],
        connect_subs: 0,
        ..lp0
    };

    d.prim_context_debug(w)
        .draw_lines(&[line0, line1], &[lp0, lp1], &c);
}

fn n_gons_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let dp = prims::DrawParms::new()
        .color([d.theme[1], d.theme[9]])
        .thickness([12.0, 12.0])
        .connect_subs(if d.prim_no_connect { 0 } else { 16 });
    let trs = Dec2 {
        scale: 160.0 + st.m2.value("follow2") * 16.0,
        disp: vec2(1180.0, 860.0),
        rot: Rotation2::from_angle(Rad(st.m2.value("follow1") * 30.0)),
    };

    let (lines, parms) = prims::n_gon(4, st.m2.value("follow3"), dp, &trs, None);
    if d.prim_filled {
        d.prim_context_debug(w)
            .draw_lines_filled(&lines, &parms, &c);
    } else {
        d.prim_context_debug(w).draw_lines(&lines, &parms, &c);
    }

    let dp = prims::DrawParms::new()
        .color([d.theme[7], d.theme[10]])
        .thickness([
            4.0 + st.m2.value("follow4") * 12.0,
            4.0 + st.m2.value("follow5") * 12.0,
        ]).connect_subs(if d.prim_no_connect { 0 } else { 16 });
    let trs = Dec2 {
        scale: 150.0,
        disp: vec2(1600.0, 860.0),
        rot: Rotation2::from_angle(Rad(st.m2.value("follow6") * 30.0)),
    };

    let (lines, parms) = prims::n_gon(7, st.m2.value("follow7"), dp, &trs, None);
    if d.prim_filled {
        d.prim_context_debug(w)
            .draw_lines_filled(&lines, &parms, &c);
    } else {
        d.prim_context_debug(w).draw_lines(&lines, &parms, &c);
    }
}

/// Generate a 5-gon primitive and then animate its parameters
fn penta_dance(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let c0 = 1.0 + st.m2.value("follow4") * (d.theme.len() - 1) as f32;
    let c1 = 1.0 + st.m2.value("follow5") * (d.theme.len() - 1) as f32;

    let dp = prims::DrawParms::new()
        .color([blended_theme_color(c0, d), blended_theme_color(c1, d)])
        .thickness([
            2.0 + 30.0 * st.m2.value("follow2"),
            2.0 + 30.0 * st.m2.value("follow3"),
        ]).connect_subs(16);
    let trs = Dec2 {
        scale: 200.0 * st.m2.value("waveform2").abs() * 0.5,
        disp: vec2(d.hdims[0] as f32, d.hdims[1] as f32),
        rot: Rotation2::from_angle(Rad(st.m2.value("follow"))),
    };

    let (lines, parms) = prims::n_gon(5, 0.85 * st.m2.value("follow1"), dp, &trs, None);

    let mut cd = d.prim_context(w);
    cd.draw_lines_filled(&lines, &parms, &c);
}

/// Generate a 3-gon primitive and then animate its parameters
fn tri_dance(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let c0 = 1.0 + st.m2.value("follow1") * (d.theme.len() - 1) as f32;
    let c1 = 1.0 + st.m2.value("follow2") * (d.theme.len() - 1) as f32;

    let dp = prims::DrawParms::new()
        .color([blended_theme_color(c0, d), blended_theme_color(c1, d)])
        .thickness([
            2.0 + 30.0 * st.m2.value("follow3"),
            2.0 + 30.0 * st.m2.value("follow4"),
        ]).connect_subs(16);
    let trs = Dec2 {
        scale: 200.0 * st.m2.value("follow") * 0.5,
        disp: vec2(d.hdims[0] as f32, d.hdims[1] as f32),
        rot: Rotation2::from_angle(Rad(st.m2.value("waveform2"))),
    };

    let (lines, parms) = prims::n_gon(3, 0.85 * st.m2.value("follow5"), dp, &trs, None);

    let mut cd = d.prim_context(w);
    cd.draw_lines(&lines, &parms, &c);
}

/// Testing cubic curves
struct CubicCurvesData {
    c0: CubicCurve, // curve being drawn

    tracking: bool,                       // the curve is being tracked
    tracking_at: prims::CurveQueryResult, // cached query result when we started tracking
    tracking_ct: [Vec2; 4],               // snapshot of controls when tracking started

    thick_choices: Vec<[f32; 2]>,
    thick_choice: usize,

    connectors: u16,
}
impl CubicCurvesData {
    fn new() -> Self {
        let mut c0 = CubicCurve::new();

        c0.set_control_points(&[
            vec2(286.0, 729.0),
            vec2(598.0, 925.0),
            vec2(308.0, 198.0),
            vec2(1232.0, 462.0),
        ]);

        CubicCurvesData {
            c0,
            tracking: false,
            tracking_at: prims::CurveQueryResult::None,
            tracking_ct: [vec2(0.0, 0.0); 4],

            thick_choices: vec![[12.0, 12.0], [32.0, 32.0], [6.0, 48.0], [3.0, 3.0]],
            thick_choice: 0,

            connectors: 4,
        }
    }
}

fn cubic_curves(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    cubic_curves_exe(st, d, c, w);

    // Draw the menu
    let menu = [
        "CUBIC CURVES - prim rendered cubic curves with Shemanarev metrics".to_string(),
        "".to_string(),
        "[ 1 ] Cycle thickness".to_string(),
        "[ 2 ] Toggle connectors".to_string(),
        "[ 3 ] Toggle animation".to_string(),
        "".to_string(),
        "[ 0 ] Back".to_string(),
    ];
    draw_menu(&menu, d.theme[1], d, c, w);

    // Process the menu
    match st.entered_text.as_ref() {
        "0" => {
            st.menu_page = MenuPages::MainMenu;
            st.entered_text = String::new();
        }
        "1" => {
            cycle_curve_thickness(st);
            st.entered_text = String::new();
        }
        "2" => {
            toggle_curve_connectors(st);
            st.entered_text = String::new();
        }
        "3" => {
            st.curve_animate = !st.curve_animate;
            st.entered_text = String::new();
        }
        _ => (),
    }
}

/// Cycle the closure/activation state used for the wave modulator
fn cycle_curve_thickness(st: &mut StateData) {
    let n = st.curve_data.thick_choices.len();
    st.curve_data.thick_choice = if st.curve_data.thick_choice < n - 1 {
        st.curve_data.thick_choice + 1
    } else {
        0
    };
}

/// Show/hide the curve sample segment connectors
fn toggle_curve_connectors(st: &mut StateData) {
    if st.curve_data.connectors == 0 {
        st.curve_data.connectors = 4;
    } else {
        st.curve_data.connectors = 0;
    }
}

fn cubic_curves_exe(st: &mut StateData, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let mut dp = prims::CurveDrawParms::new()
        .parms(
            prims::DrawParms::new()
                .color([d.theme[5], d.theme[1]])
                .thickness(st.curve_data.thick_choices[st.curve_data.thick_choice])
                .cap([prims::CapStyle::Round(16), prims::CapStyle::Round(16)])
                .connect_subs(st.curve_data.connectors),
        ).extra_features(true)
        .segment_scale(0.8);

    let (mouse_pos, cq) = curve_tracking(st, d, &dp);
    if st.curve_animate {
        curve_animation(st, &mut dp, d);
    }

    d.prim_context(w)
        .draw_curve(&mut st.curve_data.c0, &dp, mouse_pos, &cq, None, &c);

    // If enabled, draw some info text
    if dp.feature_info {
        let mut tx = String::new();

        match cq {
            prims::CurveQueryResult::Sample {
                k,
                pt: _,
                time,
                distance: _,
            } => tx = format!("sample: {:?} at: {:.2}", k, time),
            prims::CurveQueryResult::Handle {
                k: _,
                pt: _,
                time: _,
                distance: _,
            } => (),
            prims::CurveQueryResult::Control { k, pt, distance: _ } => {
                tx = format!("control: {:?} loc: {:.2}, {:.2}", k, pt.x, pt.y)
            }
            prims::CurveQueryResult::None => {
                if st.curve_data.c0.in_bounds(mouse_pos, dp.sample_radius) {
                    tx = format!(
                        "samples: {:?} length: {:.2}",
                        st.curve_data.c0.samples().len(),
                        st.curve_data.c0.length()
                    );
                }
            }
        }

        if tx.len() > 0 {
            let tc = dp.selecc.mul_rgba(1.0, 1.0, 1.0, 0.5 as f32);
            let at = mouse_pos + vec2(24.0, 24.0);

            draw_text(&tx, at, tc, 12, d, c, w);
        }
    }
}

/// Manage the tracking/interaction of the curve
fn curve_tracking(
    st: &mut StateData,
    d: &mut DrawData,
    dp: &prims::CurveDrawParms,
) -> (Vec2, prims::CurveQueryResult) {
    let mut mouse_pos = d.mouse_pos; // begin with the current mouse position
    let mut cq = dp.find(&mut st.curve_data.c0, mouse_pos);

    let td = mouse_pos - st.tracking_from; // tracking offset

    if st.tracking == false {
        st.curve_data.tracking = false;
        st.curve_data.tracking_at = prims::CurveQueryResult::None;
    } else if st.curve_data.tracking == false {
        st.curve_data.tracking = true;
        st.curve_data.tracking_at = cq;
        st.curve_data.tracking_ct = st.curve_data.c0.control_points().clone(); // starting controls
    }

    if st.curve_data.tracking {
        match st.curve_data.tracking_at {
            prims::CurveQueryResult::Sample {
                k,
                pt,
                time,
                distance,
            } => {
                let cs = [
                    st.curve_data.tracking_ct[0] + td,
                    st.curve_data.tracking_ct[1] + td,
                    st.curve_data.tracking_ct[2] + td,
                    st.curve_data.tracking_ct[3] + td,
                ];
                st.curve_data.c0.set_control_points(&cs);
                cq = prims::CurveQueryResult::Sample {
                    k,
                    pt: pt + td,
                    time,
                    distance,
                };
            }
            prims::CurveQueryResult::Handle {
                k,
                pt,
                time,
                distance,
            } => {
                let cs = [
                    if k == 0 {
                        st.curve_data.tracking_ct[0] + td
                    } else {
                        st.curve_data.tracking_ct[0]
                    },
                    if k == 0 {
                        st.curve_data.tracking_ct[1] + td
                    } else {
                        st.curve_data.tracking_ct[1]
                    },
                    if k == 1 {
                        st.curve_data.tracking_ct[2] + td
                    } else {
                        st.curve_data.tracking_ct[2]
                    },
                    if k == 1 {
                        st.curve_data.tracking_ct[3] + td
                    } else {
                        st.curve_data.tracking_ct[3]
                    },
                ];
                st.curve_data.c0.set_control_points(&cs);
                cq = prims::CurveQueryResult::Handle {
                    k,
                    pt: pt + td,
                    time,
                    distance,
                };
            }
            prims::CurveQueryResult::Control { k, pt, distance } => {
                let cs = [
                    if k == 0 {
                        st.curve_data.tracking_ct[0] + td
                    } else {
                        st.curve_data.tracking_ct[0]
                    },
                    if k == 1 {
                        st.curve_data.tracking_ct[1] + td
                    } else {
                        st.curve_data.tracking_ct[1]
                    },
                    if k == 2 {
                        st.curve_data.tracking_ct[2] + td
                    } else {
                        st.curve_data.tracking_ct[2]
                    },
                    if k == 3 {
                        st.curve_data.tracking_ct[3] + td
                    } else {
                        st.curve_data.tracking_ct[3]
                    },
                ];
                st.curve_data.c0.set_control_points(&cs);
                cq = prims::CurveQueryResult::Control {
                    k,
                    pt: pt + td,
                    distance,
                };
            }
            prims::CurveQueryResult::None => {
                mouse_pos = st.tracking_from;
                cq = prims::CurveQueryResult::None;
            }
        }
    }

    (mouse_pos, cq)
}

/// Animate the curve with modulators
fn curve_animation(st: &mut StateData, dp: &mut prims::CurveDrawParms, d: &mut DrawData) {
    let x = d.dims[0] as f32;
    let y = d.dims[1] as f32;

    let cs = [
        vec2(x * st.m3.value("follow0"), y * st.m3.value("follow1")),
        vec2(x * st.m3.value("follow2"), y * st.m3.value("follow3")),
        vec2(x * st.m3.value("follow4"), y * st.m3.value("follow5")),
        vec2(x * st.m3.value("follow6"), y * st.m3.value("follow7")),
    ];

    st.curve_data.c0.set_control_points(&cs);

    dp.samples = false; // not while animating
    dp.bounds = false;
    dp.feature_info = false;

    let c0 = 1.0 + st.m2.value("follow4") * (d.theme.len() - 1) as f32;
    let c1 = 1.0 + st.m2.value("follow5") * (d.theme.len() - 1) as f32;

    dp.parms.color = [blended_theme_color(c0, d), blended_theme_color(c1, d)];
}

/// ###
/// ### Build and drive the application state
/// ###

enum WaveShape {
    Sine,
    Triangle,
    Square,
    Saw,
    None,
}

/// Container for application state data
struct StateData {
    menu_page: MenuPages,
    paused: bool,

    m1: ModulatorEnv<f32>,
    m2: ModulatorEnv<f32>,
    m3: ModulatorEnv<f32>, // dedicated to animating the curve controls

    buf_samples_per_sec: u32, // number of modulator samples collected per second
    buf_max_fade_time: f32,   // max seconds to fade out the buffer

    buf_scroll_vel: f32, // horizontal velocity of samples

    bufs: HashMap<String, VecDeque<f32>>, // modulator ring buffers
    sampling_dt: u64,                     // microseconds since the last time we captured samples

    shape: WaveShape,            // current shape for the wave modulator
    curve_data: CubicCurvesData, // testing curve state

    tracking: bool,      // when true, the left mouse button is being held
    tracking_from: Vec2, // position of the mouse pointer when the tracking started
    mouse_scroll: Vec2,  // mouse scrolling (wheels)
    curve_animate: bool, // animate the curve with followers

    entered_text: String, // current text input
}

impl StateData {
    fn new() -> Self {
        StateData {
            menu_page: MenuPages::MainMenu,
            paused: false,

            m1: ModulatorEnv::new(),
            m2: ModulatorEnv::new(),
            m3: ModulatorEnv::new(),

            buf_samples_per_sec: 30,
            buf_max_fade_time: 2.0,

            buf_scroll_vel: 500.0,

            bufs: HashMap::new(),
            sampling_dt: 0,

            shape: WaveShape::Sine,
            curve_data: CubicCurvesData::new(),

            tracking: false,
            tracking_from: vec2(f32::INFINITY, f32::INFINITY),
            mouse_scroll: vec2(0.0, 0.0),
            curve_animate: false,

            entered_text: String::new(),
        }
    }
}

/// Container for rendering state data
struct DrawData {
    theme: [types::Color; 14], // color theme
    glyphs: Glyphs,            // font data

    dot_rad: f64, // radius for the position dots
    tak_len: f64, // lenght of the axis tacks

    px_target: f64, // target subdivision of range in pixels
    px_base: f64,   // log base for the grid subdivisions

    margin_scale: f32, // scale of the calculated domain, to leave some margin

    prim_no_connect: bool, // do not draw the connectors
    prim_debug: bool,      // enable wireframe debug rendering for prim page
    prim_face_debug: bool, // enable debug face rendering for prim page
    prim_filled: bool,     // draw the bottom ngons filled in prim page

    domain: [f32; 2], // frame parameters

    dims: [f64; 2],
    hdims: [f64; 2],

    c2d: Box<Prims2d<gfx_device_gl::Resources>>,

    mouse_pos: Vec2, // current position of the mouse pointer
}

impl DrawData {
    /// Initialize all rendering data
    fn new(window: &mut PistonWindow) -> Self {
        let assets = find_folder::Search::ParentsThenKids(3, 3)
            .for_folder("assets")
            .unwrap();

        let theme = color_theme();
        let glyphs = Glyphs::new(
            assets.join("FiraCode-Medium.ttf"),
            window.factory.clone(),
            TextureSettings::new(),
        ).unwrap();

        let c2d = Prims2d::new(OpenGL::V3_2, &mut window.factory);

        DrawData {
            theme,
            glyphs,
            dot_rad: 5.0,
            tak_len: 14.0,
            px_target: 120.0,
            px_base: 2.0,
            margin_scale: 1.1, // 10% on each side
            prim_no_connect: false,
            prim_debug: false,
            prim_face_debug: false,
            prim_filled: false,
            domain: [-1.0, 1.0],
            dims: [0.0, 0.0],
            hdims: [0.0, 0.0],
            c2d: Box::new(c2d),
            mouse_pos: vec2(f32::INFINITY, f32::INFINITY),
        }
    }

    /// Create a piston drawing context
    fn draw_context<'a>(
        &self,
        window: &'a mut PistonWindow,
    ) -> gfx_graphics::GfxGraphics<'a, gfx_device_gl::Resources, gfx_device_gl::CommandBuffer> {
        gfx_graphics::GfxGraphics::new(
            &mut window.encoder,
            &window.output_color,
            &window.output_stencil,
            &mut window.g2d,
        )
    }

    /// Create a drawing context for our custom primitives
    fn prim_context<'a>(
        &'a mut self,
        window: &'a mut PistonWindow,
    ) -> PrimGraphics<'a, gfx_device_gl::Resources, gfx_device_gl::CommandBuffer> {
        PrimGraphics::new(
            &mut window.encoder,
            &window.output_color,
            &window.output_stencil,
            &mut self.c2d,
        )
    }

    /// Create a custom primitive drawing context, with optional debug wireframe rendering
    fn prim_context_debug<'a>(
        &'a mut self,
        window: &'a mut PistonWindow,
    ) -> PrimGraphics<'a, gfx_device_gl::Resources, gfx_device_gl::CommandBuffer> {
        let dw = self.prim_debug;
        let df = self.prim_face_debug;
        let mut d = self.prim_context(window);
        d.debug_wireframe = dw;
        d.debug_faces = df;
        d
    }
}

/// Calculate the corrected rendering coordinate given a location in the domain
fn calc_y(n: f64, d: &mut DrawData) -> [f64; 2] {
    let dom = (d.domain[1] - d.domain[0]) as f64 * 0.5;
    [d.hdims[0], d.hdims[1] - n / dom * d.hdims[1]]
}
fn calc_xy(n: f64, x: f64, d: &mut DrawData) -> [f64; 2] {
    let xy = calc_y(n, d);
    [xy[0] + x, xy[1]]
}

/// Draw a tack with label on the y axis
fn draw_tack(n: f64, scale: f64, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let g = &mut d.draw_context(w);
    let at = calc_y(n, d);

    line::Line::new(d.theme[8], 1.0).draw(
        [d.hdims[0], at[1], d.hdims[0] - d.tak_len * scale, at[1]],
        &c.draw_state,
        c.transform,
        g,
    );

    let tx = format!("{:.1}", n);
    let tw = d.glyphs.width(12, &tx).unwrap();
    let transform = c
        .transform
        .trans(at[0] - d.tak_len * scale - tw - 6.0, at[1] + d.dot_rad);
    text::Text::new_color(d.theme[7].mul_rgba(1.0, 1.0, 1.0, scale as f32), 12)
        .draw(&tx, &mut d.glyphs, &c.draw_state, transform, g)
        .unwrap();
}

/// Draw a positon dot with label on the y axis
fn draw_dot(n: f64, col: usize, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let at = calc_y(n, d);

    ellipse::Ellipse::new(d.theme[col]).draw(
        [
            at[0] - d.dot_rad,
            at[1] - d.dot_rad,
            d.dot_rad * 2.0,
            d.dot_rad * 2.0,
        ],
        &c.draw_state,
        c.transform,
        &mut d.draw_context(w),
    );

    draw_label(n, col, d, c, w);
}

/// Draw a positon dot with label on the y axis
fn draw_label(n: f64, col: usize, d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    let g = &mut d.draw_context(w);
    let at = calc_y(n, d);

    let transform = c
        .transform
        .trans(at[0] + d.dot_rad + 6.0, at[1] + d.dot_rad);
    text::Text::new_color(d.theme[col], 12)
        .draw(
            &format!("{:+.3}", n),
            &mut d.glyphs,
            &c.draw_state,
            transform,
            g,
        ).unwrap();
}

/// Draw a domain bracket
fn draw_bracket(
    key: &str,
    val: f32,
    gol: f32,
    dom: [f32; 2],
    ord: usize,
    col: usize,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let xvalue = 82.0;
    let xbase = xvalue + 64.0 * ord as f64;
    let xoffset = 64.0;

    let top = calc_xy(dom[1] as f64, xbase, d);
    let bot = calc_xy(dom[0] as f64, xbase, d);

    let val = calc_xy(val as f64, xvalue, d);
    let gol = calc_xy(gol as f64, xbase + xoffset, d);

    let col = d.theme[col].mul_rgba(1.0, 1.0, 1.0, 0.5 as f32);

    {
        let g = &mut d.draw_context(w);
        line::Line::new(col, 1.0).draw(
            [top[0], top[1], top[0] + xoffset, top[1]],
            &c.draw_state,
            c.transform,
            g,
        );
        line::Line::new(col, 1.0).draw(
            [top[0] + xoffset, top[1], bot[0] + xoffset, bot[1]],
            &c.draw_state,
            c.transform,
            g,
        );
        line::Line::new(col, 1.0).draw(
            [bot[0] + xoffset, bot[1], bot[0], bot[1]],
            &c.draw_state,
            c.transform,
            g,
        );

        line::Line::new(col, 1.0).draw(
            [val[0], val[1], gol[0], gol[1]],
            &c.draw_state,
            c.transform,
            g,
        );
    }

    draw_text(
        key,
        vec2(top[0] as f32 + 2.0, top[1] as f32 - 6.0),
        col,
        10,
        d,
        c,
        w,
    );
}

/// Draw a little animated shape to show the modulator affecting its rotation
fn draw_anishape(
    val: f32,
    ord: usize,
    col: usize,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let dp = prims::DrawParms::new()
        .color([d.theme[col], d.theme[col]])
        .thickness([2.0, 2.0])
        .connect_subs(4);
    let trs = Dec2 {
        scale: 80.0,
        disp: vec2(d.dims[0] as f32 - 100.0, 100.0 + 160.0 * ord as f32),
        rot: Rotation2::from_angle(Rad(val * 2.0)),
    };

    let (lines, parms) = prims::n_gon(3, 0.7, dp, &trs, None);

    let mut cd = d.prim_context(w);
    cd.draw_lines(&lines, &parms, &c);
}

/// Plot some text
fn draw_text(
    tx: &str,
    at: Vec2,
    col: types::Color,
    size: types::FontSize,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let g = &mut d.draw_context(w);
    let transform = c.transform.trans(at.x as f64, at.y as f64);

    text::Text::new_color(col, size)
        .draw(&tx, &mut d.glyphs, &c.draw_state, transform, g)
        .unwrap();
}

/// Draw a sample buffer as a fading, scrolling smooth connected line
fn draw_buf(
    cur: f32,
    buf: &VecDeque<f32>,
    col: usize,
    st: &StateData,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let n = buf.len();
    if n == 0 {
        return;
    }

    let xbase = d.hdims[0] as f32;
    let dom = (d.domain[1] - d.domain[0]) * 0.5;
    let ydim = d.hdims[1] as f32;

    let mut ls = Vec::with_capacity(n);
    let mut l0 = vec2(xbase, ydim - cur / dom * ydim);

    let sample_time = 1.0 / st.buf_samples_per_sec as f32; // time between taking samples
    let sample_vel = st.buf_scroll_vel * sample_time; // displacement over one period
    let frac_period = st.sampling_dt as f32 / 1_000_000.0; // fraction of current period
    let xbase = xbase - frac_period * sample_vel; // displacement over fraction of period

    for i in 1..n {
        let l1 = vec2(xbase - sample_vel * i as f32, ydim - buf[i] / dom * ydim);
        ls.push([l0, l1]);
        l0 = l1;
    }

    let cl = d.theme[col];
    let rad = d.dot_rad as f32;
    let dp = prims::DrawParms {
        color: [cl, [cl[0], cl[1], cl[2], 0.0]],
        thickness: [rad, 0.0],
        cap: [prims::CapStyle::Round(8), prims::CapStyle::Round(8)],
        connect_subs: 4,
    };

    d.prim_context(w)
        .draw_lines_auto(&ls, dp, 0.0, false, false, None, c);
}

/// ###
/// ### Render drivers
/// ###

/// Clear background and draw the grid/axis
fn draw_ruler(d: &mut DrawData, c: &Context, w: &mut PistonWindow) {
    line::Line::new(d.theme[8], 1.0).draw(
        [d.hdims[0], d.dims[1], d.hdims[0], 0.0],
        &c.draw_state,
        c.transform,
        &mut d.draw_context(w),
    );

    let domain = (d.domain[1] - d.domain[0]) as f64;
    let guess = d.px_target / d.dims[1] * domain;

    let exp = guess.log(d.px_base);
    let floored = exp.floor();
    let sdom = d.px_base.powf(floored);

    let x = floored + 1.0 - exp;
    let scale = (3.0 - 2.0 * x) * x * x; // smooth step

    let center = d.domain[0] as f64 + domain * 0.5;
    draw_tack(center, 1.0, d, c, w);

    let mut at = center + sdom;
    let mut odd = true;

    while at < d.domain[1] as f64 {
        draw_tack(at, if odd { scale } else { 1.0 }, d, c, w);
        at += sdom;
        odd = odd == false;
    }

    let mut at = center - sdom;
    let mut odd = true;

    while at > d.domain[0] as f64 {
        draw_tack(at, if odd { scale } else { 1.0 }, d, c, w);
        at -= sdom;
        odd = odd == false;
    }
}

/// Generic modulator drawing
fn draw_modulator(
    key: &str,
    ordinal: &mut usize,
    col: usize,
    st: &mut StateData,
    d: &mut DrawData,
    c: &Context,
    w: &mut PistonWindow,
) {
    let m = if let Some(s) = get_modulator(key, st) {
        s
    } else {
        return;
    };

    if !m.enabled() {
        return;
    }

    let val = m.value();
    let gol = m.goal();

    if let Some(b) = st.bufs.get(key) {
        draw_buf(val, b, col, st, d, c, w);
    } else {
        draw_dot(val as f64, col, d, c, w);
    }

    draw_label(val as f64, col, d, c, w);
    draw_bracket(key, val, gol, m.range(), *ordinal, col, d, c, w);
    draw_anishape(val, *ordinal, col, d, c, w);

    *ordinal += 1;
}

/// ###
/// ### Color utilities
/// ###

/// Create and return a color theme table
fn color_theme() -> [types::Color; 14] {
    [
        prims::color_from(0xff_24_24_2e), // background
        prims::color_from(0xff_be_be_ef), // darkSPACE: main colors (5 shades, bright to dim)
        prims::color_from(0xff_86_86_cb),
        prims::color_from(0xff_72_72_a1),
        prims::color_from(0xff_5b_5b_7b),
        prims::color_from(0xff_49_49_5a),
        prims::color_from(0xff_fe_77_34), // darkSPACE: sub color (3 shades, bright to dim)
        prims::color_from(0xff_b0_68_45),
        prims::color_from(0xff_64_45_40),
        prims::color_from(0xff_dd_f8_dd), // darkFOREST: more colors (5 shades, bright to dim)
        prims::color_from(0xff_a9_bc_a9),
        prims::color_from(0xff_86_98_86),
        prims::color_from(0xff_73_82_73),
        prims::color_from(0xff_58_5f_58),
    ]
}

fn blended_theme_color(realindex: f32, d: &DrawData) -> types::Color {
    let i = realindex as usize;
    let j = if i < d.theme.len() - 1 { i + 1 } else { 1 };
    let f = realindex - (i as f32);

    prims::col_lerp(f, prims::as_vec4(d.theme[i]), prims::as_vec4(d.theme[j]))
}

/// ###
/// ### Other functions
/// ###

/// Advance time and update the earlier time, returns elapsed microseconds
fn time_delta(earlier: &mut Instant) -> u64 {
    let now = Instant::now();
    let dt = ModulatorEnv::<f32>::duration_to_micros(now.duration_since(*earlier));
    *earlier = now;

    dt
}

/// Update the simulation
fn update(e: &piston_window::Event, dt: u64, st: &mut StateData, d: &mut DrawData) {
    if st.paused == false {
        st.m1.advance(dt);
        st.m2.advance(dt);

        if st.curve_animate {
            st.m3.advance(dt);
        }

        // collect the samples of modulators that have buffers
        st.sampling_dt += dt;
        let wait_us = ((1.0 / st.buf_samples_per_sec as f32) * 1_000_000.0) as u64;

        if st.sampling_dt >= wait_us {
            let sample_count = (st.buf_samples_per_sec as f32 * st.buf_max_fade_time) as usize;

            for (key, buf) in &mut st.bufs {
                let md = if let Some(m) = st.m1.get(key) {
                    m
                } else if let Some(m) = st.m2.get(key) {
                    m
                } else {
                    continue;
                };

                if buf.len() >= sample_count {
                    let _ = buf.pop_back();
                }
                buf.push_front(md.value());
            }

            st.sampling_dt = 0;
        }
    }

    // Use amp_mod to modulate the amp of waveform2
    let ampmod = st.m2.value("amp_mod");
    if let Some(sw) = st.m2.get_mut("waveform2") {
        if let Some(ss) = sw.as_any().downcast_mut::<Wave>() {
            ss.amplitude = 1.0 + ampmod;
        }
    }

    if let Some(sw) = st.m1.get_mut("waveform") {
        if let Some(ss) = sw.as_any().downcast_mut::<Wave>() {
            ss.amplitude = 1.0 + ampmod * 0.25;
        }
    }

    if let Some(sw) = st.m1.get_mut("waveform") {
        if let Some(ss) = sw.as_any().downcast_mut::<Wave>() {
            ss.amplitude = 1.0 + ampmod * 0.25;
        }
    }

    let mods = st.m1.get_mods();
    d.domain = [-1.0, 1.0];

    for (_, v) in mods {
        if v.enabled() {
            let q = v.range();

            if q[0] < d.domain[0] {
                d.domain[0] = q[0];
            }
            if q[1] > d.domain[1] {
                d.domain[1] = q[1];
            }
        }
    }

    // We make the domain centered around 0 for now
    let abs_max = f32::max(d.domain[0].abs(), d.domain[1].abs());

    d.domain[0] = abs_max * -d.margin_scale;
    d.domain[1] = abs_max * d.margin_scale;

    // Cache the current mouse position
    if let Some(args) = e.mouse_cursor_args() {
        d.mouse_pos = vec2(args[0] as f32, args[1] as f32);
    }

    // Track the mouse pressed/released state
    if let Some(args) = e.button_args() {
        // If we wanted to track key presses, they would be received here as well
        if args.state == ButtonState::Press {
            st.tracking = true;
            st.tracking_from = d.mouse_pos;
        }
        if args.state == ButtonState::Release {
            st.tracking = false;
        }
    }

    // Track scrolling with the mouse wheel
    if let Some(args) = e.mouse_scroll_args() {
        st.mouse_scroll = vec2(args[0] as f32, args[1] as f32)
    }
    // Track entered text
    if let Some(args) = e.text_args() {
        st.entered_text = args
    }
}
