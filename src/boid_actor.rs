use actix::prelude::*;
use kiss3d::nalgebra as na;
use super::window_manager::WindowManager;

pub mod messages;

use messages::BoidDataMsg;

const ViewConeLength: f32 = 1.0;
const ViewConeFactor: f32 = 1.0;

pub struct Boid {
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    // positive bank angle corresponds to right roll
    bank_angle: f32,
    foward_accel: f32,
    local_accel: na::Vector3<f32>,
    flock: Option< Vec< Option<Addr<Boid>> > >,
    window_manager: Option< Addr<WindowManager> >,
    id: usize,
}

impl Boid {
    pub fn new(pos: na::Vector3<f32>, vel: na::Vector3<f32>) -> Self {
        Boid {
            position: pos,
            velocity: vel,
            bank_angle: 0f32,
            foward_accel: 0f32,
            local_accel: na::Vector3::default(),
            flock: None,
            window_manager: None,
            id: 0,
        }
    }

    fn observe_boid(&mut self, obs: BoidDataMsg) {
        unimplemented!();
    }
}

impl Actor for Boid {
    type Context = Context<Self>;
}