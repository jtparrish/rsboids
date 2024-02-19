use super::window_manager::WindowManager;
use actix::prelude::*;
use kiss3d::nalgebra as na;

pub mod messages;
mod motion;

use messages::BoidDataMsg;

const VIEW_CONE_LENGTH: f32 = 1.0;
const VIEW_CONE_FACTOR: f32 = 1.0;

pub struct Boid {
    position: na::Vector3<f32>,
    //velocity: na::Vector3<f32>,
    // positive bank angle corresponds to right roll
    //bank_angle: f32,
    //foward_accel: f32,
    //local_accel: na::Vector3<f32>,
    speed: f32,
    orientation: na::geometry::Rotation3<f32>,
    longitudinal_accel: f32,
    vertical_accel: f32,
    flock: Option<Vec<Option<Addr<Boid>>>>,
    window_manager: Option<Addr<WindowManager>>,
    update_trkr: motion::BoidUpdateTrkr,
    id: usize,
}

impl Boid {
    pub fn new(pos: na::Vector3<f32>, vel: na::Vector3<f32>) -> Self {
        let spd = vel.norm();
        let orient = {
            let x = vel.normalize();
            let y = na::Vector3::y();
            assert!(x.dot(&y) == 0f32);
            let z = x.cross(&y);

            na::geometry::Rotation3::from_basis_unchecked(&[x, y, z])
        };

        Boid {
            position: pos,
            speed: spd,
            orientation: orient,
            longitudinal_accel: 0f32,
            vertical_accel: 0f32,
            flock: None,
            window_manager: None,
            update_trkr: motion::BoidUpdateTrkr::new(),
            id: 0,
        }
    }

    fn reset(&mut self) {
        self.position = na::Vector3::zeros();
        let vel: na::Vector3<f32> = na::Vector3::x();
        let spd = vel.norm();
        let orient = {
            let x = vel.normalize();
            let y = na::Vector3::y();
            assert!(x.dot(&y) == 0f32);
            let z = x.cross(&y);

            na::geometry::Rotation3::from_basis_unchecked(&[x, y, z])
        };
        self.speed = spd;
        self.orientation = orient;
        self.longitudinal_accel = 0f32;
        self.vertical_accel = 0f32;
    }

    fn observe_boid(&mut self, obs: BoidDataMsg) {
        self.update_trkr.observe(obs);
    }
}

impl Actor for Boid {
    type Context = Context<Self>;
}
