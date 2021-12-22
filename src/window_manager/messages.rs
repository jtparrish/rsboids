use kiss3d::nalgebra as na;
use std::path::Path;
// use actix::prelude::*;
// use super::WindowManager;
// use super::{BOID_MODEL_PATH, BOID_OBJ_FILE};
use super::*;
use crate::boid_actor::messages as boid_messages;

#[derive(Message)]
#[rtype(result = "()")]
pub struct Render;

impl Handler<Render> for WindowManager {
    type Result = ();

    fn handle(&mut self, msg: Render, _ctx: &mut Self::Context) -> () {
        println!("here");
        if !self.window.render() {
            // println!("leaving");
            System::current().stop();
        } //else {
        //     println!("continuing");
        // }
    }
}

#[derive(Message)]
#[rtype(result = "usize")]
pub struct RegisterBoid(pub boid_messages::BoidDataMsg);

impl Handler<RegisterBoid> for WindowManager {
    type Result = usize;

    fn handle(&mut self, msg: RegisterBoid, _ctx: &mut Self::Context) -> usize {
        let boid_model_path = Path::new(BOID_MODEL_PATH);
        let boid_obj_path = boid_model_path.join(Path::new(BOID_OBJ_FILE));
        let boid_id = self.boid_models.len();
        self.boid_models.push(
            self.window.add_obj(&boid_obj_path, boid_model_path, na::Vector3::new(1f32, 1f32, 1f32))
        );

        boid_id
    }
}