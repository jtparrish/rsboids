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
        // println!("here");
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
        self.boid_models.last_mut().expect("how?").append_translation( &na::Translation3::from_vector(
            na::Vector3::new((2f32 * (boid_id as f32) + 3f32) * ((2 * ((boid_id % 2) as i32) - 1) as f32), 0f32, -4f32 * ((boid_id) as f32))) );

        boid_id
    }
}

#[derive(Message)]
#[rtype(result = "()")]
pub struct ModelUpdate{
    pub id: usize,
    pub translation: na::Translation3<f32>,
    pub rot: na::UnitQuaternion<f32>,
}

impl Handler<ModelUpdate> for WindowManager {
    type Result = ();

    fn handle(&mut self, msg: ModelUpdate, _ctx: &mut Self::Context) {
        let model = &mut self.boid_models[msg.id];
        model.append_translation(&msg.translation);
        model.append_rotation(&msg.rot);
    }
}