use actix::prelude::*;
use kiss3d::scene::SceneNode;

pub mod messages;

const BOID_MODEL_PATH: &str = "boid_model";
const BOID_OBJ_FILE: &str = "vult2.obj";

pub struct WindowManager {
    window: kiss3d::window::Window,
    bounding_box: (f32, f32, f32),
    boid_models: Vec<SceneNode>,
}

impl WindowManager {
    pub fn new() -> Self {
        const X_BOUND: f32 = 100f32;
        const Y_BOUND: f32 = 100f32;
        const Z_BOUND: f32 = 100f32;

        WindowManager {
            window: kiss3d::window::Window::new("RS Boids"),
            bounding_box: (X_BOUND, Y_BOUND, Z_BOUND),
            boid_models: Vec::new(),
        }
    }
}

impl Actor for WindowManager {
    type Context = Context<Self>;
}