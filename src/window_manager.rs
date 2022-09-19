use actix::prelude::*;
use kiss3d::scene::SceneNode;
use kiss3d::nalgebra as na;

pub mod messages;

const BOID_MODEL_PATH: &str = "boid_model";
const BOID_OBJ_FILE: &str = "vult2.obj";

pub struct WindowManager {
    window: kiss3d::window::Window,
    bounding_box: (f32, f32, f32),
    box_geometry: Vec<SceneNode>,
    boid_models: Vec<SceneNode>,
}

impl WindowManager {
    pub fn new() -> Self {
        const X_BOUND: f32 = 100f32;
        const Y_BOUND: f32 = 100f32;
        const Z_BOUND: f32 = 100f32;

        let mut win = kiss3d::window::Window::new("RS Boids");
        let mut box_geometry: Vec<SceneNode> = Vec::new();
        let crnr_signs = [(1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1), (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)];
        for i in 0..8 {
            let signs = crnr_signs[i];
            let coords = ((signs.0 as f32) * X_BOUND / 2f32, (signs.1 as f32) * Y_BOUND / 2f32, (signs.2 as f32) * Z_BOUND / 2f32);
            let trans = na::Vector3::new(coords.0, coords.1, coords.2);
            let trans = na::Translation3::<f32>::from_vector(trans);
            let mut sphere = win.add_sphere(1f32);
            sphere.append_translation(&trans);
            box_geometry.push(sphere);
        }

        let lengths = [X_BOUND, Y_BOUND, Z_BOUND];
        let vector = na::Vector3::new(lengths[0] / 2f32, lengths[1] / 2f32, lengths[2] / 2f32);
        let offsets = [
            (vector.y * na::Vector3::y(), vector.z * na::Vector3::z()),
            (vector.x * na::Vector3::x(), vector.z * na::Vector3::z()),
            (vector.x * na::Vector3::x(), vector.y * na::Vector3::y()),
        ];
        let rotations = [
            na::Rotation3::from_axis_angle( &na::Unit::new_normalize( na::Vector3::z() ), -std::f32::consts::FRAC_PI_2),
            na::Rotation3::from_axis_angle( &na::Unit::new_normalize( na::Vector3::y() ), 0f32),
            na::Rotation3::from_axis_angle( &na::Unit::new_normalize( na::Vector3::x() ), std::f32::consts::FRAC_PI_2),
        ];
        for i in 0..3 {
            let length = lengths[i];
            let offsets = offsets[i];
            let rotation = rotations[i];
            
            for j in 0..4 {
                let signs = (2 * (j % 2) - 1, 2 * (j / 2) - 1);
                let mut edge = win.add_cylinder(0.3f32, length);
                let trans = na::Vector3::new(0f32, 0f32, 0f32) + (signs.0 as f32) * offsets.0 + (signs.1 as f32) * offsets.1;
                let trans = na::Translation3::<f32>::from_vector(trans);
                edge.append_rotation(&rotation.into());
                edge.append_translation(&trans);
                box_geometry.push(edge);
            }
        }

        WindowManager {
            window: win,
            bounding_box: (X_BOUND, Y_BOUND, Z_BOUND),
            box_geometry,
            boid_models: Vec::new(),
        }
    }
}

impl Actor for WindowManager {
    type Context = Context<Self>;
}