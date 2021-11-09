extern crate kiss3d;
use kiss3d::nalgebra as na;

use std::path::Path;

use na::{Vector3, UnitQuaternion, Translation3};
use kiss3d::window::Window;
use kiss3d::light::Light;

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    // let mut c      = window.add_cube(1.0, 1.0, 1.0);
    let mut boid = window.add_obj(Path::new("vult4.obj"), Path::new("."), Vector3::new(1f32, 1f32, 1f32));

    // boid.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);
    

    let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.014);

    while window.render() {
        // boid.prepend_to_local_rotation(&rot);
        boid.prepend_to_local_translation(&Translation3::new(0f32, 0.01f32, 0f32));
    }
}
