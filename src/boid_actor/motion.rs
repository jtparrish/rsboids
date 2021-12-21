use crate::kiss3d::nalgebra as na;
use super::{Boid, BoidDataMsg};
use super::{ViewConeLength, ViewConeFactor};
use crate::TIME_STEP;

impl Boid {
    fn convert_to_local(&self, b_data: &mut BoidDataMsg) {
        b_data.position -= self.position;
        b_data.velocity -= self.position;
    }

    fn in_view_cone(&self, local_bdata: &BoidDataMsg) -> bool {
        let local_pos = local_bdata.position;
        let view = self.velocity.normalize();
        let v_dist = local_pos * view;
        let v_offset = local_pos - (v_dist * view);
        let v_offset_d = v_offset.norm();
        
        v_dist < ViewConeLength && v_offset_d < v_dist * ViewConeFactor
    }

    fn world_accel(&self) -> na::Vector3<f32> {
        let heading = self.velocity.normalize();
        let bank_rot = na::geometry::Rotation3::from_axis_angle(heading, bank_angle);
        let local_y = na::Vector3::y().rotate(bank_rot);
        
        unimplemented!()
        
    }

    fn update(&mut self) {
        self.position += self.velocity * TIME_STEP;
        self.velocity += self.world_accel() * TIME_STEP;
    }
}

struct AveragePosTracker {
    sum: na::Vector3<f32>,
    n: usize,
}

impl AveragePosTracker {
    fn add_pos(&mut self, pos: na::Vector3<f32>) {
        self.sum += pos;
        n += 1;
    }

    fn get_avg(&self) -> na::Vector3<f32> {
        sum / n
    }

    fn reset(&mut self) {
        self.sum = na::Vector3::default();
        self.n = 0;
    }
}

struct BoidUpdate {
    centroid_trkr: AveragePosTracker,
}