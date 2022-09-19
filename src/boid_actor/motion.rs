use kiss3d::nalgebra as na;
use super::{Boid, BoidDataMsg};
use super::{VIEW_CONE_LENGTH, VIEW_CONE_FACTOR};
use crate::TIME_STEP;

impl Boid {
    // shift things so this boid is stationary and at the origin
    fn convert_to_local(&self, b_data: &mut BoidDataMsg) {
        b_data.position -= self.position;
        b_data.velocity -= self.velocity;
    }

    fn in_view_cone(&self, local_bdata: &BoidDataMsg) -> bool {
        // position of the observed boid relative to the observer boid
        let local_pos = local_bdata.position;
        // the unit vector along the boid's view axis
        let view = self.velocity.normalize();
        // the projection of the boid's position onto the view axis
        let v_dist = local_pos.dot(&view);
        // the vector from the observer boid to the projection point
        let v_offset = local_pos - (v_dist * view);
        // the magnitude of this vector from origin to projection
        let v_offset_d = v_offset.norm();
        
        // the observed boid must be within the view cone (not too far away from origin or the view axis)
        v_dist < VIEW_CONE_LENGTH && v_offset_d < v_dist * VIEW_CONE_FACTOR
    }

    // rotate from world coordinates to local coordinates (no translation applied)
    fn get_view_transform(&self) -> na::Rotation3<f32> {
        // NOTE: local z corresponds to the foward direction, local y corresponds to local up, and thus local x corresponds to left by RHR
        // NOTE: this concept of bank angle could be problematic if the boid reaches (+/-)90 degrees of pitch

        // the heading unit vector
        let heading = self.velocity.normalize();
        // the rotation of the bird along its longitudinal axis
        let bank_rot = na::geometry::Rotation3::from_axis_angle(&na::Unit::new_normalize(heading), self.bank_angle);
        // the bird's local vertical axis
        let local_y = {
            let rot_y = bank_rot.transform_vector( &na::Vector3::y() );
            rot_y - rot_y.dot(&heading) * heading
        };

        let view_transform = na::Rotation3::face_towards(&heading, &local_y);

        view_transform
    }

    fn world_accel(&self) -> na::Vector3<f32> {
        // NOTE: local z corresponds to the foward direction, local y corresponds to local up, and thus local x corresponds to left by RHR

        // the heading unit vector
        let heading = self.velocity.normalize();
        // the rotation of the bird along its longitudinal axis
        let bank_rot = na::geometry::Rotation3::from_axis_angle(&na::Unit::new_normalize(heading), self.bank_angle);
        // the bird's local vertical axis
        let local_y = {
            let rot_y = bank_rot.transform_vector( &na::Vector3::y() );
            rot_y - rot_y.dot(&heading) * heading
        };

        let view_transform = na::Rotation3::face_towards(&heading, &local_y);

        view_transform.inverse().transform_vector(&self.local_accel)
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
    fn new() -> Self {
        AveragePosTracker {
            sum: na::Vector3::default(),
            n: 0usize,
        }
    }

    fn add_pos(&mut self, pos: na::Vector3<f32>) {
        self.sum += pos;
        self.n += 1;
    }

    fn get_avg(&self) -> na::Vector3<f32> {
        self.sum / (self.n as f32)
    }

    fn reset(&mut self) {
        self.sum = na::Vector3::default();
        self.n = 0;
    }
}

pub struct BoidUpdate {
    centroid_trkr: AveragePosTracker,
}

impl BoidUpdate {
    pub fn new() -> Self {
        BoidUpdate {
            centroid_trkr: AveragePosTracker::new(),
        }
    }

    pub fn observe(&mut self, data: BoidDataMsg) {
        self.centroid_trkr.add_pos(data.position);
    }
}