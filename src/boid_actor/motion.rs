use super::{Boid, BoidDataMsg};
use super::{VIEW_CONE_FACTOR, VIEW_CONE_LENGTH};
use crate::consts;
use crate::TIME_STEP;
use kiss3d::nalgebra::{self as na, geometry};

const MAX_JERK: f32 = 0.02f32;

impl Boid {
    // TODO: consider saving this
    pub fn velocity(&self) -> na::Vector3<f32> {
        self.speed * self.orientation.transform_vector(&na::Vector3::x())
    }

    fn get_accel_request(&self) -> na::Vector3<f32> {
        let res = self.update_trkr.results();
        let [flk_cent_opt, flk_vel_opt, net_repuls_opt] = res;
        let flk_cent_frc = if let Some(flk_cent) = flk_cent_opt {
            todo!();
        } else {
            na::Vector3::zeros()
        };

        let vel_match_frc = if let Some(flk_vel) = flk_vel_opt {
            todo!();
        } else {
            na::Vector3::zeros()
        };

        let repulsion_frc = net_repuls_opt.expect("as this is a sum, it should never be None");

        let wall_force = wall_repulsion_force(self);
        let wall_force = -0.05f32 * na::Vector3::x();

        flk_cent_frc + vel_match_frc + repulsion_frc + wall_force
    }

    fn get_jerk(&self, req_accel: na::Vector3<f32>) -> na::Vector3<f32> {
        dbg!(req_accel);
        let current_accel = self.longitudinal_accel
            * self.orientation.transform_vector(&na::Vector3::x())
            + self.vertical_accel * self.orientation.transform_vector(&na::Vector3::y());

        dbg!(current_accel);

        let accel_diff = req_accel - current_accel;
        // TODO: consider something like a sigmoid here to determine the magnitude of the jerk
        // based off the magnitude of the acceleration difference
        let instant_correction_magnitude = accel_diff.norm() / TIME_STEP;
        let jerk_magnitude = if instant_correction_magnitude > MAX_JERK {
            MAX_JERK
        } else {
            instant_correction_magnitude
        };

        if accel_diff.norm() == 0f32 {
            na::Vector3::zeros()
        } else {
            jerk_magnitude * accel_diff.normalize()
        }
    }

    // shift things so this boid is stationary and at the origin (NO ROTATION APPLIED)
    fn convert_to_local(&self, b_data: &mut BoidDataMsg) {
        b_data.position -= self.position;
        b_data.velocity -= self.velocity();
    }

    fn in_view_cone(&self, local_bdata: &BoidDataMsg) -> bool {
        // position of the observed boid relative to the observer boid
        let local_pos = local_bdata.position;
        // the unit vector along the boid's view axis
        let view = self.orientation.transform_vector(&na::Vector3::x());
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
    //fn get_view_transform(&self) -> na::Rotation3<f32> {
    //    // NOTE: local x corresponds to the foward direction, local y corresponds to local up, and thus local z corresponds to right by RHR
    //    // NOTE: this concept of bank angle could be problematic if the boid reaches (+/-)90 degrees of pitch

    //    // the heading unit vector
    //    let heading = self.velocity.normalize();
    //    // the rotation of the bird along its longitudinal axis
    //    let bank_rot = na::geometry::Rotation3::from_axis_angle(&na::Unit::new_normalize(heading), self.bank_angle);
    //    // the bird's local vertical axis
    //    let local_y = {
    //        let rot_y = bank_rot.transform_vector( &na::Vector3::y() );
    //        rot_y - rot_y.dot(&heading) * heading
    //    };

    //    let view_transform = na::Rotation3::face_towards(&heading, &local_y);

    //    view_transform
    //}

    fn out_of_bounds_after(&self, delta_pos: na::Vector3<f32>) -> bool {
        use consts::geometry::{X_LEN, Y_LEN, Z_LEN};
        let new_pose = self.position + delta_pos;

        dbg!(new_pose);

        new_pose.x.abs() > X_LEN / 2f32
            || new_pose.y.abs() > Y_LEN / 2f32
            || new_pose.z.abs() > Z_LEN / 2f32
    }

    fn out_of_bounds(&self) -> bool {
        self.out_of_bounds_after(na::Vector3::zeros())
    }

    fn world_accel(&self) -> na::Vector3<f32> {
        self.longitudinal_accel * self.orientation.transform_vector(&na::Vector3::x())
            + self.vertical_accel * self.orientation.transform_vector(&na::Vector3::y())
    }

    fn perform_update(&mut self, jerk: na::Vector3<f32>) -> crate::window_messages::ModelUpdate {
        // the velocity before this update
        let old_velocity = self.velocity();
        // the accel before this update
        let old_accel = self.world_accel();
        dbg!(jerk);
        dbg!(old_velocity);
        dbg!(old_accel);
        dbg!(old_accel * TIME_STEP);
        dbg!(jerk * TIME_STEP);
        // the velocity after this update
        let new_velocity = old_velocity + (old_accel * TIME_STEP);
        // the acceleration after this update
        let new_accel = old_accel + (jerk * TIME_STEP);

        dbg!(new_velocity);
        dbg!(new_accel);

        // the longitudinal acceleration after this update
        let new_longitudinal_accel_vec =
            new_accel.dot(&new_velocity.normalize()) * new_velocity.normalize();
        dbg!(new_longitudinal_accel_vec);
        // the vertical acceleration after this update
        let new_vertical_accel_vec = new_accel - new_longitudinal_accel_vec;
        assert!(
            new_longitudinal_accel_vec
                .dot(&new_vertical_accel_vec)
                .abs()
                < 0.005
        );
        dbg!(new_vertical_accel_vec.norm());
        // the new orientation of the boid
        let new_orientation = {
            if new_vertical_accel_vec.norm() == 0f32 {
                if {
                    let parallel = old_velocity.cross(&new_velocity).norm() == 0f32;
                    let opposite = old_velocity.dot(&new_velocity) < 0f32;
                    parallel && opposite
                } {
                    eprintln!("REVERSING");
                    let x = -self.orientation.transform_vector(&na::Vector3::x());
                    let y = self.orientation.transform_vector(&na::Vector3::y());
                    let z = -self.orientation.transform_vector(&na::Vector3::z());
                    na::geometry::Rotation3::from_basis_unchecked(&[x, y, z])
                } else {
                    self.orientation
                        * na::Rotation3::rotation_between(&old_velocity, &new_velocity).unwrap()
                }
            } else {
                let x = new_velocity.normalize();
                let y = new_vertical_accel_vec.normalize();
                assert!(
                    x.dot(&y).abs() < 0.005,
                    "non-orthogonal: x = {}; y = {}; x.y = {}",
                    x,
                    y,
                    x.dot(&y)
                );
                let z = x.cross(&y);
                na::geometry::Rotation3::from_basis_unchecked(&[x, y, z])
            }
        };

        dbg!(self.orientation);
        dbg!(new_orientation);
        // the change in position of the boid
        let mut delta_position = old_velocity * TIME_STEP;
        // the change in orientation of the boid
        let mut delta_rot = self.orientation.rotation_to(&new_orientation);

        // If we are going out of bounds
        if self.out_of_bounds_after(delta_position) {
            // announce the reset
            println!("BOID RESET");
            // move the boid back to the origin
            delta_position = -self.position;
            // rotate the boid back to the axes (identity rotation)
            delta_rot = self.orientation.inverse();

            self.reset();
        } else {
            // update the boid data
            self.position += delta_position;
            self.speed = new_velocity.norm();
            // TODO: need to capture sign too
            self.longitudinal_accel = new_longitudinal_accel_vec.norm()
                * new_longitudinal_accel_vec.dot(&new_velocity).signum();
            // TODO: do we need to capture the sign? I don't thinks so b/c this vector is defined by it
            self.vertical_accel = new_vertical_accel_vec.norm();
            self.orientation = new_orientation;
        }

        dbg!(self.world_accel());

        dbg!(delta_position);
        dbg!(delta_rot);

        // return the model update for the given physics update
        crate::window_messages::ModelUpdate {
            id: self.id,
            translation: delta_position.into(),
            rot: delta_rot.into(),
        }
    }

    pub fn update(&mut self) -> crate::window_messages::ModelUpdate {
        let accel_request = self.get_accel_request();
        self.update_trkr.reset();
        let jerk = self.get_jerk(accel_request);

        self.perform_update(jerk)
    }
}

struct AverageVecTracker {
    sum: na::Vector3<f32>,
    n: usize,
}

impl AverageVecTracker {
    fn new() -> Self {
        AverageVecTracker {
            sum: na::Vector3::default(),
            n: 0usize,
        }
    }

    fn add_vec(&mut self, pos: na::Vector3<f32>) {
        self.sum += pos;
        self.n += 1;
    }

    fn get_avg(&self) -> Option<na::Vector3<f32>> {
        if self.n > 0 {
            Some(self.sum / (self.n as f32))
        } else {
            None
        }
    }

    fn get_sum(&self) -> na::Vector3<f32> {
        self.sum
    }

    fn reset(&mut self) {
        self.sum = na::Vector3::default();
        self.n = 0;
    }
}

pub struct BoidUpdateTrkr {
    centroid_trkr: AverageVecTracker,
    velocity_trkr: AverageVecTracker,
    repulsion_trkr: AverageVecTracker,
}

impl BoidUpdateTrkr {
    pub fn new() -> Self {
        BoidUpdateTrkr {
            centroid_trkr: AverageVecTracker::new(),
            velocity_trkr: AverageVecTracker::new(),
            repulsion_trkr: AverageVecTracker::new(),
        }
    }

    pub fn observe(&mut self, data: BoidDataMsg) {
        self.centroid_trkr.add_vec(data.position);
        self.velocity_trkr.add_vec(data.velocity);
        let repulsion_force = -data.position.normalize() * (1f32 / data.position.norm().powi(2));
        self.repulsion_trkr.add_vec(repulsion_force);
    }

    fn results(&self) -> [Option<na::Vector3<f32>>; 3] {
        let res = [
            self.centroid_trkr.get_avg(),
            self.velocity_trkr.get_avg(),
            Some(self.repulsion_trkr.get_sum()),
        ];

        res
    }

    fn reset(&mut self) {
        self.centroid_trkr.reset();
        self.velocity_trkr.reset();
        self.repulsion_trkr.reset();
    }
}

fn wall_repulsion_force(boid: &Boid) -> na::Vector3<f32> {
    let pose = boid.position;

    let walls = (0..6).map(|x| {
        let axis = x / 2;
        let dir_bit = x % 2;
        let dir = if dir_bit == 0 { -1 } else { 1 };

        (axis, dir)
    });

    let box_bounds = {
        use crate::consts::geometry;

        na::Vector3::new(geometry::X_LEN, geometry::Y_LEN, geometry::Z_LEN)
    };

    walls
        .map(|wall| {
            // TODO: need to half the bounds
            let (axis, dir) = wall;
            let dist = (dir as f32) * box_bounds[axis] - pose[axis];
            let magnitude = 1f32 / dist.powi(2);

            -1f32
                * dist.signum()
                * magnitude
                * na::Vector3::from_iterator((0..3).map(|x| if x == axis { 1f32 } else { 0f32 }))
        })
        .sum()
}

// TODO: remove me
#[allow(unreachable_code, dead_code, unused_variables)]
fn wall_turn_force(boid: &Boid) -> na::Vector3<f32> {
    use crate::consts::geometry;
    let box_dims: na::Vector3<f32> =
        na::Vector3::new(geometry::X_LEN, geometry::Y_LEN, geometry::Z_LEN);
    // velocity of the boid
    let velocity: na::Vector3<f32> = boid.velocity();
    let hdg: na::Vector3<f32> = velocity.normalize();
    let edges = (0..12).map(|x| {
        let pair_idx = x / 4;
        let orientation = x % 4;
        let mut coords = [1; 3];
        // pair_idx tells us which axis we don't use
        coords[pair_idx] = 0;
        for (i, x) in coords.iter_mut().filter(|&&mut x| x != 0).enumerate() {
            // unpack the direction_bit
            let direction_bit = if i == 0 { orientation % 2 } else { orientation / 2 };
            // 0 => negative; 1 => positive
            let direction = if direction_bit == 0 { -1 } else { 1 };
            *x *= direction;
        }
        coords
    });
    // we compute the rotation contribution for each edge
    let net_b_field = edges
        .map(|edge| {
            let edge_component: usize = {
                let singleton_lst: Vec<usize> = edge
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &x)| if x == 0 { Some(i) } else { None })
                    .collect();
                assert_eq!(singleton_lst.len(), 1);

                singleton_lst[0]
            };
            let edge_vec: na::Vector3<f32> =
                na::Vector3::from_iterator(edge.iter().map(|&x| x as f32).collect::<Vec<f32>>());
            // vector from origin to edge
            let to_edge_vec: na::Vector3<f32> = edge_vec.component_mul(&box_dims);
            // the "B field" for the edge
            let b_field = {
                // direction of field determined by whether boid is aimed left or right
                // of going straight towards the edge
                let sgn = {
                    let sgn = (to_edge_vec.cross(&velocity))[edge_component].signum();
                    let ret = if sgn == 0f32 { 1f32 } else { sgn };
                    ret
                };
                // the magnitude is determined by the distance to the edge
                let mag = {
                    let dist_to_edge: f32 = todo!();

                    1f32 / dist_to_edge.powi(2)
                };

                let basis_vec = {
                    let mut vec = na::Vector3::zeros();
                    vec[edge_component] = 1f32;

                    vec
                };

                sgn * mag * basis_vec
            };

            b_field
        })
        .sum();

    let mut force: na::Vector3<f32> = velocity.cross(&net_b_field);

    // if we are heading straight towards a wall (and perfectly centered) we need to account for this
    // (the rotations from the edges could cancel out)
    let walls = (0..6).map(|x| {
        let axis = x / 2;
        let dir_bit = x % 2;
        let dir = if dir_bit == 0 { -1 } else { 1 };

        let mut wall = [0; 3];
        wall[axis] = dir;

        wall
    });

    let wall_warnings = walls.filter_map(|wall| {
        // from boid to wall
        let vec_dist_to_wall: na::Vector3<f32> = todo!();
        // discrimination scalar (we divide by norm squared to get magnitude = 1/norm)
        let discr = velocity.dot(&(vec_dist_to_wall / vec_dist_to_wall.norm_squared()));
        // if the velocity doesn't move us towards the wall, we ignore
        if discr <= 0f32 {
            None
        } else {
            Some((discr, vec_dist_to_wall.normalize()))
        }
    });

    // TODO: might replace by if-let
    match wall_warnings
        .max_by(|&(discr1, vec1), &(discr2, vec2)| discr1.partial_cmp(&discr2).unwrap())
    {
        Some((discr, dir_wall)) if discr > 1f32 => {
            const LOW_FORCE: f32 = 1e-5;
            if force.norm() < LOW_FORCE {
                force = dir_wall.cross(&hdg).cross(&hdg);
                // if we are pointed directly at the wall, we perturb by some perpendicular vector
                if force.norm() == 0f32 {
                    use rand::Rng;
                    let mut rng = rand::thread_rng();
                    // TODO: generate random vector then take the perp component
                    let pert = {
                        const PERT_MAG: f32 = 1f32;
                        let rand =
                            na::Vector3::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>())
                                .normalize();
                        // get only the component perpendicular to the heading and scale to length
                        PERT_MAG * (rand - (rand.dot(&hdg) * hdg)).normalize()
                    };

                    force = dir_wall.cross(&(hdg + pert)).cross(&hdg);

                    // NOTE: random chance the vec is parallel to dir_wall but we let that slide
                }
            }

            // scale by the severity value
            force *= discr
        }
        _ => {}
    }

    unimplemented!();
}
