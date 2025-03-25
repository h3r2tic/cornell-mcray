use std::collections::HashMap;

use crate::physics::{IntoPhysx, PhysicsState, RigidDynamicHandle};
use kajiya::world_renderer::{AddMeshOptions, InstanceHandle, WorldRenderer};
use kajiya_simple::*;
use physx::{prelude::*, traits::Class};

fn kajiya_2_macaw_vec3(a: kajiya_simple::Vec3) -> macaw::Vec3 {
	return macaw::Vec3::new(a.x, a.y, a.z);
}

fn macaw_2_kajiya_vec3(a: macaw::Vec3) -> kajiya_simple::Vec3 {
	return kajiya_simple::Vec3::new(a.x, a.y, a.z);
}

fn kajiya_2_macaw_quat(a: kajiya_simple::Quat) -> macaw::Quat {
	return macaw::Quat::from_xyzw(a.x, a.y, a.z, a.w);
}

fn macaw_2_kajiya_quat(a: macaw::Quat) -> kajiya_simple::Quat {
	return kajiya_simple::Quat::from_xyzw(a.x, a.y, a.z, a.w);
}

pub struct CarMeshes {
    body: kajiya::world_renderer::MeshHandle,
    left_wheel: kajiya::world_renderer::MeshHandle,
    right_wheel: kajiya::world_renderer::MeshHandle,
}

impl CarMeshes {
    pub fn load(world_renderer: &mut WorldRenderer) -> anyhow::Result<Self> {
        let body = world_renderer.add_baked_mesh("/cache/car.mesh", AddMeshOptions::default())?;
        let left_wheel =
            world_renderer.add_baked_mesh("/cache/wheel_left.mesh", AddMeshOptions::default())?;
        let right_wheel =
            world_renderer.add_baked_mesh("/cache/wheel_right.mesh", AddMeshOptions::default())?;

        Ok(Self {
            body,
            left_wheel,
            right_wheel,
        })
    }
}

pub struct Car {
    pub position: Vec3,
    pub rotation: Quat,

    gfx_meshes: HashMap<RigidDynamicHandle, InstanceHandle>,
    motors: Vec<*mut physx_sys::PxD6Joint>,
    steering: Vec<*mut physx_sys::PxD6Joint>,
    max_steering_angle: f32,
    suspension_height: f32,
    main_rb: RigidDynamicHandle,
}

impl Car {
    pub fn update_motors(&mut self, _physics: &mut PhysicsState, thrust: f32, steering: f32) {
        for motor in self.steering.iter().copied() {
            unsafe {
                physx_sys::PxD6Joint_setDrivePosition_mut(
                    motor,
                    PxTransform::from_translation_rotation(
                        &Vec3::new(0.0, -self.suspension_height, 0.0).into_px(),
                        &Quat::from_rotation_y(self.max_steering_angle * steering).into_px(),
                    )
                    .as_ptr(),
                    true,
                );
            }
        }

        // Pseudo-differential adjusting speed of engines depending on steering arc
        // Higher values result in more drifty behavior.
        let differential_strength = 2.0;
        let sideways_shift = (self.max_steering_angle * steering).sin() * differential_strength;
        let speed_diff = if sideways_shift > 0.0 {
            f32::hypot(1.0, sideways_shift)
        } else {
            1.0 / f32::hypot(1.0, sideways_shift)
        };

        let ms = [1.0 / speed_diff, speed_diff];
        for (motor, &ms) in self.motors.iter().copied().zip(ms.iter()) {
            unsafe {
                physx_sys::PxD6Joint_setDriveVelocity_mut(
                    motor,
                    &physx_sys::PxVec3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    &physx_sys::PxVec3 {
                        x: -30.0 * thrust * ms,
                        y: 0.0,
                        z: 0.0,
                    },
                    true,
                );
            }
        }
    }

    pub fn sync_transforms_from_physics(
        &mut self,
        physics: &PhysicsState,
        world_renderer: &mut WorldRenderer,
    ) {
        for (rb_handle, inst) in &self.gfx_meshes {
            let xform = physics
                .get_dynamic_actor_extrapolated_transform(*rb_handle)
                .unwrap();

            world_renderer.set_instance_transform(
                *inst,
                Affine3A::from_rotation_translation(macaw_2_kajiya_quat(xform.rotation()), macaw_2_kajiya_vec3(xform.translation())),
            );

            if self.main_rb == *rb_handle {
                self.position = macaw_2_kajiya_vec3(xform.translation());
                self.rotation = macaw_2_kajiya_quat(xform.rotation());
            }
        }
    }

    pub fn new(
        car_meshes: &CarMeshes,
        physics: &mut PhysicsState,
        world_renderer: &mut WorldRenderer,
        car_position: Vec3,
    ) -> Self {
        let mut gfx_meshes = HashMap::new();

        let body_position_in_car_space = Vec3::new(0.0, 0.4739, 0.0);
        let body_position = body_position_in_car_space + car_position;
        let body_gfx = world_renderer.add_instance(
            car_meshes.body,
            Affine3A::from_rotation_translation(Quat::IDENTITY, Vec3::ZERO),
        );

        let mut material = physics
            .foundation
            .create_material(0.8, 0.8, 0.6, ())
            .unwrap();

        let wheel_params = vec![
            (car_meshes.left_wheel, Vec3::new(0.6874, 0.2783, -0.7802)),
            (car_meshes.right_wheel, Vec3::new(-0.6874, 0.2783, -0.7802)),
            (car_meshes.left_wheel, Vec3::new(0.64, 0.2783, 1.0254)),
            (car_meshes.right_wheel, Vec3::new(-0.64, 0.2783, 1.0254)),
        ];

        let mut body_center_of_mass: Vec3 = wheel_params
            .iter()
            .map(|p| p.1)
            .fold(Vec3::ZERO, |a, b| a + b)
            / (wheel_params.len() as f32);
        body_center_of_mass.y = 0.0;

        let body_geo = PxBoxGeometry::new(0.65, 0.3, 0.9);
        let mut body_actor = physics
            .foundation
            .create_rigid_dynamic(
                PxTransform::from_translation(&body_position.into_px()),
                &body_geo,
                material.as_mut(),
                1000.0,
                PxTransform::default(),
                (),
            )
            .unwrap();
        body_actor.set_c_mass_local_pose(&PxTransform::from_translation(
            &body_center_of_mass.into_px(),
        ));

        let max_steering_angle = 35.0f32.to_radians();
        let suspension_strength = 0.4;
        let suspension_height = 0.12;
        let drive_strength = 2.0;

        let mut motors = vec![];
        let mut steering = vec![];

        for (wheel_idx, (mesh, position_in_car_space)) in wheel_params.into_iter().enumerate() {
            let is_front = wheel_idx >= 2;

            let position = car_position + position_in_car_space;
            let gfx = world_renderer.add_instance(
                mesh,
                Affine3A::from_rotation_translation(Quat::IDENTITY, position),
            );

            let axle_geo = PxSphereGeometry::new(0.28);
            let mut axle_actor = physics
                .foundation
                .create_rigid_dynamic(
                    PxTransform::from_translation(&position.into_px()),
                    &axle_geo,
                    material.as_mut(),
                    1000.0,
                    PxTransform::default(),
                    (),
                )
                .unwrap();
            axle_actor.get_shapes_mut()[0].set_flag(ShapeFlag::SimulationShape, false);

            let wheel_geo = PxSphereGeometry::new(0.28);
            let mut wheel_actor = physics
                .foundation
                .create_rigid_dynamic(
                    PxTransform::from_translation(&position.into_px()),
                    &wheel_geo,
                    material.as_mut(),
                    1000.0,
                    PxTransform::default(),
                    (),
                )
                .unwrap();

            // HACK: Disable collision between the body and the wheell by creating a dummy joint
            unsafe {
                let local_frame = PxTransform::default();

                let joint = physx_sys::phys_PxD6JointCreate(
                    physics.foundation.physics_mut().as_mut_ptr(),
                    body_actor.as_mut_ptr(),
                    local_frame.as_ptr(),
                    wheel_actor.as_mut_ptr(),
                    local_frame.as_ptr(),
                );

                for axis in 0..physx_sys::PxD6Axis::eCOUNT {
                    physx_sys::PxD6Joint_setMotion_mut(joint, axis, physx_sys::PxD6Motion::eFREE);
                }

                physx_sys::PxJoint_setConstraintFlag_mut(
                    joint as *mut physx_sys::PxJoint,
                    physx_sys::PxConstraintFlag::eCOLLISION_ENABLED,
                    false,
                );
            }

            let suspension_attachment_in_body_space: Vec3 =
                position_in_car_space - body_position_in_car_space;

            // Create suspension between the body and the axle
            unsafe {
                let local_frame0 =
                    PxTransform::from_translation(&suspension_attachment_in_body_space.into_px());
                let local_frame1 = PxTransform::default();

                let joint = physx_sys::phys_PxD6JointCreate(
                    physics.foundation.physics_mut().as_mut_ptr(),
                    body_actor.as_mut_ptr(),
                    local_frame0.as_ptr(),
                    axle_actor.as_mut_ptr(),
                    local_frame1.as_ptr(),
                );
                physx_sys::PxD6Joint_setMotion_mut(
                    joint,
                    physx_sys::PxD6Axis::eY,
                    physx_sys::PxD6Motion::eLIMITED,
                );

                physx_sys::PxD6Joint_setLinearLimit_mut(
                    joint,
                    &physx_sys::PxJointLinearLimit_new_1(
                        suspension_height,
                        &physx_sys::PxSpring_new(1e5, 1e4),
                    ),
                );

                if is_front {
                    physx_sys::PxD6Joint_setMotion_mut(
                        joint,
                        physx_sys::PxD6Axis::eSWING1, // Y axis
                        physx_sys::PxD6Motion::eLIMITED,
                    );

                    physx_sys::PxD6Joint_setSwingLimit_mut(
                        joint,
                        &physx_sys::PxJointLimitCone_new_1(
                            max_steering_angle,
                            0.0,
                            &physx_sys::PxSpring_new(1e5, 1e4),
                        ),
                    );

                    // Steering
                    physx_sys::PxD6Joint_setDrive_mut(
                        joint,
                        physx_sys::PxD6Drive::eSWING,
                        &physx_sys::PxD6JointDrive_new_1(
                            drive_strength * 100.0,
                            drive_strength * 10.0,
                            drive_strength * 1000.0,
                            true,
                        ),
                    );

                    steering.push(joint);
                }

                physx_sys::PxD6Joint_setDrive_mut(
                    joint,
                    physx_sys::PxD6Drive::eY,
                    &physx_sys::PxD6JointDrive_new_1(
                        suspension_strength * 2000.0,
                        suspension_strength * 50.0,
                        1e5,
                        true,
                    ),
                );

                let drive_target = PxTransform::from_translation(
                    &Vec3::new(0.0, -suspension_height, 0.0).into_px(),
                );
                physx_sys::PxD6Joint_setDrivePosition_mut(joint, drive_target.as_ptr(), true);

                physx_sys::PxJoint_setConstraintFlag_mut(
                    joint as *mut physx_sys::PxJoint,
                    physx_sys::PxConstraintFlag::eCOLLISION_ENABLED,
                    false,
                );
            }

            // Create the motor
            unsafe {
                let local_frame = PxTransform::default();

                let joint = physx_sys::phys_PxD6JointCreate(
                    physics.foundation.physics_mut().as_mut_ptr(),
                    axle_actor.as_mut_ptr(),
                    local_frame.as_ptr(),
                    wheel_actor.as_mut_ptr(),
                    local_frame.as_ptr(),
                );

                physx_sys::PxD6Joint_setMotion_mut(
                    joint,
                    physx_sys::PxD6Axis::eTWIST,
                    physx_sys::PxD6Motion::eFREE,
                );

                physx_sys::PxJoint_setConstraintFlag_mut(
                    joint as *mut physx_sys::PxJoint,
                    physx_sys::PxConstraintFlag::eCOLLISION_ENABLED,
                    false,
                );

                if !is_front {
                    physx_sys::PxD6Joint_setDrive_mut(
                        joint,
                        physx_sys::PxD6Drive::eTWIST,
                        &physx_sys::PxD6JointDrive_new_1(0.0, 100.0, 1000.0, true),
                    );

                    motors.push(joint);
                }
            }

            let wheel_rb = physics.add_dynamic_actor(wheel_actor);
            physics.add_dynamic_actor(axle_actor);
            gfx_meshes.insert(wheel_rb, gfx);
        }

        let body_rb = physics.add_dynamic_actor(body_actor);
        gfx_meshes.insert(body_rb, body_gfx);

        Self {
            gfx_meshes,
            motors,
            steering,
            suspension_height,
            max_steering_angle,
            position: car_position,
            rotation: Quat::IDENTITY,
            main_rb: body_rb,
        }
    }
}
