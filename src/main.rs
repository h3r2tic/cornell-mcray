mod physics;

use std::{collections::HashMap, ffi::c_void};

use dolly::prelude::*;
use kajiya::world_renderer::{AddMeshOptions, InstanceHandle, WorldRenderer};
use kajiya_simple::*;
use physics::{FromPhysx, IntoPhysx, PhysicsState, RigidDynamicHandle};
use physx::{
    cooking::{PxCooking, PxCookingParams, PxTriangleMeshDesc},
    prelude::*,
    traits::Class,
};
use physx_sys::{PxMeshGeometryFlags, PxMeshScale};

struct CarMeshes {
    body: kajiya::world_renderer::MeshHandle,
    left_wheel: kajiya::world_renderer::MeshHandle,
    right_wheel: kajiya::world_renderer::MeshHandle,
}

impl CarMeshes {
    fn load(world_renderer: &mut WorldRenderer) -> anyhow::Result<Self> {
        let body = world_renderer.add_baked_mesh("/baked/car.mesh", AddMeshOptions::default())?;
        let left_wheel =
            world_renderer.add_baked_mesh("/baked/wheel_left.mesh", AddMeshOptions::default())?;
        let right_wheel =
            world_renderer.add_baked_mesh("/baked/wheel_right.mesh", AddMeshOptions::default())?;

        Ok(Self {
            body,
            left_wheel,
            right_wheel,
        })
    }
}

struct Car {
    gfx_meshes: HashMap<RigidDynamicHandle, InstanceHandle>,
    motors: Vec<*mut physx_sys::PxD6Joint>,
    steering: Vec<*mut physx_sys::PxD6Joint>,
    max_steering_angle: f32,
    suspension_height: f32,
    main_rb: RigidDynamicHandle,
    position: Vec3,
    rotation: Quat,
}

impl Car {
    fn update_motors(&mut self, _physics: &mut PhysicsState, thrust: f32, steering: f32) {
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

    fn sync_transforms_from_physics(
        &mut self,
        physics: &PhysicsState,
        world_renderer: &mut WorldRenderer,
    ) {
        for (rb_handle, inst) in &self.gfx_meshes {
            let rb = physics.get_dynamic_actor(*rb_handle).unwrap();

            let position: Vec3 = rb.get_global_position().new_from_px();
            let rotation: Quat = rb.get_global_rotation().new_from_px();

            world_renderer.set_instance_transform(*inst, position, rotation);

            if self.main_rb == *rb_handle {
                self.position = position;
                self.rotation = rotation;
            }
        }
    }
}

fn create_car(
    car_meshes: &CarMeshes,
    physics: &mut PhysicsState,
    world_renderer: &mut WorldRenderer,
    car_offset: Vec3,
) -> Car {
    let mut gfx_meshes = HashMap::new();

    let body_position_in_car_space = Vec3::new(0.0, 0.4739, 0.0);
    let body_position = body_position_in_car_space + car_offset;
    let body_gfx = world_renderer.add_instance(car_meshes.body, Vec3::ZERO, Quat::IDENTITY);

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

        let position = car_offset + position_in_car_space;
        let gfx = world_renderer.add_instance(mesh, position, Quat::IDENTITY);

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

            let drive_target =
                PxTransform::from_translation(&Vec3::new(0.0, -suspension_height, 0.0).into_px());
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

    Car {
        gfx_meshes,
        motors,
        steering,
        suspension_height,
        max_steering_angle,
        position: car_offset,
        rotation: Quat::IDENTITY,
        main_rb: body_rb,
    }
}

use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "cornell-mcray", about = "Cornell McRay t'Racing")]
struct Opt {
    #[structopt(long, default_value = "1920")]
    width: u32,

    #[structopt(long, default_value = "1080")]
    height: u32,

    #[structopt(long, default_value = "1.0")]
    temporal_upsampling: f32,

    #[structopt(long)]
    no_vsync: bool,

    #[structopt(long)]
    fullscreen: bool,
}

fn main() -> anyhow::Result<()> {
    // Can't be bothered to properly shut it down, and it crashes otherwise ¯\_(ツ)_/¯
    let physics = Box::leak(Box::new(PhysicsState::new()));
    let opt = Opt::from_args();

    // Point `kajiya` to standard assets and shaders
    set_standard_vfs_mount_points("../kajiya");

    // Game-specific assets
    set_vfs_mount_point("/baked", "./baked");

    let mut kajiya = SimpleMainLoop::builder()
        .vsync(!opt.no_vsync)
        .temporal_upsampling(opt.temporal_upsampling)
        .resolution([opt.width, opt.height])
        .fullscreen(opt.fullscreen.then(|| FullscreenMode::Exclusive))
        .build(
            WindowBuilder::new()
                .with_title("Cornell McRay t'Racer")
                .with_resizable(false),
        )?;

    kajiya.world_renderer.world_gi_scale = 7.0;

    let env_mesh_asset = kajiya::mmap::mmapped_asset::<kajiya::asset::mesh::PackedTriMesh::Flat, _>(
        "/baked/track.mesh",
    )?;
    let env_mesh_gfx = kajiya
        .world_renderer
        .add_mesh(env_mesh_asset, AddMeshOptions::default());
    kajiya
        .world_renderer
        .add_instance(env_mesh_gfx, Vec3::ZERO, Quat::IDENTITY);

    let cooking_params =
        PxCookingParams::new(physics.foundation.physics()).expect("PxCookingParams::new");
    let cooking = PxCooking::new(physics.foundation.foundation_mut(), &cooking_params)
        .expect("PxCooking::new");
    let mut env_mesh_desc = PxTriangleMeshDesc::new();
    env_mesh_desc.obj.triangles.data = env_mesh_asset.indices.as_ptr() as *const c_void;
    env_mesh_desc.obj.triangles.count = (env_mesh_asset.indices.len() / 3) as _;
    env_mesh_desc.obj.triangles.stride = (std::mem::size_of::<u32>() * 3) as _;
    env_mesh_desc.obj.points.data = env_mesh_asset.verts.as_ptr() as *const c_void;
    env_mesh_desc.obj.points.count = env_mesh_asset.verts.len() as _;
    env_mesh_desc.obj.points.stride = std::mem::size_of::<kajiya::asset::mesh::PackedVertex>() as _;
    let env_mesh = cooking.create_triangle_mesh(physics.foundation.physics_mut(), &env_mesh_desc);
    let mut env_mesh = match env_mesh {
        physx::cooking::TriangleMeshCookingResult::Success(env_mesh) => env_mesh,
        physx::cooking::TriangleMeshCookingResult::LargeTriangle => {
            panic!("LargeTriangle")
        }
        physx::cooking::TriangleMeshCookingResult::Failure => {
            panic!("Failure")
        }
        physx::cooking::TriangleMeshCookingResult::InvalidDescriptor => {
            panic!("InvalidDescriptor")
        }
    };

    // Create the ground
    let mut material = physics
        .foundation
        .create_material(0.5, 0.5, 0.6, ())
        .unwrap();

    /*let ground_plane = physics
    .foundation
    .create_plane(PxVec3::new(0.0, 1.0, 0.0), 0.0, material.as_mut(), ())
    .unwrap();*/
    //physics.scene.add_static_actor(ground_plane);

    let env_mesh_geo = PxTriangleMeshGeometry::new(
        &mut *env_mesh,
        &PxMeshScale {
            scale: physx_sys::PxVec3 {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            rotation: physx_sys::PxQuat {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        },
        PxMeshGeometryFlags { mBits: 0 },
    );
    let env_rigid = physics
        .foundation
        .create_rigid_static(
            PxTransform::default(),
            &env_mesh_geo,
            material.as_mut(),
            PxTransform::default(),
            (),
        )
        .unwrap();
    physics.scene.add_static_actor(env_rigid);

    let occluder_mesh = kajiya
        .world_renderer
        .add_baked_mesh("/baked/occluder.mesh", AddMeshOptions::default())?;
    let occluder_inst =
        kajiya
            .world_renderer
            .add_instance(occluder_mesh, Vec3::ZERO, Quat::IDENTITY);

    let lights_mesh = kajiya
        .world_renderer
        .add_baked_mesh("/baked/lights.mesh", AddMeshOptions::default())?;
    let lights_inst = kajiya
        .world_renderer
        .add_instance(lights_mesh, Vec3::ZERO, Quat::IDENTITY);

    let mut keyboard: KeyboardState = Default::default();
    let mut keymap = KeyboardMap::new()
        .bind(
            VirtualKeyCode::W,
            KeyMap::new("thrust", 1.0).activation_time(0.2),
        )
        .bind(
            VirtualKeyCode::S,
            KeyMap::new("thrust", -1.0).activation_time(0.2),
        )
        .bind(
            VirtualKeyCode::A,
            KeyMap::new("steering", -1.0).activation_time(0.15),
        )
        .bind(
            VirtualKeyCode::D,
            KeyMap::new("steering", 1.0).activation_time(0.15),
        )
        .bind(
            VirtualKeyCode::LShift,
            KeyMap::new("boost", 1.0).activation_time(0.3),
        );

    let car_meshes = CarMeshes::load(&mut kajiya.world_renderer)?;
    let mut car = create_car(
        &car_meshes,
        physics,
        &mut kajiya.world_renderer,
        Vec3::new(0.0, 0.0, 0.0),
    );

    let mut camera = CameraRig::builder()
        .with(Position::new(car.position))
        .with(Rotation::new(car.rotation))
        .with(Smooth::new_position(1.25).predictive(true))
        .with(Arm::new(Vec3::new(0.0, 1.5, -3.5)))
        .with(Smooth::new_position(2.5))
        .with(
            LookAt::new(car.position + Vec3::Y)
                .tracking_smoothness(1.25)
                .tracking_predictive(true),
        )
        .build();

    let lens = CameraLens {
        aspect_ratio: kajiya.window_aspect_ratio(),
        ..Default::default()
    };

    let cube_mesh = kajiya
        .world_renderer
        .add_baked_mesh("/baked/cube.mesh", AddMeshOptions::default())?;
    let mut dynamic_cubes: Vec<(InstanceHandle, RigidDynamicHandle, f32)> = Vec::new();

    //let mut sun_direction = spherical_to_cartesian(-4.54, 1.3);
    let sun_direction = spherical_to_cartesian(-4.365, 1.3);
    let mut global_time: f64 = 0.0;

    let mut occluder_enabled = false;
    let mut occluder_t = 0.0;

    kajiya.run(move |mut ctx| {
        global_time += ctx.dt_filtered as f64;
        keyboard.update(ctx.events);
        let input = keymap.map(&keyboard, ctx.dt_filtered);

        if keyboard.was_just_pressed(VirtualKeyCode::Space) {
            match ctx.world_renderer.render_mode {
                RenderMode::Standard => {
                    ctx.world_renderer.reset_reference_accumulation = true;
                    ctx.world_renderer.render_mode = RenderMode::Reference;
                }
                RenderMode::Reference => {
                    ctx.world_renderer.render_mode = RenderMode::Standard;
                }
            };
        }

        car.update_motors(
            physics,
            input["thrust"] * 2.0f32.powf(input["boost"]),
            -input["steering"],
        );

        // TODO: run this at fixed rate
        physics
            .scene
            .step(
                1.0 / 60.0,
                None::<&mut physx_sys::PxBaseTask>,
                Some(unsafe { &mut ScratchBuffer::new(4) }),
                true,
            )
            .expect("error occured during simulation");

        car.sync_transforms_from_physics(physics, ctx.world_renderer);

        if keyboard.was_just_pressed(VirtualKeyCode::Q) {
            occluder_enabled = !occluder_enabled;
        }

        if keyboard.is_down(VirtualKeyCode::B) {
            let spawn_pos = Vec3::new(fastrand::f32() - 0.5, 8.0, fastrand::f32() - 0.5);
            let spawn_rot = Quat::from_euler(
                EulerRot::YXZ,
                fastrand::f32() * std::f32::consts::PI,
                fastrand::f32() * std::f32::consts::PI,
                0.0,
            );

            let geo = PxBoxGeometry::new(1.0, 1.0, 1.0);
            let actor = physics
                .foundation
                .create_rigid_dynamic(
                    PxTransform::from_translation_rotation(
                        &spawn_pos.into_px(),
                        &spawn_rot.into_px(),
                    ),
                    &geo,
                    material.as_mut(),
                    10.0,
                    PxTransform::default(),
                    (),
                )
                .unwrap();
            let actor = physics.add_dynamic_actor(actor);

            let inst = ctx
                .world_renderer
                .add_instance(cube_mesh, spawn_pos, spawn_rot);
            dynamic_cubes.push((inst, actor, 0.0));
        }

        let occluder_target = if occluder_enabled { 1.0 } else { 0.0 };
        occluder_t = lerp(occluder_t, occluder_target, 0.01);

        ctx.world_renderer.set_instance_transform(
            lights_inst,
            Vec3::new(0.0, (1.0 - occluder_t) * 8.0, 0.0),
            Quat::from_rotation_y((global_time * 0.2 % std::f64::consts::TAU) as f32),
        );
        ctx.world_renderer
            .get_instance_dynamic_parameters_mut(lights_inst)
            .emissive_multiplier = ((occluder_t - 0.8) * 5.0).max(0.0).powi(3) * 20.0;
        //ctx.world_renderer.sun_size_multiplier = 0.0;

        ctx.world_renderer.set_instance_transform(
            occluder_inst,
            Vec3::new(0.0, (1.0 - occluder_t).powi(2) * -65.0, 0.0),
            Quat::IDENTITY,
        );

        for (_, _, t) in &mut dynamic_cubes {
            *t += ctx.dt_filtered;
        }

        dynamic_cubes.retain(|(inst, rb, t)| {
            if *t >= 5.0 {
                ctx.world_renderer.remove_instance(*inst);
                physics.remove_dynamic_actor(*rb);
                false
            } else {
                true
            }
        });

        for (inst, rb_handle, _) in &dynamic_cubes {
            let rb = physics.get_dynamic_actor(*rb_handle).unwrap();
            let position: Vec3 = rb.get_global_position().new_from_px();
            let rotation: Quat = rb.get_global_rotation().new_from_px();
            ctx.world_renderer
                .set_instance_transform(*inst, position, rotation);
        }

        camera.driver_mut::<Position>().position = car.position;
        camera.driver_mut::<Rotation>().rotation = car.rotation;
        camera.driver_mut::<LookAt>().target = car.position + Vec3::Y;

        let camera_matrices = camera
            .update(ctx.dt_filtered)
            .into_position_rotation()
            .through(&lens);

        WorldFrameDesc {
            camera_matrices,
            render_extent: ctx.render_extent,
            sun_direction,
        }
    })
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn spherical_to_cartesian(theta: f32, phi: f32) -> Vec3 {
    let x = phi.sin() * theta.cos();
    let y = phi.cos();
    let z = phi.sin() * theta.sin();
    Vec3::new(x, y, z)
}
