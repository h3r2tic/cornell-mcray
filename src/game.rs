use crate::car::*;
use crate::physics::{IntoPhysx, PhysicsState, PxMaterial, RigidDynamicHandle};
use dolly::prelude::*;
use kajiya::world_renderer::{AddMeshOptions, InstanceHandle, WorldRenderer};
use kajiya_simple::*;
use physx::prelude::*;

pub struct Game {
    physics: &'static mut PhysicsState,
    physics_material: Owner<PxMaterial>,

    car: Car,
    cube_mesh: kajiya::world_renderer::MeshHandle,
    lights_inst: InstanceHandle,
    occluder_inst: InstanceHandle,
    camera: CameraRig,

    keyboard: KeyboardState,
    keymap: KeyboardMap,

    dynamic_cubes: Vec<(InstanceHandle, RigidDynamicHandle, f32)>,
    global_time: f64,
    occluder_enabled: bool,
    occluder_t: f32,
}

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

impl Game {
    pub fn new(world_renderer: &mut WorldRenderer) -> anyhow::Result<Self> {
        // Can't be bothered to properly shut it down, and it crashes otherwise ¯\_(ツ)_/¯
        let physics = Box::leak(Box::new(PhysicsState::new()));

        // Default physics material
        let mut physics_material = physics
            .foundation
            .create_material(0.5, 0.5, 0.6, ())
            .unwrap();

        // Create track renderables
        let track_mesh_asset = kajiya::mmap::mmapped_asset::<
            kajiya::asset::mesh::PackedTriMesh::Flat,
            _,
        >("/cache/track.mesh")?;

        let env_mesh_gfx = world_renderer.add_mesh(track_mesh_asset, AddMeshOptions::default());
        world_renderer.add_instance(
            env_mesh_gfx,
            Affine3A::from_rotation_translation(Quat::IDENTITY, Vec3::ZERO),
        );

        // Create track physics meshes
        let track_mesh = physics.cook_mesh(track_mesh_asset);
        physics.add_static_mesh_actor(track_mesh, Affine3A::IDENTITY, physics_material.as_mut());

        // Create the dynamic occluder renderables (for party mode!)
        let occluder_mesh =
            world_renderer.add_baked_mesh("/cache/occluder.mesh", AddMeshOptions::default())?;
        let occluder_inst = world_renderer.add_instance(
            occluder_mesh,
            Affine3A::from_rotation_translation(Quat::IDENTITY, Vec3::ZERO),
        );

        // Create the spinning cube light renderables
        let lights_mesh =
            world_renderer.add_baked_mesh("/cache/lights.mesh", AddMeshOptions::default())?;
        let lights_inst = world_renderer.add_instance(
            lights_mesh,
            Affine3A::from_rotation_translation(Quat::IDENTITY, Vec3::ZERO),
        );

        // Create the car
        let car_meshes = CarMeshes::load(world_renderer)?;
        let car = Car::new(&car_meshes, physics, world_renderer, Vec3::ZERO);

        let keyboard: KeyboardState = Default::default();
        let keymap = KeyboardMap::new()
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
                KeyMap::new("nitro", 1.0).activation_time(0.3),
            );

        let camera = CameraRig::builder()
            .with(Position::new(kajiya_2_macaw_vec3(car.position)))
            .with(Rotation::new(kajiya_2_macaw_quat(car.rotation)))
            .with(Smooth::new_position(1.25).predictive(true))
            .with(Arm::new(macaw::Vec3::new(0.0, 1.5, -3.5)))
            .with(Smooth::new_position(2.5))
            .with(
                LookAt::new(kajiya_2_macaw_vec3(car.position + Vec3::Y))
                    .tracking_smoothness(1.25)
                    .tracking_predictive(true),
            )
            .build();

        let cube_mesh =
            world_renderer.add_baked_mesh("/cache/cube.mesh", AddMeshOptions::default())?;

        Ok(Self {
            physics,
            physics_material,
            car,
            cube_mesh,
            lights_inst,
            occluder_inst,
            camera,
            keyboard,
            keymap,
            dynamic_cubes: Vec::new(),
            global_time: 0.0,
            occluder_enabled: false,
            occluder_t: 0.0,
        })
    }

    fn process_events(&mut self, dt: f32, ctx: &mut FrameContext) {
        self.global_time += dt as f64;
        self.keyboard.update(ctx.events);

        if self.keyboard.was_just_pressed(VirtualKeyCode::Space) {
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

        if self.keyboard.was_just_pressed(VirtualKeyCode::Q) {
            self.occluder_enabled = !self.occluder_enabled;
        }

        if self.keyboard.is_down(VirtualKeyCode::B) {
            let spawn_pos = Vec3::new(fastrand::f32() - 0.5, 8.0, fastrand::f32() - 0.5);
            let spawn_rot = Quat::from_euler(
                EulerRot::YXZ,
                fastrand::f32() * std::f32::consts::PI,
                fastrand::f32() * std::f32::consts::PI,
                0.0,
            );

            let geo = PxBoxGeometry::new(1.0, 1.0, 1.0);
            let actor = self
                .physics
                .foundation
                .create_rigid_dynamic(
                    PxTransform::from_translation_rotation(
                        &spawn_pos.into_px(),
                        &spawn_rot.into_px(),
                    ),
                    &geo,
                    self.physics_material.as_mut(),
                    10.0,
                    PxTransform::default(),
                    (),
                )
                .unwrap();
            let actor = self.physics.add_dynamic_actor(actor);

            let inst = ctx.world_renderer.add_instance(
                self.cube_mesh,
                Affine3A::from_rotation_translation(spawn_rot, spawn_pos),
            );
            self.dynamic_cubes.push((inst, actor, 0.0));
        }

        let input = self.keymap.map(&self.keyboard, ctx.dt_filtered);
        self.car.update_motors(
            self.physics,
            input["thrust"] * 2.0f32.powf(input["nitro"]),
            -input["steering"],
        );
    }

    fn update_game_objects(&mut self, dt: f32, ctx: &mut FrameContext) {
        self.car
            .sync_transforms_from_physics(self.physics, ctx.world_renderer);

        // Update the dynamic occluder
        let occluder_target = if self.occluder_enabled { 1.0 } else { 0.0 };
        self.occluder_t = lerp(self.occluder_t, occluder_target, 0.01);
        ctx.world_renderer.set_instance_transform(
            self.occluder_inst,
            Affine3A::from_rotation_translation(
                Quat::IDENTITY,
                Vec3::new(0.0, (1.0 - self.occluder_t).powi(2) * -65.0, 0.0),
            ),
        );

        // Rotate the cube lights
        ctx.world_renderer.set_instance_transform(
            self.lights_inst,
            Affine3A::from_rotation_translation(
                Quat::from_rotation_y((self.global_time * 0.2 % std::f64::consts::TAU) as f32),
                Vec3::new(0.0, (1.0 - self.occluder_t) * 8.0, 0.0),
            ),
        );
        ctx.world_renderer
            .get_instance_dynamic_parameters_mut(self.lights_inst)
            .emissive_multiplier = ((self.occluder_t - 0.8) * 5.0).max(0.0).powi(3) * 20.0;

        // Age the dynamic cubes
        for (_, _, t) in &mut self.dynamic_cubes {
            *t += dt;
        }

        // Remove old cubes
        self.dynamic_cubes.retain(|(inst, rb, t)| {
            if *t >= 5.0 {
                ctx.world_renderer.remove_instance(*inst);
                self.physics.remove_dynamic_actor(*rb);
                false
            } else {
                true
            }
        });

        // Sync cube transforms
        for (inst, rb_handle, _) in &self.dynamic_cubes {
            let xform = self
                .physics
                .get_dynamic_actor_extrapolated_transform(*rb_handle)
                .unwrap();
            ctx.world_renderer.set_instance_transform(
                *inst,
                Affine3A::from_rotation_translation(macaw_2_kajiya_quat(xform.rotation()), macaw_2_kajiya_vec3(xform.translation())),
            );
        }
    }

    fn update_physics(&mut self, dt: f32) {
        self.physics.simulate(dt);
    }

    pub fn frame(&mut self, mut ctx: FrameContext) -> WorldFrameDesc {
        // Cap the max sim time so we don't skip too much time if the game's running slow
        let dt = ctx.dt_filtered.min(1.0 / 5.0);

        self.process_events(dt, &mut ctx);

        self.update_physics(dt);
        self.update_game_objects(dt, &mut ctx);

        self.camera.driver_mut::<Position>().position = kajiya_2_macaw_vec3(self.car.position);
        self.camera.driver_mut::<Rotation>().rotation = kajiya_2_macaw_quat(self.car.rotation);
        self.camera.driver_mut::<LookAt>().target = kajiya_2_macaw_vec3(self.car.position + Vec3::Y);

        let lens = CameraLens {
            aspect_ratio: ctx.aspect_ratio(),
            ..Default::default()
        };

        let _camera_matrices = self
            .camera
            .update(dt)
            .into_position_rotation();

        let camera_matrices = (macaw_2_kajiya_vec3(_camera_matrices.0), macaw_2_kajiya_quat(_camera_matrices.1)).through(&lens);

        WorldFrameDesc {
            camera_matrices,
            render_extent: ctx.render_extent,
            sun_direction: spherical_to_cartesian(-4.365, 1.3),
        }
    }
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
