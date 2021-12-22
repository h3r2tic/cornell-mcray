use std::ffi::c_void;

use kajiya_simple::*;
use macaw::IsoTransform;
use physx::{
    cooking::{PxCooking, PxCookingParams, PxTriangleMeshDesc},
    prelude::*,
    traits::Class,
};
use physx_sys::{PxMeshGeometryFlags, PxMeshScale};
use slotmap::{new_key_type, SlotMap};

/// Many of the main types in PhysX have a userData *mut c_void field.
/// Representing this safely in Rust requires generics everywhere,
/// and pre-defining all the generic parameters makes things more usable.
pub type PxMaterial = physx::material::PxMaterial<()>;
pub type PxShape = physx::shape::PxShape<(), PxMaterial>;
pub type PxArticulationLink = physx::articulation_link::PxArticulationLink<(), PxShape>;
pub type PxRigidStatic = physx::rigid_static::PxRigidStatic<(), PxShape>;
pub type PxRigidDynamic = physx::rigid_dynamic::PxRigidDynamic<(), PxShape>;
pub type PxArticulation = physx::articulation::PxArticulation<(), PxArticulationLink>;
pub type PxArticulationReducedCoordinate =
    physx::articulation_reduced_coordinate::PxArticulationReducedCoordinate<(), PxArticulationLink>;
pub type PxScene = physx::scene::PxScene<
    (),
    PxArticulationLink,
    PxRigidStatic,
    PxRigidDynamic,
    PxArticulation,
    PxArticulationReducedCoordinate,
    OnCollision,
    OnTrigger,
    OnConstraintBreak,
    OnWakeSleep,
    OnAdvance,
>;

/// Next up, the simulation event callbacks need to be defined, and possibly an
/// allocator callback as well.
pub struct OnCollision;
impl CollisionCallback for OnCollision {
    fn on_collision(
        &mut self,
        _header: &physx_sys::PxContactPairHeader,
        _pairs: &[physx_sys::PxContactPair],
    ) {
    }
}
pub struct OnTrigger;
impl TriggerCallback for OnTrigger {
    fn on_trigger(&mut self, _pairs: &[physx_sys::PxTriggerPair]) {}
}

pub struct OnConstraintBreak;
impl ConstraintBreakCallback for OnConstraintBreak {
    fn on_constraint_break(&mut self, _constraints: &[physx_sys::PxConstraintInfo]) {}
}
pub struct OnWakeSleep;
impl WakeSleepCallback<PxArticulationLink, PxRigidStatic, PxRigidDynamic> for OnWakeSleep {
    fn on_wake_sleep(
        &mut self,
        _actors: &[&physx::actor::ActorMap<PxArticulationLink, PxRigidStatic, PxRigidDynamic>],
        _is_waking: bool,
    ) {
    }
}
pub struct OnAdvance;
impl AdvanceCallback<PxArticulationLink, PxRigidDynamic> for OnAdvance {
    fn on_advance(
        &self,
        _actors: &[&physx::rigid_body::RigidBodyMap<PxArticulationLink, PxRigidDynamic>],
        _transforms: &[PxTransform],
    ) {
    }
}

new_key_type! {
    pub struct RigidDynamicHandle;
    pub struct TriangleMeshHandle;
    pub struct TriangleMeshGeometryHandle;
}

#[derive(Default)]
struct Resources {
    dynamic_actor: SlotMap<RigidDynamicHandle, *mut physx_sys::PxRigidDynamic>,
    triangle_mesh: SlotMap<TriangleMeshHandle, Owner<physx::triangle_mesh::TriangleMesh>>,
}

pub struct PhysicsState {
    pub foundation: PhysicsFoundation<physx::foundation::DefaultAllocator, PxShape>,
    pub scene: Owner<PxScene>,
    resources: Resources,
}

impl PhysicsState {
    pub fn new() -> Self {
        let mut foundation: PhysicsFoundation<physx::foundation::DefaultAllocator, PxShape> =
            Default::default();

        // Setup the scene object.  The PxScene type alias makes this much cleaner.
        // There are lots of unwrap calls due to potential null pointers.
        let scene: Owner<PxScene> = foundation
            .create(SceneDescriptor {
                gravity: PxVec3::new(0.0, -9.81, 0.0),
                on_advance: Some(OnAdvance),
                ..SceneDescriptor::new(())
            })
            .unwrap();

        Self {
            foundation,
            scene,
            resources: Default::default(),
        }
    }

    pub fn add_dynamic_actor(&mut self, mut actor: Owner<PxRigidDynamic>) -> RigidDynamicHandle {
        let handle = self.resources.dynamic_actor.insert(actor.as_mut_ptr());
        self.scene.add_dynamic_actor(actor);
        handle
    }

    pub fn remove_dynamic_actor(&mut self, h: RigidDynamicHandle) {
        let actor = self.resources.dynamic_actor.remove(h).expect("double free");
        self.scene
            .remove_actor(unsafe { actor.as_mut().unwrap() }, true);
    }

    fn get_dynamic_actor(&self, handle: RigidDynamicHandle) -> Option<&mut PxRigidDynamic> {
        self.resources
            .dynamic_actor
            .get(handle)
            .and_then(|actor| unsafe { (*actor as *mut PxRigidDynamic).as_mut() })
    }

    pub fn get_dynamic_actor_transform(&self, handle: RigidDynamicHandle) -> Option<IsoTransform> {
        self.get_dynamic_actor(handle).map(|rb| {
            let position: Vec3 = rb.get_global_position().new_from_px();
            let rotation: Quat = rb.get_global_rotation().new_from_px();
            IsoTransform::from_rotation_translation(rotation, position)
        })
    }

    pub fn add_static_mesh_actor(
        &mut self,
        mesh: TriangleMeshHandle,
        transform: Affine3A,
        material: &mut PxMaterial,
    ) {
        let (scale, rotation, translation) = transform.to_scale_rotation_translation();

        let mesh_geo = PxTriangleMeshGeometry::new(
            &mut *self.resources.triangle_mesh.get_mut(mesh).unwrap(),
            &PxMeshScale {
                scale: scale.into_px().into(),
                rotation: physx_sys::PxQuat {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            PxMeshGeometryFlags { mBits: 0 },
        );

        let rigid_static = self
            .foundation
            .create_rigid_static(
                PxTransform::from_translation_rotation(&translation.into_px(), &rotation.into_px()),
                &mesh_geo,
                material,
                PxTransform::default(),
                (),
            )
            .unwrap();

        self.scene.add_static_actor(rigid_static);
    }

    pub fn cook_mesh(
        &mut self,
        asset: &kajiya::asset::mesh::PackedTriMesh::Flat,
    ) -> TriangleMeshHandle {
        let cooking_params =
            PxCookingParams::new(self.foundation.physics()).expect("PxCookingParams::new");
        let cooking = PxCooking::new(self.foundation.foundation_mut(), &cooking_params)
            .expect("PxCooking::new");

        let mut mesh_desc = PxTriangleMeshDesc::new();
        mesh_desc.obj.triangles.data = asset.indices.as_ptr() as *const c_void;
        mesh_desc.obj.triangles.count = (asset.indices.len() / 3) as _;
        mesh_desc.obj.triangles.stride = (std::mem::size_of::<u32>() * 3) as _;
        mesh_desc.obj.points.data = asset.verts.as_ptr() as *const c_void;
        mesh_desc.obj.points.count = asset.verts.len() as _;
        mesh_desc.obj.points.stride = std::mem::size_of::<kajiya::asset::mesh::PackedVertex>() as _;

        let mesh = cooking.create_triangle_mesh(self.foundation.physics_mut(), &mesh_desc);
        let mesh = match mesh {
            physx::cooking::TriangleMeshCookingResult::Success(mesh) => mesh,
            physx::cooking::TriangleMeshCookingResult::LargeTriangle => {
                panic!("PxCooking_createTriangleMesh failed: LargeTriangle")
            }
            physx::cooking::TriangleMeshCookingResult::Failure => {
                panic!("PxCooking_createTriangleMesh failed: Failure")
            }
            physx::cooking::TriangleMeshCookingResult::InvalidDescriptor => {
                panic!("PxCooking_createTriangleMesh failed: InvalidDescriptor")
            }
        };

        self.resources.triangle_mesh.insert(mesh)
    }
}

pub trait IntoPhysx {
    type Target;
    fn into_px(self) -> Self::Target;
}

pub trait FromPhysx {
    type Target;
    fn new_from_px(self) -> Self::Target;
}

impl IntoPhysx for Vec3 {
    type Target = physx::math::PxVec3;

    fn into_px(self) -> Self::Target {
        Self::Target::new(self.x, self.y, self.z)
    }
}

impl IntoPhysx for Quat {
    type Target = physx::math::PxQuat;

    fn into_px(self) -> Self::Target {
        Self::Target::new(self.x, self.y, self.z, self.w)
    }
}

impl FromPhysx for physx::math::PxVec3 {
    type Target = Vec3;

    fn new_from_px(self) -> Self::Target {
        Vec3::new(self.x(), self.y(), self.z())
    }
}

impl FromPhysx for physx_sys::PxVec3 {
    type Target = Vec3;

    fn new_from_px(self) -> Self::Target {
        Vec3::new(self.x, self.y, self.z)
    }
}

impl FromPhysx for physx::math::PxQuat {
    type Target = Quat;

    fn new_from_px(self) -> Self::Target {
        Quat::from_xyzw(self.x(), self.y(), self.z(), self.w())
    }
}

impl FromPhysx for physx_sys::PxQuat {
    type Target = Quat;

    fn new_from_px(self) -> Self::Target {
        Quat::from_xyzw(self.x, self.y, self.z, self.w)
    }
}
