use std::collections::HashMap;

use kajiya_simple::*;
use physx::{prelude::*, traits::Class};

#[derive(Clone, Copy, Hash, Eq, PartialEq)]
pub struct RigidDynamicHandle(usize);

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

pub struct PhysicsState {
    pub foundation: PhysicsFoundation<physx::foundation::DefaultAllocator, PxShape>,
    pub scene: Owner<PxScene>,
    dynamic_actors: HashMap<RigidDynamicHandle, *mut physx_sys::PxRigidDynamic>,
    next_handle: usize,
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
            dynamic_actors: Default::default(),
            next_handle: 1,
        }
    }

    pub fn add_dynamic_actor(&mut self, mut actor: Owner<PxRigidDynamic>) -> RigidDynamicHandle {
        let handle = self.next_handle;
        self.next_handle += 1;

        let handle = RigidDynamicHandle(handle);
        self.dynamic_actors.insert(handle, actor.as_mut_ptr());

        self.scene.add_dynamic_actor(actor);
        handle
    }

    pub fn remove_dynamic_actor(&mut self, h: RigidDynamicHandle) {
        let actor = self.dynamic_actors.remove(&h).expect("double free");
        self.scene
            .remove_actor(unsafe { actor.as_mut().unwrap() }, true);
    }

    pub fn get_dynamic_actor(&self, handle: RigidDynamicHandle) -> Option<&mut PxRigidDynamic> {
        self.dynamic_actors
            .get(&handle)
            .and_then(|actor| unsafe { (*actor as *mut PxRigidDynamic).as_mut() })
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
