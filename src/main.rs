use koi3::*;
use rapier2d::prelude::*;

struct RapierRigidBody {
    handle: rapier2d::prelude::RigidBodyHandle,
}

struct RapierIntegration {
    gravity: Vec2,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
}

impl RapierIntegration {
    pub fn new() -> Self {
        Self {
            gravity: Vec2::new(0.0, -9.81),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
        }
    }

    pub fn add_collider(&mut self, collider: Collider) {
        self.collider_set.insert(collider);
    }

    pub fn add_rigid_body_with_collider(
        &mut self,
        rigid_body: RigidBody,
        collider: Collider,
    ) -> RapierRigidBody {
        let rigid_body_handle = self.rigid_body_set.insert(rigid_body);
        self.collider_set
            .insert_with_parent(collider, rigid_body_handle, &mut self.rigid_body_set);
        RapierRigidBody {
            handle: rigid_body_handle,
        }
    }

    pub fn step(&mut self, world: &mut World) {
        let gravity: [f32; 2] = self.gravity.into();
        let gravity = gravity.into();
        self.physics_pipeline.step(
            &gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &(),
            &(),
        );

        for (_, (transform, rigid_body)) in
            world.query::<(&mut Transform, &RapierRigidBody)>().iter()
        {
            let body = &self.rigid_body_set[rigid_body.handle];
            let p: [f32; 2] = body.position().translation.into();
            transform.position = Vec3::new(p[0], p[1], 0.0);
        }
    }
}

fn main() {
    App::default().setup_and_run(|world, resources| {
        world.spawn((
            Transform::new().with_position(Vec3::Z * 2.0),
            Camera {
                clear_color: Some(Color::CORNFLOWER_BLUE),
                ..Default::default()
            },
        ));

        let mut rapier_integration = RapierIntegration::new();
        rapier_integration.add_collider(ColliderBuilder::cuboid(100.0, 0.1).build());

        let rapier_handle = rapier_integration.add_rigid_body_with_collider(
            RigidBodyBuilder::dynamic()
                .translation(vector![0.0, 10.0])
                .build(),
            ColliderBuilder::ball(0.5).restitution(0.7).build(),
        );

        let ball_visuals = world.spawn((
            Transform::new(),
            Mesh::SPHERE,
            Material::UNLIT,
            rapier_handle,
        ));

        resources.add(rapier_integration);

        move |event, world, resources| match event {
            Event::FixedUpdate => {
                if resources.get_mut::<Input>().key_down(Key::Space) {
                    let (_, camera) = world.query_mut::<&mut Camera>().into_iter().next().unwrap();
                    camera.clear_color = Some(Color::ELECTRIC_INDIGO);
                }

                resources.get_mut::<RapierIntegration>().step(world);
            }

            _ => {}
        }
    });
}
