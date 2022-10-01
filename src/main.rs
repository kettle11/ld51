use koi3::*;
use rapier2d::prelude::*;

struct RapierRigidBody {
    rigid_body_handle: rapier2d::prelude::RigidBodyHandle,
    collider_handle: rapier2d::prelude::ColliderHandle,
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
    query_pipeline: QueryPipeline,
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
            query_pipeline: QueryPipeline::new(),
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
        let collider_handle = self.collider_set.insert_with_parent(
            collider,
            rigid_body_handle,
            &mut self.rigid_body_set,
        );
        RapierRigidBody {
            rigid_body_handle: rigid_body_handle,
            collider_handle,
        }
    }

    pub fn step(&mut self, world: &mut World) {
        let mut to_despawn = Vec::new();
        for collider in self.collider_set.iter() {
            if collider.1.user_data != 0 {
                let entity = Entity::from_bits(collider.1.user_data as _).unwrap();
                if !world.contains(entity) {
                    to_despawn.push(collider.1.parent().unwrap());
                }
            }
        }

        for c in to_despawn {
            self.rigid_body_set.remove(
                c,
                &mut self.island_manager,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                true,
            );
        }

        // If the collider has been moved manually
        for (e, (transform, rigid_body)) in
            world.query::<(&mut Transform, &RapierRigidBody)>().iter()
        {
            self.collider_set
                .get_mut(rigid_body.collider_handle)
                .unwrap()
                .user_data = e.to_bits().get() as _;

            let body = &mut self.rigid_body_set[rigid_body.rigid_body_handle];
            let p_array: [f32; 2] = body.position().translation.into();
            if p_array != transform.position.xy().as_array() {
                let new_p: [f32; 2] = transform.position.xy().into();
                body.set_translation(new_p.into(), true);
            }
        }

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
            let body = &self.rigid_body_set[rigid_body.rigid_body_handle];
            let p: [f32; 2] = body.position().translation.into();
            let r: f32 = body.rotation().angle();

            transform.position = Vec3::new(p[0], p[1], transform.position.z);
            transform.rotation = Quat::from_angle_axis(r, Vec3::Z);
        }

        self.query_pipeline.update(
            &self.island_manager,
            &self.rigid_body_set,
            &self.collider_set,
        );
    }
}

struct SceneInfo {
    moving_entity: Option<Entity>,
    moving_offset: Vec2,
    running: bool,
    materials: Vec<Handle<Material>>,
}

impl SceneInfo {
    pub fn new(materials: Vec<Handle<Material>>) -> Self {
        Self {
            moving_entity: None,
            moving_offset: Vec2::ZERO,
            running: true,
            materials,
        }
    }
}

pub fn setup(world: &mut World, resources: &mut Resources) {
    let materials = {
        let mut material_store = resources.get::<AssetStore<Material>>();
        let mut materials = Vec::new();
        materials.push(material_store.add(Material {
            base_color: Color::RED,
            shader: Shader::UNLIT,
            ..Default::default()
        }));
        materials
    };

    world.clear();

    world.spawn((
        Transform::new(),
        Camera {
            clear_color: Some(Color::CORNFLOWER_BLUE),
            projection_mode: ProjectionMode::Orthographic {
                height: 10.0,
                z_near: -1.0,
                z_far: 2.0,
            },
            ..Default::default()
        },
    ));

    let mut rapier_integration = RapierIntegration::new();
    rapier_integration.add_collider(ColliderBuilder::cuboid(100.0, 0.1).build());

    /*
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).restitution(0.7).build(),
    );


    world.spawn((
        Transform::new().with_position(Vec3::Y * 3.0),
        Mesh::SPHERE,
        Material::UNLIT,
        rapier_handle,
    ));

    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).restitution(0.7).build(),
    );
    world.spawn((
        Transform::new().with_position(Vec3::Y * 1.0),
        Mesh::SPHERE,
        Material::UNLIT,
        rapier_handle,
    ));

    */

    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5, 0.5).restitution(0.7).build(),
    );
    world.spawn((
        Transform::new()
            .with_position(Vec3::Y * 2.0 + Vec3::Z * 0.1)
            .with_rotation(Quat::from_angle_axis(0.9, Vec3::Z)),
        Mesh::VERTICAL_QUAD,
        materials[0].clone(),
        Cannon { timer: 0.0 },
        rapier_handle,
    ));

    resources.add(rapier_integration);
    resources.add(SceneInfo {
        moving_entity: None,
        moving_offset: Vec2::ZERO,
        running: true,
        materials,
    });
}

struct Cannon {
    timer: f32,
}

fn run_cannon(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();
    let mut rapier_integration = resources.get::<RapierIntegration>();

    let mut to_spawn = Vec::new();
    for (_, (transform, cannon)) in world.query::<(&Transform, &mut Cannon)>().iter() {
        cannon.timer -= time.fixed_time_step_seconds as f32;
        if cannon.timer < 0.0 {
            to_spawn.push((transform.clone(), transform.right().xy() * 5.0));
            cannon.timer = 0.5;
        }
    }

    for (t, v) in to_spawn {
        let rapier_handle = rapier_integration.add_rigid_body_with_collider(
            RigidBodyBuilder::dynamic()
                .linvel([v.x, v.y].into())
                .build(),
            ColliderBuilder::ball(0.5).restitution(0.7).build(),
        );
        world.spawn((
            t.clone()
                .with_position(t.position + v.extend(0.0) * t.scale.max_component() * 0.1),
            Mesh::SPHERE,
            Material::UNLIT,
            rapier_handle,
        ));
    }
}

fn main() {
    App::default().setup_and_run(|world, resources| {
        setup(world, resources);

        move |event, world, resources| match event {
            Event::FixedUpdate => {
                {
                    let key_pressed = resources.get::<Input>().key_down(Key::R);
                    if key_pressed {
                        setup(world, resources);
                    }
                }

                {
                    let input = resources.get::<Input>();
                    let mut rapier_integration = resources.get::<RapierIntegration>();
                    let mut scene_info = resources.get::<SceneInfo>();

                    if input.key_down(Key::P) {
                        scene_info.running = !scene_info.running;
                    }

                    if input.pointer_button_released(PointerButton::Primary) {
                        if let Some(moving_entity) = scene_info.moving_entity {
                            let rigid_body_handle = rapier_integration
                                .rigid_body_set
                                .get_mut(
                                    world
                                        .get::<&mut RapierRigidBody>(moving_entity)
                                        .unwrap()
                                        .rigid_body_handle,
                                )
                                .unwrap();

                            rigid_body_handle.set_gravity_scale(1.0, false);
                        }
                        scene_info.moving_entity = None;
                    }

                    let pointer_position = {
                        let (x, y) = input.pointer_position();
                        let mut q = world.query::<(&Transform, &Camera)>();
                        let mut iter = q.iter();
                        let (camera_transform, camera) = iter.next().unwrap().1;

                        let view_size = resources.get::<kapp::Window>().size();
                        let ray = camera.view_to_ray(
                            camera_transform,
                            x as _,
                            y as _,
                            view_size.0 as _,
                            view_size.1 as _,
                        );
                        ray.origin.xy()
                    };

                    if let Some(moving_entity) = scene_info.moving_entity {
                        world.get::<&mut Transform>(moving_entity).unwrap().position =
                            (pointer_position - scene_info.moving_offset).extend(0.0);
                        let rigid_body_handle = rapier_integration
                            .rigid_body_set
                            .get_mut(
                                world
                                    .get::<&mut RapierRigidBody>(moving_entity)
                                    .unwrap()
                                    .rigid_body_handle,
                            )
                            .unwrap();
                        rigid_body_handle.set_linvel([0.0, 0.0].into(), true);
                        rigid_body_handle.set_angvel(0.0, false);
                        rigid_body_handle.set_gravity_scale(0.0, false);
                    }

                    if input.pointer_button_down(PointerButton::Primary) {
                        rapier_integration.query_pipeline.intersections_with_point(
                            &rapier_integration.rigid_body_set,
                            &rapier_integration.collider_set,
                            &point![pointer_position.x, pointer_position.y],
                            QueryFilter::default(),
                            |handle| {
                                let collider = rapier_integration.collider_set.get(handle).unwrap();
                                if collider.user_data != 0 {
                                    let entity =
                                        Entity::from_bits(collider.user_data as _).unwrap();
                                    scene_info.moving_offset = pointer_position
                                        - world
                                            .get::<&mut Transform>(entity)
                                            .unwrap()
                                            .position
                                            .xy();

                                    scene_info.moving_entity = Some(entity);
                                }
                                false
                            },
                        );
                    }
                }

                if resources.get::<SceneInfo>().running {
                    run_cannon(world, resources);
                    resources.get::<RapierIntegration>().step(world);
                }
            }

            _ => {}
        }
    });
}
