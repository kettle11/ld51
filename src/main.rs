use koi3::{scripts::CameraControls, *};
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

            let angle = transform.rotation.to_angle_axis().0;
            if body.rotation().angle() != angle {
                body.set_rotation(angle, true);
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
    let materials: Vec<Handle<Material>> = {
        let mut material_store = resources.get::<AssetStore<Material>>();

        [
            material_store.add(Material {
                base_color: Color::RED,
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            material_store.add(Material {
                base_color: Color::GREEN,
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            material_store.add(Material {
                base_color: Color::YELLOW.with_chroma(0.2),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
        ]
        .into()
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
        CameraControls::default(),
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
        Mesh::VERTICAL_CIRCLE,
        Material::UNLIT,
        rapier_handle,
    ));

    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::dynamic().build(),
        ColliderBuilder::ball(0.5).restitution(0.7).build(),
    );
    world.spawn((
        Transform::new().with_position(Vec3::Y * 1.0),
        Mesh::VERTICAL_CIRCLE,
        Material::UNLIT,
        rapier_handle,
    ));

    */

    create_cannon(world, &mut rapier_integration, &materials);

    create_victory_orb(world, &mut rapier_integration, &materials, -Vec3::X * 4.0);
    create_laser(world, &mut rapier_integration, &materials, -Vec3::X * 1.0);

    resources.add(rapier_integration);
    resources.add(SceneInfo {
        moving_entity: None,
        moving_offset: Vec2::ZERO,
        running: true,
        materials,
    });
}

struct VictoryOrb {
    power: f32,
    child: Entity,
}

struct Particle {}

fn run_victory_orb(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();
    let mut rapier_integration = resources.get::<RapierIntegration>();

    let mut to_despawn = Vec::new();

    for (e, (transform, victory_orb)) in world.query::<(&Transform, &mut VictoryOrb)>().iter() {
        let collider = world.get::<&RapierRigidBody>(e).unwrap();
        for contact_pair in rapier_integration
            .narrow_phase
            .contacts_with(collider.collider_handle)
        {
            let other_collider = if contact_pair.collider1 == collider.collider_handle {
                contact_pair.collider2
            } else {
                contact_pair.collider1
            };

            let user_data = rapier_integration
                .collider_set
                .get(other_collider)
                .unwrap()
                .user_data;

            if user_data != 0 {
                let entity = Entity::from_bits(user_data as _).unwrap();
                if world.get::<&Particle>(entity).is_ok() {
                    println!("CONTACT WITH VICTORY ORB: {:?}", contact_pair.collider2);
                    to_despawn.push(entity);
                    victory_orb.power += 0.1;
                    if victory_orb.power > 1.0 {
                        victory_orb.power = 1.0;
                        println!("WIN CONDITION");
                    }
                }
            }
        }

        world
            .get::<&mut Transform>(victory_orb.child)
            .unwrap()
            .scale = Vec3::fill(victory_orb.power);
    }
    for e in to_despawn {
        let _ = world.despawn(e);
    }
}

fn create_victory_orb(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &[Handle<Material>],
    position: Vec3,
) {
    let scale = 2.0;
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5 * scale, 0.5 * scale)
            .restitution(0.7)
            .build(),
    );

    let child = world.spawn((
        Transform::new()
            .with_scale(Vec3::fill(1.2))
            .with_position(Vec3::Z * 0.2),
        Mesh::VERTICAL_CIRCLE,
        materials[2].clone(),
    ));

    let parent = world.spawn((
        Transform::new()
            .with_position(position)
            .with_scale(Vec3::fill(scale)),
        Mesh::VERTICAL_CIRCLE,
        materials[1].clone(),
        rapier_handle,
        VictoryOrb { power: 0.2, child },
    ));

    world.set_parent(parent, child).unwrap();
}

struct Laser;

struct Activated(bool);

struct ActivateOnTimer {
    current: f32,
    frequency: f32, // Every 10.0 seconds
}

fn run_activate_on_timer(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();

    for (_, (timer, activated)) in world
        .query::<(&mut ActivateOnTimer, &mut Activated)>()
        .iter()
    {
        activated.0 = true;

        timer.current -= time.fixed_time_step_seconds as f32;
        if timer.current < 0.0 {
            timer.current = timer.frequency;
            activated.0 = true;
        } else {
            activated.0 = false;
        }
    }
}

fn create_laser(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &[Handle<Material>],
    position: Vec3,
) {
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5, 0.5)
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );
    world.spawn((
        Transform::new()
            .with_position(position + Vec3::Z * 0.1)
            .with_rotation(Quat::from_angle_axis(std::f32::consts::PI * 0.25, Vec3::Z)),
        Mesh::VERTICAL_QUAD,
        materials[2].clone(),
        Laser,
        Activated(false),
        ActivateOnTimer {
            current: 0.0,
            frequency: 10.0, // Every 10 seconds!
        },
        rapier_handle,
    ));
}

fn run_laser(world: &mut World, resources: &mut Resources) {
    let rapier_integration = resources.get::<RapierIntegration>();

    let mut to_spawn = Vec::new();

    for (_, (transform, _laser, rigid_body, activated)) in world
        .query::<(&GlobalTransform, &mut Laser, &RapierRigidBody, &Activated)>()
        .iter()
    {
        if activated.0 {
            println!("ACTIVATE LASER!");
            let direction = transform.right().xy().normalized();
            let ray = Ray::new(
                point![transform.position.x, transform.position.y],
                vector![direction.x, direction.y],
            );

            if let Some((collider_handle, intersection)) =
                rapier_integration.query_pipeline.cast_ray_and_get_normal(
                    &rapier_integration.rigid_body_set,
                    &rapier_integration.collider_set,
                    &ray,
                    1000.0,
                    true,
                    QueryFilter::new().exclude_rigid_body(rigid_body.rigid_body_handle),
                )
            {
                let hit_point = ray.point_at(intersection.toi);
                let hit_normal = intersection.normal;

                let collider = rapier_integration
                    .collider_set
                    .get(collider_handle)
                    .unwrap();
                if collider.user_data != 0 {
                    let entity = Entity::from_bits(collider.user_data as _).unwrap();
                    if let Ok(mut activated) = world.get::<&mut Activated>(entity) {
                        activated.0 = true;
                    }
                }

                to_spawn.push((transform.position.xy(), Vec2::new(hit_point.x, hit_point.y)));
            } else {
                to_spawn.push((
                    transform.position.xy(),
                    transform.position.xy() + direction * 100.0,
                ));
            }
        }
    }

    for (start, end) in to_spawn {
        let diff = end - start;
        let diff_normalized = diff.normalized();
        let middle = (end - start) / 2.0 + start;

        world.spawn((
            Transform::new()
                .with_position(middle.extend(0.0))
                .with_scale(Vec3::new(diff.length(), 0.1, 1.0))
                .with_rotation(Quat::from_forward_up(
                    -Vec3::Z,
                    Vec3::new(diff_normalized.y, -diff_normalized.x, 0.0),
                )),
            Mesh::VERTICAL_QUAD,
            Material::UNLIT,
            Ephemeral(0.5),
        ));
    }
}

struct Cannon;

fn create_cannon(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &[Handle<Material>],
) {
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5, 0.5)
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );
    world.spawn((
        Transform::new()
            .with_position(Vec3::Y * 2.0 + Vec3::Z * 0.1)
            .with_rotation(Quat::from_angle_axis(0.9, Vec3::Z)),
        Mesh::VERTICAL_QUAD,
        materials[0].clone(),
        Cannon,
        Activated(false),
        rapier_handle,
    ));
}

fn run_cannon(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();
    let mut rapier_integration = resources.get::<RapierIntegration>();

    let mut to_spawn = Vec::new();
    for (_, (transform, cannon, activated)) in world
        .query::<(&Transform, &mut Cannon, &Activated)>()
        .iter()
    {
        //  cannon.timer -= time.fixed_time_step_seconds as f32;
        if activated.0 {
            to_spawn.push((transform.clone(), transform.right().xy() * 5.0));
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
            Mesh::VERTICAL_CIRCLE,
            Material::UNLIT,
            rapier_handle,
            Particle {},
        ));
    }
}

struct Ephemeral(f32);

pub fn run_ephemeral(world: &mut World, time: &Time) {
    let mut to_despawn = Vec::new();
    for (e, ephermal) in world.query::<With<&mut Ephemeral, ()>>().iter() {
        ephermal.0 -= time.draw_delta_seconds as f32;
        if ephermal.0 < 0.0 {
            to_despawn.push(e);
        }
    }

    for e in to_despawn {
        world.despawn(e).unwrap();
    }
}

fn main() {
    App::default().setup_and_run(|world, resources| {
        setup(world, resources);

        move |event, world, resources| match event {
            Event::Draw => {
                run_ephemeral(world, &*resources.get::<Time>());
            }
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
                    for (_, activated) in world.query::<&mut Activated>().iter() {
                        activated.0 = false;
                    }

                    run_activate_on_timer(world, resources);
                    run_laser(world, resources);
                    run_cannon(world, resources);
                    run_victory_orb(world, resources);
                    resources.get::<RapierIntegration>().step(world);
                }
            }

            _ => {}
        }
    });
}
