use std::io::Split;

use camera_controls::CameraControls;
use koi3::*;
use rapier2d::prelude::*;

mod camera_controls;

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

    pub fn update_colliders(&mut self, world: &mut World) {
        let mut entities_to_despawn = Vec::new();
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

            if transform.position.xy().length_squared() > (1000. * 1000.) {
                entities_to_despawn.push(e);
            }
        }

        for e in entities_to_despawn {
            let _ = world.despawn(e);
        }

        let mut to_despawn = Vec::new();

        for collider in self.collider_set.iter() {
            if collider.1.user_data != 0 {
                let entity = Entity::from_bits(collider.1.user_data as _).unwrap();
                if !world.contains(entity) {
                    to_despawn.push(collider.1.parent().unwrap());
                }
            } else {
                to_despawn.push(collider.1.parent().unwrap());
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
    }
    pub fn step(&mut self, world: &mut World) {
        self.update_colliders(world);

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
    materials: Materials,
    screen_shake_amount: f32,
    freeze_time: f32,
    max_power: f32,
    splitter_mesh: Handle<Mesh>,
}

struct Materials {
    sparkle: Handle<Material>,
    cannon: Handle<Material>,
    cannon_inner: Handle<Material>,
    laser: Handle<Material>,
    laser_inner: Handle<Material>,
    victory_orb: Handle<Material>,
    victory_orb_inner: Handle<Material>,
    environment_chunk: Handle<Material>,
}

// struct PoweredUpBar;

pub fn setup(world: &mut World, resources: &mut Resources) {
    let materials = {
        let mut material_store = resources.get::<AssetStore<Material>>();

        Materials {
            sparkle: {
                let texture = new_texture_from_bytes(
                    &mut resources.get::<Renderer>().raw_graphics_context,
                    "png",
                    include_bytes!("../assets/star.png"),
                    koi_graphics_context::TextureSettings::default(),
                )
                .unwrap();
                let star_texture = resources.get::<AssetStore<Texture>>().add(texture);

                material_store.add(Material {
                    shader: Shader::UNLIT_TRANSPARENT,
                    base_color_texture: Some(star_texture),
                    base_color: Color::YELLOW,
                    ..Default::default()
                })
            },
            cannon: material_store.add(Material {
                base_color: Color::RED,
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            cannon_inner: material_store.add(Material {
                base_color: Color::GREEN,
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            laser: material_store.add(Material {
                base_color: Color::YELLOW.with_chroma(0.2),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            laser_inner: material_store.add(Material {
                base_color: Color::PURPLE.with_chroma(0.8).with_lightness(0.6),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            victory_orb: material_store.add(Material {
                base_color: Color::from_srgb_hex(0xFFD700, 1.0),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            victory_orb_inner: material_store.add(Material {
                base_color: Color::PURPLE.with_chroma(0.8).with_lightness(0.6),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
            environment_chunk: material_store.add(Material {
                base_color: Color::PURPLE.with_chroma(0.8).with_lightness(0.6),
                shader: Shader::UNLIT,
                ..Default::default()
            }),
        }
    };

    world.clear();

    /*
    let screen_ui = world.spawn((
        Transform::new(),
        PoweredUpBar,
        Mesh::VERTICAL_QUAD,
        Material::UNLIT,
    ));
    */

    let camera = world.spawn((
        Transform::new(),
        Camera {
            clear_color: Some(Color::CORNFLOWER_BLUE),

            projection_mode: ProjectionMode::Orthographic {
                height: 16.0,
                z_near: -1.0,
                z_far: 2.0,
            },

            ..Default::default()
        },
    ));
    let camera_parent = world.spawn((Transform::new(), CameraControls::new(camera)));

    //world.set_parent(camera, screen_ui).unwrap();
    world.set_parent(camera_parent, camera).unwrap();

    let mut rapier_integration = RapierIntegration::new();

    let scene_info = SceneInfo {
        splitter_mesh: {
            resources.get::<AssetStore<Mesh>>().add(Mesh::new(
                &mut resources.get::<Renderer>().raw_graphics_context,
                circle(-Vec3::Z, 8, 0.5),
            ))
        },
        moving_entity: None,
        moving_offset: Vec2::ZERO,
        running: true,
        materials,
        screen_shake_amount: 0.0,
        freeze_time: 0.0,
        max_power: 1.0,
    };

    {
        let mut meshes = resources.get::<AssetStore<Mesh>>();

        deserialize_world(
            &std::fs::read_to_string("src/level.txt").unwrap(),
            world,
            &mut rapier_integration,
            &scene_info,
            &meshes,
        );
    }

    resources.add(rapier_integration);

    resources.add(scene_info);
}

struct VictoryOrb {
    health: usize,
    full_health: usize,
    child: Entity,
    gifts: Vec<Gift>,
}

enum Gift {
    Cannon,
    Chunk,
    Laser,
    Splitter,
}

struct Particle {}

fn run_victory_orb(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();
    let mut rapier_integration = resources.get::<RapierIntegration>();

    let mut scene_info = resources.get::<SceneInfo>();

    let mut to_despawn = Vec::new();
    let mut to_explode = Vec::new();

    let mut random = Random::new();
    let mut commands = CommandBuffer::new();

    for (e, (transform, victory_orb)) in world.query::<(&Transform, &mut VictoryOrb)>().iter() {
        //victory_orb.power -= 0.05 * time.fixed_time_step_seconds as f32;

        if random.f32() > 0.98 {
            let position_offset: Vec2 =
                transform.scale.x * random.f32() * random.normalized_vec() * 0.8;
            commands.spawn((
                transform
                    .clone()
                    .with_scale(Vec3::fill(1.0))
                    .with_position(transform.position + position_offset.extend(0.0)),
                Shrink::new(1.0),
                scene_info.materials.sparkle.clone(),
                Mesh::VERTICAL_QUAD,
            ))
        }

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
                    to_despawn.push(entity);
                    victory_orb.health = victory_orb.health.saturating_sub(1);
                    scene_info.screen_shake_amount += 0.05;
                    if victory_orb.health == 0 {
                        scene_info.screen_shake_amount += 0.3;

                        scene_info.freeze_time = 1.0;
                        to_explode.push((transform.clone(), e));
                    }
                }
            }
        }

        world
            .get::<&mut Transform>(victory_orb.child)
            .unwrap()
            .scale = Vec3::fill(1.0 - (victory_orb.health as f32 / victory_orb.full_health as f32));
    }

    commands.run_on(world);

    for e in to_despawn {
        let _ = world.insert(e, (Ephemeral(0.2),));
        let _ = world.remove_one::<Particle>(e);
    }

    let meshes = resources.get::<AssetStore<Mesh>>();
    for (transform, e) in to_explode {
        let transform = transform.with_scale(Vec3::fill(1.0));
        let _ = world.insert(e, (Ephemeral(0.2),));

        let victory_orb = world.remove_one::<VictoryOrb>(e).unwrap();

        for gift in victory_orb.gifts.iter() {
            match gift {
                Gift::Cannon => create_cannon(
                    world,
                    &mut rapier_integration,
                    &scene_info.materials,
                    transform,
                ),
                Gift::Chunk => create_environment_chunk(
                    world,
                    &mut rapier_integration,
                    &scene_info.materials,
                    transform,
                ),
                Gift::Laser => create_laser(
                    world,
                    &mut rapier_integration,
                    &scene_info.materials,
                    transform,
                ),
                Gift::Splitter => create_splitter(
                    world,
                    &scene_info,
                    &meshes,
                    &mut rapier_integration,
                    transform,
                ),
            }
        }

        // let _ = world.insert(e, (Ephemeral(0.2),));
        // let _ = world.remove_one::<Particle>(e);
    }
}

fn create_victory_orb(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &Materials,
    transform: Transform,
) {
    let scale = 2.0;
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::ball(0.5 * scale).restitution(0.7).build(),
    );

    let child = world.spawn((
        Transform::new(),
        Mesh::VERTICAL_CIRCLE,
        materials.victory_orb_inner.clone(),
    ));

    let parent = world.spawn((
        transform.with_scale(Vec3::fill(scale) * 1.2),
        Mesh::VERTICAL_CIRCLE,
        materials.victory_orb.clone(),
        rapier_handle,
        VictoryOrb {
            health: 2,
            full_health: 2,
            child,
            gifts: vec![Gift::Splitter],
        },
        Movable,
    ));

    world.set_parent(parent, child).unwrap();
}

struct Laser {
    laser_trigger: f32,
}

struct Activated {
    this_frame: bool,
    activated: bool,
    reset_charge_on_activate: bool,
}

impl Default for Activated {
    fn default() -> Self {
        Self {
            this_frame: false,
            activated: false,
            reset_charge_on_activate: false,
        }
    }
}

struct ChargeOnTimer {
    amount: f32, // Every 10.0 seconds
}

fn run_charge_on_timer(world: &mut World, resources: &mut Resources) {
    let time = resources.get::<Time>();

    for (_, (charge_on_timer, chargable)) in
        world.query::<(&mut ChargeOnTimer, &mut Chargable)>().iter()
    {
        chargable.charge(charge_on_timer.amount * time.fixed_time_step_seconds as f32);
    }
}

fn create_laser(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &Materials,
    transform: Transform,
) {
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5, 0.5)
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let child = world.spawn((
        Transform::new().with_position(Vec3::Z * 0.2),
        Mesh::VERTICAL_QUAD,
        materials.laser_inner.clone(),
    ));

    let parent = world.spawn((
        transform,
        // Transform::new()
        //     .with_position(position + Vec3::Z * 0.1)
        //     .with_rotation(Quat::from_angle_axis(std::f32::consts::PI * 0.25, Vec3::Z)),
        Mesh::VERTICAL_QUAD,
        materials.laser.clone(),
        Laser { laser_trigger: 0.0 },
        Activated {
            reset_charge_on_activate: true,
            ..Default::default()
        },
        ChargeOnTimer {
            amount: 1.0, // Every 10 seconds!
        },
        Chargable::default(),
        Movable,
        rapier_handle,
        ScaleChildToChargable { child },
    ));

    world.set_parent(parent, child).unwrap();
}

fn run_laser(world: &mut World, resources: &mut Resources) {
    let rapier_integration = resources.get::<RapierIntegration>();
    let time = resources.get::<Time>();

    let mut to_spawn = Vec::new();

    for (_, (transform, laser, rigid_body, activated)) in world
        .query::<(&GlobalTransform, &mut Laser, &RapierRigidBody, &Activated)>()
        .iter()
    {
        if activated.this_frame {
            laser.laser_trigger = 0.3;
        }

        if laser.laser_trigger > 0.0 {
            // println!("LASER --------------");

            laser.laser_trigger -= time.fixed_time_step_seconds as f32;

            let direction = transform.right().xy().normalized();

            fn raycast(
                rapier_integration: &RapierIntegration,
                query_filter: QueryFilter,
                start: Vec2,
                direction: Vec2,
                to_spawn: &mut Vec<(Vec2, Vec2)>,
                world: &World,
                depth: usize,
                delta_time: f32,
            ) {
                if depth == 0 {
                    return;
                }

                let ray = Ray::new(point![start.x, start.y], vector![direction.x, direction.y]);

                if let Some((collider_handle, intersection)) =
                    rapier_integration.query_pipeline.cast_ray_and_get_normal(
                        &rapier_integration.rigid_body_set,
                        &rapier_integration.collider_set,
                        &ray,
                        1000.0,
                        true,
                        query_filter,
                    )
                {
                    let hit_point = ray.point_at(intersection.toi);
                    let hit_normal = intersection.normal;

                    let hit_normal = Vec2::new(hit_normal.x, hit_normal.y);

                    let collider = rapier_integration
                        .collider_set
                        .get(collider_handle)
                        .unwrap();

                    let mut reflect = true;
                    let end = Vec2::new(hit_point.x, hit_point.y);
                    to_spawn.push((start, end));
                    let new_dir = direction - 2.0 * direction.dot(hit_normal) * hit_normal;

                    if collider.user_data != 0 {
                        let entity = Entity::from_bits(collider.user_data as _).unwrap();
                        if let Ok(mut chargable) = world.get::<&mut Chargable>(entity) {
                            chargable.charge(8.0 * delta_time);
                            reflect = false;
                        }

                        if let Ok(mut chargable) = world.get::<&Splitter>(entity) {
                            let collider = world
                                .get::<&RapierRigidBody>(entity)
                                .unwrap()
                                .collider_handle;
                            let transform = world.get::<&Transform>(entity).unwrap();
                            let new_dir = Vec2::new(direction.y, -direction.x);

                            raycast(
                                rapier_integration,
                                QueryFilter::default().exclude_collider(collider),
                                transform.position.xy(),
                                new_dir,
                                to_spawn,
                                world,
                                depth - 1,
                                delta_time,
                            );
                            let new_dir = Vec2::new(-direction.y, direction.x);

                            raycast(
                                rapier_integration,
                                QueryFilter::default().exclude_collider(collider),
                                transform.position.xy(),
                                new_dir,
                                to_spawn,
                                world,
                                depth - 1,
                                delta_time,
                            );
                            reflect = false;
                        }

                        if let Ok(mut cannon) = world.get::<&mut Cannon>(entity) {
                            cannon.fire_direction = direction;
                        }
                    }

                    let end = Vec2::new(hit_point.x, hit_point.y);
                    to_spawn.push((start, end));

                    let new_dir = direction - 2.0 * direction.dot(hit_normal) * hit_normal;

                    if reflect {
                        raycast(
                            rapier_integration,
                            QueryFilter::default(),
                            end + new_dir * 0.05,
                            new_dir,
                            to_spawn,
                            world,
                            depth - 1,
                            delta_time,
                        )
                    }
                } else {
                    to_spawn.push((start, start + direction * 100.0));
                }
            }

            raycast(
                &rapier_integration,
                QueryFilter::default().exclude_rigid_body(rigid_body.rigid_body_handle),
                transform.position.xy(),
                direction,
                &mut to_spawn,
                world,
                10,
                time.fixed_time_step_seconds as f32,
            );
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
            Ephemeral(0.1),
        ));
    }
}

struct Cannon {
    fire_direction: Vec2,
}

struct Movable;

struct Chargable {
    charged: bool,
    amount: f32,
    rate: f32,
}

impl Chargable {
    pub fn charge(&mut self, amount: f32) {
        self.amount += amount * self.rate;
        self.charged = true;
    }
}

impl Default for Chargable {
    fn default() -> Self {
        Self {
            charged: false,
            amount: 0.0,
            rate: 1.0,
        }
    }
}
struct ScaleChildToChargable {
    child: Entity,
}

struct Shake {
    amount: f32,
    target: Entity,
}

fn run_shake_child(world: &mut World, _resources: &Resources) {
    let mut random = Random::new();
    for (_, shake) in world.query::<(&mut Shake)>().iter() {
        let screen_shake = Vec2::new(
            random.range_f32(-shake.amount..shake.amount),
            random.range_f32(-shake.amount..shake.amount),
        );

        let mut transform = world.get::<&mut Transform>(shake.target).unwrap();
        transform.position = screen_shake.extend(transform.position.z);

        shake.amount *= 0.94;
    }
}

struct Splitter;

fn create_splitter(
    world: &mut World,
    scene_info: &SceneInfo,
    meshes: &AssetStore<Mesh>,
    rapier_integration: &mut RapierIntegration,
    transform: Transform,
) {
    let mesh = meshes.get(&scene_info.splitter_mesh);
    let mut points = Vec::new();
    for p in mesh.mesh_data.as_ref().unwrap().positions.iter() {
        let p: [f32; 2] = p.xy().into();
        points.push(p.into());
    }
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::convex_hull(&points)
            .unwrap()
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let visuals = world.spawn((
        Transform::new(),
        scene_info.splitter_mesh.clone(),
        scene_info.materials.laser.clone(),
    ));

    let parent = world.spawn((
        transform.with_rotation(Quat::from_angle_axis(std::f32::consts::TAU * 0.25, Vec3::Z)),
        Movable,
        rapier_handle,
        Splitter,
    ));
    world.set_parent(parent, visuals).unwrap();
}

fn create_cannon(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &Materials,
    transform: Transform,
) {
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5, 0.5)
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );

    let child = world.spawn((
        Transform::new(),
        Mesh::VERTICAL_QUAD,
        materials.cannon_inner.clone(),
    ));
    let visuals = world.spawn((
        Transform::new(),
        Mesh::VERTICAL_QUAD,
        materials.cannon.clone(),
    ));

    let parent = world.spawn((
        transform,
        Cannon {
            fire_direction: Vec2::X,
        },
        Activated::default(),
        Chargable::default(),
        ScaleChildToChargable { child },
        Shake {
            amount: 0.0,
            target: visuals,
        },
        Movable,
        rapier_handle,
    ));
    world.set_parent(visuals, child).unwrap();
    world.set_parent(parent, visuals).unwrap();
}

struct EnvironmentChunk;

fn create_environment_chunk(
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    materials: &Materials,
    transform: Transform,
) {
    let scale = transform.scale.x;
    let rapier_handle = rapier_integration.add_rigid_body_with_collider(
        RigidBodyBuilder::kinematic_position_based().build(),
        ColliderBuilder::cuboid(0.5 * scale, 0.5 * scale)
            .restitution(0.7)
            .active_events(ActiveEvents::COLLISION_EVENTS)
            .build(),
    );
    world.spawn((
        transform.with_scale(Vec3::fill(transform.scale.x)),
        Mesh::VERTICAL_QUAD,
        materials.environment_chunk.clone(),
        rapier_handle,
        Movable,
        EnvironmentChunk,
    ));
}

fn run_cannon(world: &mut World, resources: &mut Resources) {
    //let time = resources.get::<Time>();
    let mut rapier_integration = resources.get::<RapierIntegration>();

    let mut to_spawn = Vec::new();
    for (_, (transform, cannon, activated, shake)) in world
        .query::<(&Transform, &mut Cannon, &Activated, &mut Shake)>()
        .iter()
    {
        //  cannon.timer -= time.fixed_time_step_seconds as f32;fr
        if activated.this_frame {
            to_spawn.push((transform.clone(), cannon.fire_direction * 6.0));
            shake.amount += 0.02;
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
            Movable,
            Particle {},
        ));
    }
}

struct Ephemeral(f32);
struct Shrink {
    value: f32,
    base_scale: f32,
}
impl Shrink {
    pub fn new(base_scale: f32) -> Self {
        Self {
            value: 1.0,
            base_scale,
        }
    }
}

pub fn run_ephemeral(world: &mut World, time: &Time) {
    let mut to_despawn = Vec::new();
    for (e, ephermal) in world.query::<With<&mut Ephemeral, ()>>().iter() {
        ephermal.0 -= time.fixed_time_step_seconds as f32;
        if ephermal.0 < 0.0 {
            to_despawn.push(e);
        }
    }

    for (e, (transform, shrink)) in world.query::<(&mut Transform, &mut Shrink)>().iter() {
        shrink.value -= time.fixed_time_step_seconds as f32;
        transform.scale = Vec3::fill(shrink.value * shrink.base_scale);
        if shrink.value < 0.0 {
            to_despawn.push(e);
        }
    }

    for e in to_despawn {
        world.despawn(e).unwrap();
    }
}

fn serialize_world(world: &mut World) -> String {
    let mut output = String::new();
    for (_e, (transform, _laser)) in world.query::<(&Transform, &Laser)>().iter() {
        output += &format!(
            "l {:?} {:?} {:?} {:?}\n",
            transform.position.x,
            transform.position.y,
            transform.rotation.to_angle_axis().0,
            transform.scale.x
        );
    }

    for (_e, (transform, _laser)) in world.query::<(&Transform, &Cannon)>().iter() {
        output += &format!(
            "c {:?} {:?} {:?} {:?}\n",
            transform.position.x,
            transform.position.y,
            transform.rotation.to_angle_axis().0,
            transform.scale.x
        );
    }

    for (_e, (transform, _laser)) in world.query::<(&Transform, &VictoryOrb)>().iter() {
        output += &format!(
            "v {:?} {:?} {:?} {:?}\n",
            transform.position.x,
            transform.position.y,
            transform.rotation.to_angle_axis().0,
            transform.scale.x
        );
    }
    for (_e, (transform, _laser)) in world.query::<(&Transform, &EnvironmentChunk)>().iter() {
        output += &format!(
            "b {:?} {:?} {:?} {:?}\n",
            transform.position.x,
            transform.position.y,
            transform.rotation.to_angle_axis().0,
            transform.scale.x
        );
    }

    for (_e, (transform, _laser)) in world.query::<(&Transform, &Splitter)>().iter() {
        output += &format!(
            "s {:?} {:?} {:?} {:?}\n",
            transform.position.x,
            transform.position.y,
            transform.rotation.to_angle_axis().0,
            transform.scale.x
        );
    }
    output
}

fn deserialize_world(
    string: &str,
    world: &mut World,
    rapier_integration: &mut RapierIntegration,
    scene_info: &SceneInfo,
    meshes: &AssetStore<Mesh>,
) {
    let materials = &scene_info.materials;
    for line in string.trim().split('\n') {
        let mut values = line.split_ascii_whitespace();
        let first_char = values.next();

        let x: f32 = values.next().unwrap().parse().unwrap();
        let y: f32 = values.next().unwrap().parse().unwrap();

        let position = Vec3::new(x, y, 0.0);

        let r: f32 = values.next().unwrap().parse().unwrap();

        let s: f32 = values.next().unwrap().parse().unwrap();

        let transform = Transform::new()
            .with_position(position)
            .with_rotation(Quat::from_angle_axis(r, Vec3::Z))
            .with_scale(Vec3::fill(s));

        match first_char {
            Some("l") => create_laser(world, rapier_integration, materials, transform),
            Some("c") => create_cannon(world, rapier_integration, materials, transform),
            Some("v") => create_victory_orb(world, rapier_integration, materials, transform),
            Some("b") => create_environment_chunk(world, rapier_integration, materials, transform),
            Some("s") => create_splitter(world, scene_info, meshes, rapier_integration, transform),

            Some(_) => {
                panic!()
            }
            None => {}
        }
    }
}

fn save_level(world: &mut World) {
    // Save the level when editing.
    let level = serialize_world(world);
    klog::log!("SAVING LEVEL");
    klog::log!("{}", level);
    std::fs::write("src/level.txt", level.as_bytes()).unwrap();
}

fn main() {
    App::default()
        .with_resource(InitialSettings {
            color_space: koi_graphics_context::ColorSpace::SRGB,
            window_width: 1200,
            window_height: 1200,
            ..Default::default()
        })
        .setup_and_run(|world, resources| {
            camera_controls::initialize_plugin(resources);

            setup(world, resources);

            let mut random = Random::new();

            move |event, world, resources| {
                koi3::transform_plugin::update_global_transforms(event, world, resources);
                match event {
                    Event::Draw => {
                        run_shake_child(world, resources);

                        let mut scene_info = resources.get::<SceneInfo>();
                        let screen_shake_amount = &mut scene_info.screen_shake_amount;
                        let screen_shake = Vec2::new(
                            random.range_f32(-*screen_shake_amount..*screen_shake_amount),
                            random.range_f32(-*screen_shake_amount..*screen_shake_amount),
                        );

                        let mut q = world.query::<(&mut Transform, &Camera)>();
                        let mut iter = q.iter();
                        let (camera_transform, camera) = iter.next().unwrap().1;
                        camera_transform.position =
                            screen_shake.extend(camera_transform.position.z);
                        *screen_shake_amount *= 0.94;

                        /*
                        // Handle the powered up meter
                        {
                            let total_power: f32 =
                                world.query::<&VictoryOrb>().iter().map(|v| v.1.power).sum();

                            let percent_of_max_power = total_power / scene_info.max_power;
                            let mut q = world.query::<(&mut Transform, &PoweredUpBar)>();
                            let mut i = q.iter();
                            let powered_up_bar = i.next().unwrap().1 .0;

                            let camera_height = match camera.projection_mode {
                                ProjectionMode::Orthographic { height, .. } => height,
                                _ => unimplemented!(),
                            };

                            let view_size = resources.get::<kapp::Window>().size();
                            let aspect_ratio = view_size.1 as f32 / view_size.0 as f32;
                            let camera_width_world_space = camera_height * aspect_ratio;
                            powered_up_bar.scale.x =
                                camera_width_world_space * percent_of_max_power;
                            powered_up_bar.scale.y = 0.5;
                            powered_up_bar.position.y =
                                -camera_height / 2.0 + powered_up_bar.scale.y * 0.5;

                            if percent_of_max_power >= 1.0 {
                                // Powered up
                            }
                        }
                        */
                    }
                    Event::FixedUpdate => {
                        {
                            if resources.get::<Input>().key_down(Key::E) {
                                setup(world, resources);
                                return;
                            }

                            #[cfg(debug_assertions)]
                            {
                                /*
                                if resources.get::<Input>().key_down(Key::S) {
                                    let level = serialize_world(world);
                                    klog::log!("SAVING LEVEL");
                                    klog::log!("{}", level);
                                    std::fs::write("src/level.txt", level.as_bytes()).unwrap();
                                }
                                */

                                let mut rapier_integration = resources.get::<RapierIntegration>();
                                let mut scene_info = resources.get::<SceneInfo>();

                                if !scene_info.running {
                                    if resources.get::<Input>().key_down(Key::Backspace) {
                                        if let Some(moving_entity) = scene_info.moving_entity {
                                            world.despawn(moving_entity).unwrap();
                                            scene_info.moving_entity = None;

                                            #[cfg(debug_assertions)]
                                            if !scene_info.running {
                                                println!("STEPPING");
                                                rapier_integration.step(world);

                                                save_level(world);
                                            }
                                        }
                                    }

                                    if resources.get::<Input>().key(Key::R) {
                                        if let Some(moving_entity) = scene_info.moving_entity {
                                            {
                                                let rotation = &mut world
                                                    .get::<&mut Transform>(moving_entity)
                                                    .unwrap()
                                                    .rotation;
                                                *rotation = *rotation
                                                    * Quat::from_angle_axis(
                                                        resources.get::<Input>().scroll().1 as f32
                                                            * 0.05,
                                                        Vec3::Z,
                                                    );
                                            }
                                            #[cfg(debug_assertions)]
                                            if !scene_info.running {
                                                println!("STEPPING");
                                                rapier_integration.step(world);
                                                save_level(world);
                                            }
                                        }
                                    }

                                    if resources.get::<Input>().key_down(Key::B) {
                                        create_environment_chunk(
                                            world,
                                            &mut rapier_integration,
                                            &scene_info.materials,
                                            Transform::new(),
                                        );
                                    }
                                }
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

                                    #[cfg(debug_assertions)]
                                    if !scene_info.running {
                                        println!("STEPPING");
                                        rapier_integration.step(world);
                                        save_level(world);
                                    }
                                }
                                scene_info.moving_entity = None;
                            }

                            let pointer_position = {
                                let (x, y) = input.pointer_position();
                                let mut q = world.query::<(&mut GlobalTransform, &Camera)>();
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
                                if !world.contains(moving_entity) {
                                    scene_info.moving_entity = None;
                                } else {
                                    let position = &mut world
                                        .get::<&mut Transform>(moving_entity)
                                        .unwrap()
                                        .position;
                                    *position = (pointer_position - scene_info.moving_offset)
                                        .extend(position.z);
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
                            }

                            if input.pointer_button_down(PointerButton::Primary) {
                                rapier_integration.query_pipeline.intersections_with_point(
                                    &rapier_integration.rigid_body_set,
                                    &rapier_integration.collider_set,
                                    &point![pointer_position.x, pointer_position.y],
                                    QueryFilter::default()
                                        .predicate(&|_, c: &Collider| c.parent().is_some()),
                                    |handle| {
                                        let collider =
                                            rapier_integration.collider_set.get(handle).unwrap();
                                        if collider.user_data != 0 {
                                            let entity =
                                                Entity::from_bits(collider.user_data as _).unwrap();

                                            if world.get::<&Movable>(entity).is_ok() {
                                                scene_info.moving_offset = pointer_position
                                                    - world
                                                        .get::<&mut Transform>(entity)
                                                        .unwrap()
                                                        .position
                                                        .xy();

                                                scene_info.moving_entity = Some(entity);
                                            }
                                        }
                                        false
                                    },
                                );
                            }
                        }

                        if resources.get::<SceneInfo>().running {
                            if resources.get::<SceneInfo>().freeze_time <= 0.0 {
                                run_ephemeral(world, &*resources.get::<Time>());

                                {
                                    let time = resources.get::<Time>();

                                    for (_, (chargable, activated)) in
                                        world.query::<(&mut Chargable, &mut Activated)>().iter()
                                    {
                                        if chargable.charged {
                                            chargable.charged = false;
                                        } else {
                                            chargable.amount -=
                                                1.0 * time.fixed_time_step_seconds as f32;
                                        }
                                        if chargable.amount > 1.0 {
                                            activated.this_frame = !activated.activated;
                                            activated.activated = true;
                                            if activated.reset_charge_on_activate {
                                                chargable.amount = 0.0
                                            }
                                            //chargable.amount = 0.0;
                                        } else {
                                            activated.this_frame = false;
                                            activated.activated = false;
                                        }
                                        chargable.amount = chargable.amount.clamp(0.0, 1.0);
                                    }

                                    for (_, (chargable, scale_child_to_chargable)) in
                                        world.query::<(&Chargable, &ScaleChildToChargable)>().iter()
                                    {
                                        world
                                            .get::<&mut Transform>(scale_child_to_chargable.child)
                                            .unwrap()
                                            .scale = Vec3::fill(animation_curves::smooth_step(
                                            chargable.amount,
                                        ));
                                    }
                                }

                                run_charge_on_timer(world, resources);
                                run_laser(world, resources);
                                run_cannon(world, resources);
                                run_victory_orb(world, resources);

                                resources.get::<RapierIntegration>().step(world);
                            } else {
                                resources.get::<SceneInfo>().freeze_time -=
                                    resources.get::<Time>().fixed_time_step_seconds as f32;
                            }
                        }
                    }

                    _ => {}
                }
                camera_controls::update_camera_controls(event, world, resources);
            }
        });
}
