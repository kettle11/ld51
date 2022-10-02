//! Camera controls to be used by the editor or to quickly get a 3D camera up and running.
use koi3::*;

pub fn initialize_plugin(resources: &mut Resources) {
    resources
        .get_mut::<EventHandlers>()
        .add_handler(Event::FixedUpdate, update_camera_controls);
}

#[derive(Clone)]
pub enum CameraControlsMode {
    Fly,
    Orbit,
}

#[derive(Clone)]
pub struct CameraControls {
    velocity: Vec3,
    pub max_speed: f32,
    pub rotation_sensitivity: f32,
    pub mode: CameraControlsMode,
    pub rotate_button: PointerButton,
    pub panning_mouse_button: Option<PointerButton>,
    pub panning_scale: f32,
    pub touch_rotate_enabled: bool,
    pub enabled: bool,
    pub orbit_target: Vec3,
    pub target_camera: Entity,
}

impl CameraControls {
    pub fn new(target_camera: Entity) -> Self {
        Self {
            velocity: Vec3::ZERO,
            max_speed: 5.0,
            rotation_sensitivity: 1.5,
            mode: CameraControlsMode::Fly,
            rotate_button: PointerButton::Secondary,
            panning_mouse_button: Some(PointerButton::Auxillary),
            panning_scale: 1.0,
            touch_rotate_enabled: true,
            enabled: true,
            orbit_target: Vec3::ZERO,
            target_camera,
        }
    }
}

pub fn update_camera_controls(
    _event: &Event,
    world: &mut koi_ecs::World,
    resources: &mut Resources,
) {
    let input = resources.get::<Input>();
    let time = resources.get::<Time>();

    /*
      input: &Input,
    time: &Time,
    mut query: Query<(&mut CameraControls, &mut Camera, &mut Transform)>, */
    let mut query = world.query::<(&mut CameraControls, &mut Transform)>();
    for (_, (controls, transform)) in query.iter() {
        let mut camera = world.get::<&mut Camera>(controls.target_camera).unwrap();

        if !controls.enabled {
            continue;
        }
        let (x, y) = input.mouse_motion();
        let difference: Vec2 = Vec2::new(x as f32, y as f32) / 1000.;

        let mut direction = Vec3::ZERO;

        if input.key(Key::W) || input.key(Key::Up) {
            direction += transform.up();
        }

        if input.key(Key::S) || input.key(Key::Down) {
            direction += transform.down();
        }

        if input.key(Key::A) || input.key(Key::Left) {
            direction += transform.left();
        }

        if input.key(Key::D) || input.key(Key::Right) {
            direction += transform.right();
        }

        /*
        // Switch quickly to top
        if input.key(Key::Digit1) {
            transform.rotation =
                Quaternion::from_angle_axis(-core::f32::consts::TAU * 0.25, Vec3::X);
        }
        */

        if direction != Vec3::ZERO {
            controls.velocity = direction.normalized() * controls.max_speed;
        } else {
            controls.velocity = Vec3::ZERO;
        }

        if controls.velocity.length() > controls.max_speed {
            controls.velocity = controls.velocity.normalized() * controls.max_speed;
        }

        // Rotation
        let (mut pitch, mut yaw, rotating) = if input.pointer_button(controls.rotate_button) {
            let scale = 4.0;
            (-difference[1] * scale, -difference[0] * scale, true)
        } else {
            (0.0, 0.0, false)
        };

        let mut pan = Vec2::ZERO;

        // Panning
        //  if input.key(Key::LeftShift) || input.key(Key::RightShift) || input.key(Key::Shift) {
        //      let scale = 0.005;
        //      pitch = -input.scroll().1 as f32 * scale;
        //      yaw = -input.scroll().0 as f32 * scale;
        //  } else {
        let scale = 0.0125;
        pan.x -= -input.scroll().0 as f32 * scale;
        pan.y -= -input.scroll().1 as f32 * scale;
        // };

        if controls.touch_rotate_enabled && input.touch_state.touches.len() == 1 {
            if let Some((_, touch)) = input.touch_state.touches.iter().next() {
                let diff = touch.delta();
                pitch -= diff.y / 400.;
                yaw -= diff.x / 400.;
            }
        }

        if let Some(panning_mouse_button) = controls.panning_mouse_button {
            if input.pointer_button(panning_mouse_button) {
                let scale = controls.panning_scale * 10.0;
                pan += difference * scale;
            }
        }

        pan += input.two_finger_pan() * 0.5;

        let left = transform.left();
        let up = transform.up();
        let offset = left * pan.x + up * pan.y;

        match &mut controls.mode {
            CameraControlsMode::Orbit => {
                controls.orbit_target += offset;
                transform.position += offset;
            }
            _ => {
                transform.position += offset;
            }
        };

        let mut pinch = input.pinch() * 3.0;

        if input.key(Key::Equal) {
            pinch = 0.3;
        }
        if input.key(Key::Minus) {
            pinch = -0.3;
        }
        match camera.projection_mode {
            ProjectionMode::Orthographic {
                height,
                z_near,
                z_far,
            } => {
                camera.projection_mode = ProjectionMode::Orthographic {
                    height: (height - pinch).max(10.0).min(50.0),
                    z_near,
                    z_far,
                }
            }
            _ => unimplemented!(),
        }

        /*
        let pointer_position = input.pointer_position();
        let zoom_direction = camera.view_to_ray(
            transform,
            pointer_position.0 as f32,
            pointer_position.1 as f32,
        );
        transform.position += zoom_direction.direction * pinch * 3.;
        */

        // let rotation_pitch = Quat::from_yaw_pitch_roll(0., pitch, 0.);
        // let rotation_yaw = Quat::from_yaw_pitch_roll(yaw, 0., 0.);

        // transform.rotation = rotation_yaw * transform.rotation * rotation_pitch;
        transform.position += controls.velocity * time.fixed_time_step_seconds as f32;
        controls.orbit_target += controls.velocity * time.fixed_time_step_seconds as f32;
    }
}
