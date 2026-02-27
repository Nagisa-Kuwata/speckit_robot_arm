use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use egui_plot::{Line, Plot, PlotPoints};
use robot_arm_core::kinematics::RobotModel;
use robot_arm_core::collision::{CollisionDetector, CollisionType};
use robot_arm_core::simulator::SimulatorState;
use std::collections::VecDeque;
use std::f32::consts::PI;
use nalgebra::Vector3;
use std::fs::File;
use std::io::BufReader;
/// Sliding-window state history for real-time graphs.
#[derive(Resource)]
struct StateHistory {
    /// Per-joint deque of (t, pos_rad, vel_rad_s, acc_rad_s2).
    samples: Vec<VecDeque<(f64, f64, f64, f64)>>,
    /// How many seconds of history to keep.
    window_secs: f64,
    /// Which joint index to display in the graph window.
    graph_joint: usize,
    #[allow(dead_code)]
    show_graph: bool,
    prev_q_dot: Vec<f64>,
}
impl StateHistory {
    fn new(num_joints: usize) -> Self {
        Self {
            samples: vec![VecDeque::new(); num_joints],
            window_secs: 5.0,
            graph_joint: 0,
            show_graph: true,
            prev_q_dot: vec![0.0; num_joints],
        }
    }
    fn record(&mut self, t: f64, q: &[f64], q_dot: &[f64], dt: f64) {
        let n = q.len().min(self.samples.len());
        for i in 0..n {
            let q_ddot = if dt > 1e-9 { (q_dot[i] - self.prev_q_dot[i]) / dt } else { 0.0 };
            self.samples[i].push_back((t, q[i], q_dot[i], q_ddot));
        }
        self.prev_q_dot = q_dot.to_vec();
        // Remove samples outside the sliding window.
        let cutoff = t - self.window_secs;
        for buf in &mut self.samples {
            while buf.front().map_or(false, |(ts, ..)| *ts < cutoff) {
                buf.pop_front();
            }
        }
    }
}
#[derive(Component)]
struct RobotJoint {
    index: usize,
    #[allow(dead_code)]
    axis_type: JointType,
}
#[allow(dead_code)]
enum JointType {
    Revolute,
    Prismatic, // Reserved for future prismatic joints
}
#[derive(Component)]
struct IkTarget;
#[derive(Resource)]
struct RobotResource {
    model: RobotModel,
    joint_angles: Vec<f64>,
    target_position: [f64; 3], // For IK
    use_ik: bool,
    show_axes: bool,
    collision_detector: CollisionDetector,
    collision_status: CollisionType,
    simulator: SimulatorState,
    /// Whether IK was enabled last frame (edge-trigger flag).
    ik_was_enabled: bool,
}
fn load_config() -> RobotModel {
    let path = "config.yaml";
    match File::open(path) {
        Ok(file) => {
            let reader = BufReader::new(file);
            match serde_yaml::from_reader(reader) {
                Ok(model) => {
                    info!("Successfully loaded config from {}", path);
                    model
                },
                Err(e) => {
                    warn!("Failed to parse {}: {}. Using default SIA30D model.", path, e);
                    RobotModel::new_sia30d()
                }
            }
        },
        Err(e) => {
            warn!("Could not open {}: {}. Using default SIA30D model.", path, e);
            RobotModel::new_sia30d()
        }
    }
}
fn main() {
    let model = load_config();
    let simulator = SimulatorState::new(model.clone());
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.1, 0.1, 0.1)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SpecKit Robot Arm Simulator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin)
        // .add_plugins(TransformGizmoPlugin::default()) // Add Gizmo Plugin
        .insert_resource(RobotResource {
            model: model.clone(),
            joint_angles: vec![0.0; 7],
            target_position: [0.5, 0.5, 0.5], // Default target
            use_ik: false,
            show_axes: true,
            collision_detector: CollisionDetector::new(0.0, 0.2), // Ground at 0.0, 20cm min link dist
            collision_status: CollisionType::None,
            simulator,
            ik_was_enabled: false,
        })
        .insert_resource(StateHistory::new(7))
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, spawn_robot)
        .add_systems(Startup, spawn_ik_target)
        .add_systems(Update, (
            physics_system,
            update_robot_joints,
            update_ik_target_visuals,
            ui_system,
            graph_window_system,
            ik_system,
            draw_axes_system,
        ))
        .run();
}
fn physics_system(
    mut robot_res: ResMut<RobotResource>,
    mut history: ResMut<StateHistory>,
    time: Res<Time>,
) {
    let dt = time.delta_secs() as f64;
    robot_res.simulator.update(dt);
    // Record state history for graphs.
    let t = robot_res.simulator.time;
    let q = robot_res.simulator.q.clone();
    let q_dot = robot_res.simulator.q_dot.clone();
    history.record(t, &q, &q_dot, dt);
    // Sync joint angles for rendering and UI.
    robot_res.joint_angles = q;
}
/// Real-time state graph window (position, velocity, acceleration plots).
fn graph_window_system(
    mut contexts: EguiContexts,
    mut history: ResMut<StateHistory>,
) {
    egui::Window::new("State Graph")
        .default_width(480.0)
        .show(contexts.ctx_mut(), |ui| {
            ui.horizontal(|ui| {
                ui.label("Joint:");
                for j in 0..7 {
                    ui.selectable_value(&mut history.graph_joint, j, format!("J{}", j + 1));
                }
                ui.separator();
                ui.label("Window:");
                ui.add(egui::DragValue::new(&mut history.window_secs).range(1.0..=30.0).suffix("s"));
            });
            let idx = history.graph_joint;
            let samples: Vec<(f64, f64, f64, f64)> = history.samples[idx].iter().cloned().collect();
            if samples.is_empty() {
                ui.label("No data yet ? start simulation.");
                return;
            }
            let t0 = samples.first().map(|(t, ..)| *t).unwrap_or(0.0);
            // Position graph (radians).
            ui.label("Position [rad]");
            let pos_pts: PlotPoints = samples.iter()
                .map(|(t, q, _, _)| [t - t0, *q])
                .collect();
            Plot::new(format!("pos_{}", idx))
                .height(80.0)
                .show(ui, |p| p.line(Line::new(pos_pts).name("q [rad]")));
            // Velocity graph.
            ui.label("Velocity [rad/s]");
            let vel_pts: PlotPoints = samples.iter()
                .map(|(t, _, qd, _)| [t - t0, *qd])
                .collect();
            Plot::new(format!("vel_{}", idx))
                .height(80.0)
                .show(ui, |p| p.line(Line::new(vel_pts).name("dq/dt [rad/s]")));
            // Acceleration graph.
            ui.label("Acceleration [rad/s^2]");
            let acc_pts: PlotPoints = samples.iter()
                .map(|(t, _, _, qdd)| [t - t0, *qdd])
        // -- Joint Control -------------------------------------------
                .collect();
        ui.small("Adjusting the position slider resets velocity to 0.");
            Plot::new(format!("acc_{}", idx))
                .height(80.0)
                .show(ui, |p| p.line(Line::new(acc_pts).name("d2q/dt2 [rad/s^2]")));
        });
}
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    _materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(2.0, 2.5, 3.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
        GlobalTransform::default(),
    ));
    // Light
    commands.spawn((
        PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
        GlobalTransform::default(),
        Visibility::default(),
        InheritedVisibility::default(),
        ViewVisibility::default(),
        // -- End Effector State (FK result) --------------------------
    ));
    // Ground Plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(10.0, 10.0))),
        // -- System Status -------------------------------------------
        Transform::default(),
        GlobalTransform::default(),
        Visibility::default(),
        InheritedVisibility::default(),
        ViewVisibility::default(),
    ));
}
fn spawn_robot(
        // -- Inverse Kinematics ------------------------------------
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_res: Res<RobotResource>,
) {
    let material_joint = materials.add(Color::srgb(0.7, 0.7, 0.7));
    let material_link = materials.add(Color::srgb(0.2, 0.2, 0.8));
    // Base
    let mut current_entity = commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.1, 0.05))),
        MeshMaterial3d(material_joint.clone()),
        Transform::from_xyz(0.0, 0.025, 0.0),
        GlobalTransform::default(),
        Visibility::default(),
        InheritedVisibility::default(),
        ViewVisibility::default(),
    )).id();
    // Iterate DH Params and build hierarchy
    // Note: Visualizing DH parameters directly with primitives is non-trivial because 
    // physics/math frames are abstract. We'll verify structure visually.
    
    // For now, we spawn just placeholders at origin to be moved by FK if needed,
    // or parenting them. Parenting is better for Forward Kinematics.
    // Each joint rotates around its Z axis in DH.
    
    for (i, param) in robot_res.model.dh_params.iter().enumerate() {
        // Create an entity for the Joint Frame
        // The transform of this entity relative to parent will be controlled by joint angle.
        
        let joint_entity = commands.spawn((
            Transform::IDENTITY, // Will be updated
            GlobalTransform::default(),
            Visibility::default(),
            InheritedVisibility::default(),
            ViewVisibility::default(),
            RobotJoint { index: i, axis_type: JointType::Revolute },
        )).id();
        
        commands.entity(current_entity).add_child(joint_entity);
        
        // Add visual link geometry attached to this joint frame
        // The visual should represent the physical link connecting to the NEXT joint.
        // In DH, the link goes along X axis (by 'a') and Z axis (by 'd').
        
        // Visual for "d" offset (along Z axis of current frame)
        if param.d.abs() > 0.01 {
            commands.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.05, param.d as f32))),
                MeshMaterial3d(material_link.clone()),
                Transform::from_xyz(0.0, 0.0, param.d as f32 / 2.0)
                    .with_rotation(Quat::from_rotation_x(PI / 2.0)),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
                ViewVisibility::default(),
            )).set_parent(joint_entity);
        }
        // After Z-translation (d) and Z-rotation (theta), we have the X-translation (a) and X-rotation (alpha).
        // Standard composed transform for frame {i} is RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha).
        // In Bevy hierarchy, we can split this:
        // Parent -> [Joint Rotate Z (theta)] -> [Static Offset Z(d) * X(a) * RotX(alpha)] -> Next Joint
        // So `joint_entity` handles the dynamic rotation `theta`.
        // We need an intermediate "Link Frame" that handles the static parts (d, a, alpha) to get to the next joint origin.
        
        let link_static_transform = Transform::from_translation(Vec3::new(param.a as f32, 0.0, param.d as f32))
            * Transform::from_rotation(Quat::from_rotation_x(param.alpha as f32));
        let next_base = commands.spawn((
            link_static_transform,
            GlobalTransform::default(),
            Visibility::default(),
            InheritedVisibility::default(),
            ViewVisibility::default(),
        )).set_parent(joint_entity).id();
        
        // Visual for "a" offset (along X axis of potentially rotated frame)
        if param.a.abs() > 0.01 {
             commands.spawn((
                Mesh3d(meshes.add(Cylinder::new(0.04, param.a as f32))),
                MeshMaterial3d(material_link.clone()),
                Transform::from_xyz(param.a as f32 / 2.0, 0.0, param.d as f32)
                    .with_rotation(Quat::from_rotation_z(PI / 2.0)),
                GlobalTransform::default(),
                Visibility::default(),
                InheritedVisibility::default(),
                ViewVisibility::default(),
            )).set_parent(joint_entity);
            // Correct visualization of DH is hard without segmenting.
            // Let's stick to simple spheres at origins for debug first.
        }
        
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.08).mesh().ico(3).unwrap())),
            MeshMaterial3d(material_joint.clone()),
            Transform::default(),
            GlobalTransform::default(),
            Visibility::default(),
        )).set_parent(joint_entity);
        current_entity = next_base;
    }
}
fn spawn_ik_target(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_res: Res<RobotResource>,
) {
    // Initial position
    let pos = Vec3::new(
        robot_res.target_position[0] as f32,
        robot_res.target_position[1] as f32,
        robot_res.target_position[2] as f32,
    );
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 1.0))),
        Transform::from_translation(pos),
        GlobalTransform::default(),
        Visibility::default(),
        InheritedVisibility::default(),
        ViewVisibility::default(),
        // GizmoTransformable,
        IkTarget,
    ));
}
fn update_ik_target_visuals(
    mut query: Query<&mut Transform, With<IkTarget>>,
    robot_res: Res<RobotResource>,
) {
    if let Ok(mut transform) = query.get_single_mut() {
        transform.translation = Vec3::new(
            robot_res.target_position[0] as f32,
            robot_res.target_position[1] as f32,
            robot_res.target_position[2] as f32,
        );
    }
}
// Update Gizmo/Target position if UI changed it
// (This requires caching or change detection, omitted for brevity but useful)
// For now, Gizmo is the source of truth if touched.
fn update_robot_joints(
    mut query: Query<(&mut Transform, &RobotJoint)>,
    robot_res: Res<RobotResource>,
) {
    for (mut transform, joint) in query.iter_mut() {
        let angle = robot_res.joint_angles[joint.index];
        // DH param theta rotation is around Z axis
        let offset = robot_res.model.dh_params[joint.index].theta_offset;
        transform.rotation = Quat::from_rotation_z((angle + offset) as f32);
    }
}
fn ui_system(
    mut contexts: EguiContexts,
    mut robot_res: ResMut<RobotResource>,
) {
    egui::Window::new("Robot Control")
        .default_width(420.0)
        .show(contexts.ctx_mut(), |ui| {
        // -- Joint Control -------------------------------------------
        ui.heading("Joint Control");
        ui.small("Adjusting the position slider resets velocity to 0.");
        let limits = robot_res.model.joint_limits.clone();
        for i in 0..7 {
            egui::CollapsingHeader::new(format!("Joint {} (J{})", i + 1, i + 1))
                .default_open(i == 0)
                .show(ui, |ui| {
                    let (min, max) = limits[i];
                    let current_rad = robot_res.simulator.q[i];
                    let current_vel = robot_res.simulator.q_dot[i];
                    // Position slider (deg control, shows both rad and deg).
                    let mut deg_val = current_rad.to_degrees();
                    ui.horizontal(|ui| {
                        ui.label("Position:");
                        let resp = ui.add(
                            egui::Slider::new(&mut deg_val, min.to_degrees()..=max.to_degrees())
                                .suffix(" deg")
                        );
                        ui.label(format!("{:.4} rad", current_rad));
                        if resp.changed() {
                            let new_rad = deg_val.to_radians();
                            let mut test_angles = robot_res.simulator.q.clone();
                            test_angles[i] = new_rad;
                            let collision = robot_res.collision_detector
                                .check_collision(&robot_res.model, &test_angles);
                            if matches!(collision, CollisionType::None) {
                                robot_res.simulator.q[i] = new_rad;
                                robot_res.simulator.q_dot[i] = 0.0;
                                robot_res.simulator.current_trajectory = None;
                                robot_res.joint_angles[i] = new_rad;
                                robot_res.collision_status = CollisionType::None;
                            } else {
                                robot_res.collision_status = collision;
                            }
                            robot_res.use_ik = false;
                        }
                    });
                    // Velocity input (rad/s).
                    let mut vel_val = current_vel;
                    ui.horizontal(|ui| {
                        ui.label("Velocity:");
                        if ui.add(
                            egui::DragValue::new(&mut vel_val)
                                .range(-10.0..=10.0)
                                .speed(0.01)
                                .suffix(" rad/s")
                        ).changed() {
                            robot_res.simulator.q_dot[i] = vel_val;
                            robot_res.simulator.current_trajectory = None;
                        }
                        ui.label(format!("{:.3} deg/s", vel_val.to_degrees()));
                    });
                });
        }
        ui.separator();
        // -- End Effector State (FK result) --------------------------
        ui.heading("End Effector (FK)");
        {
            let fk = robot_res.model.forward_kinematics(&robot_res.joint_angles);
            let pos = fk.column(3);
            ui.label(format!("X = {:.4} m  Y = {:.4} m  Z = {:.4} m",
                pos[0], pos[1], pos[2]));
        }
        ui.separator();
        // -- System Status -------------------------------------------
        ui.heading("System Status");
        match robot_res.collision_status {
            CollisionType::None => {
                ui.label(egui::RichText::new("? OK").color(egui::Color32::GREEN));
            },
            CollisionType::GroundCollision => {
                ui.label(egui::RichText::new("? GROUND COLLISION").color(egui::Color32::RED).strong());
            },
            CollisionType::SelfCollision => {
                ui.label(egui::RichText::new("? SELF COLLISION").color(egui::Color32::RED).strong());
            }
        }
        let sim_t = robot_res.simulator.time;
        let is_running = robot_res.simulator.running;
        ui.label(format!("sim t = {:.3} s  running = {}", sim_t, is_running));
        ui.separator();
        // -- Inverse Kinematics ------------------------------------
        ui.heading("Inverse Kinematics");
        ui.horizontal(|ui| {
            ui.checkbox(&mut robot_res.use_ik, "Enable IK");
            if robot_res.use_ik {
                ui.label(egui::RichText::new("(auto-solve on enable)").color(egui::Color32::YELLOW));
            }
        });
        ui.horizontal(|ui| {
            ui.label("Target X:");
            ui.add(egui::DragValue::new(&mut robot_res.target_position[0]).speed(0.01).suffix(" m"));
        });
        ui.horizontal(|ui| {
            ui.label("Target Y:");
            ui.add(egui::DragValue::new(&mut robot_res.target_position[1]).speed(0.01).suffix(" m"));
        });
        ui.horizontal(|ui| {
            ui.label("Target Z:");
            ui.add(egui::DragValue::new(&mut robot_res.target_position[2]).speed(0.01).suffix(" m"));
        });
        if ui.button("Solve IK & Move").clicked() {
            solve_ik(&mut robot_res);
        }
        ui.separator();
        ui.checkbox(&mut robot_res.show_axes, "Show Axes");
    });
}
fn ik_system(mut robot_res: ResMut<RobotResource>) {
    let currently_enabled = robot_res.use_ik;
    // Edge-trigger: solve IK only on the rising edge (enable transition).
    if currently_enabled && !robot_res.ik_was_enabled {
        solve_ik(&mut robot_res);
    }
    robot_res.ik_was_enabled = currently_enabled;
}
fn solve_ik(robot_res: &mut RobotResource) {
    let target = Vector3::new(
        robot_res.target_position[0],
        robot_res.target_position[1],
        robot_res.target_position[2]
    );
    
    // Call IK solver from core
    if let Some(new_angles) = robot_res.model.inverse_kinematics_position(&target, &robot_res.joint_angles) {
        // Check collision for the solution
        let collision = robot_res.collision_detector.check_collision(&robot_res.model, &new_angles);
        
        if matches!(collision, CollisionType::None) {
            // robot_res.joint_angles = new_angles; // Old instant move
            robot_res.simulator.set_target(&new_angles, 2.0); // Move over 2 seconds
            robot_res.collision_status = CollisionType::None;
        } else {
            robot_res.collision_status = collision;
            // IK solution found but invalid due to collision. 
            // Do not update.
        }
    } else {
        // IK failed to find solution (unreachable etc)
        // Only change status if it wasn't a collision error
        // Or maybe add an IK failure status? Ignoring for now.
    }
}
fn draw_axes_system(
    mut gizmos: Gizmos,
    query: Query<(&GlobalTransform, &RobotJoint)>, 
    robot_res: Res<RobotResource>,
) {
    if !robot_res.show_axes { return; }
    for (transform, _joint) in query.iter() {
        let translation = transform.translation();
        let rotation = transform.to_scale_rotation_translation().1;
        
        let axis_length = 0.2;
        
        // X axis (Red)
        gizmos.line(
            translation,
            translation + rotation * Vec3::X * axis_length,
            Color::srgb(1.0, 0.0, 0.0),
        );
        
        // Y axis (Green)
        gizmos.line(
            translation,
            translation + rotation * Vec3::Y * axis_length,
            Color::srgb(0.0, 1.0, 0.0),
        );
        
        // Z axis (Blue)
        gizmos.line(
            translation,
            translation + rotation * Vec3::Z * axis_length,
            Color::srgb(0.0, 0.0, 1.0),
        );
    }
}
