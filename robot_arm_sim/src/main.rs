use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use robot_arm_core::kinematics::{RobotModel, DHParam};
use robot_arm_core::simulator::SimulatorState;
use std::f32::consts::PI;

#[derive(Component)]
struct RobotJoint {
    index: usize,
    axis_type: JointType, 
}

enum JointType {
    Revolute,
    Prismatic, // Assuming all revolute for now for SIA30D
}

#[derive(Resource)]
struct RobotResource {
    model: RobotModel,
    joint_angles: Vec<f64>,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.1)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "SpecKit Robot Arm Simulator".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin)
        .insert_resource(RobotResource {
            model: RobotModel::new_sia30d(),
            joint_angles: vec![0.0; 7],
        })
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, spawn_robot)
        .add_systems(Update, update_robot_joints)
        .run();
}

fn setup_scene(
    mut commands: Commands, 
    mut meshes: ResMut<Assets<Mesh>>, 
    mut materials: ResMut<Assets<StandardMaterial>>
) {
    // Camera
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(2.0, 2.5, 3.0).looking_at(Vec3::new(0.0, 0.5, 0.0), Vec3::Y),
        ..default()
    });

    // Light
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    // Ground Plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(Plane3d::default().mesh().size(10.0, 10.0)),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3)),
        ..default()
    });
}

fn spawn_robot(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    robot_res: Res<RobotResource>,
) {
    let material_joint = materials.add(Color::rgb(0.7, 0.7, 0.7));
    let material_link = materials.add(Color::rgb(0.2, 0.2, 0.8));

    // Base
    let mut current_entity = commands.spawn(PbrBundle {
        mesh: meshes.add(Cylinder::new(0.1, 0.05)),
        material: material_joint.clone(),
        transform: Transform::from_xyz(0.0, 0.025, 0.0),
        ..default()
    }).id();

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
            SpatialBundle {
                transform: Transform::IDENTITY, // Will be updated
                ..default()
            },
            RobotJoint { index: i, axis_type: JointType::Revolute },
        )).id();
        
        commands.entity(current_entity).add_child(joint_entity);
        
        // Add visual link geometry attached to this joint frame
        // The visual should represent the physical link connecting to the NEXT joint.
        // In DH, the link goes along X axis (by 'a') and Z axis (by 'd').
        
        // Visual for "d" offset (along Z axis of current frame)
        if param.d.abs() > 0.01 {
            commands.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(0.05, param.d as f32)),
                material: material_link.clone(),
                transform: Transform::from_xyz(0.0, 0.0, param.d as f32 / 2.0)
                    .with_rotation(Quat::from_rotation_x(PI / 2.0)), 
                ..default()
            }).set_parent(joint_entity);
        }

        // After Z-translation (d) and Z-rotation (theta), we have the X-translation (a) and X-rotation (alpha).
        // Standard composed transform for frame {i} is RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha).
        // In Bevy hierarchy, we can split this:
        // Parent -> [Joint Rotate Z (theta)] -> [Static Offset Z(d) * X(a) * RotX(alpha)] -> Next Joint
        // So `joint_entity` handles the dynamic rotation `theta`.
        // We need an intermediate "Link Frame" that handles the static parts (d, a, alpha) to get to the next joint origin.
        
        let link_static_transform = Transform::from_translation(Vec3::new(param.a as f32, 0.0, param.d as f32))
            * Transform::from_rotation(Quat::from_rotation_x(param.alpha as f32));

        let next_base = commands.spawn(SpatialBundle {
            transform: link_static_transform,
            ..default()
        }).set_parent(joint_entity).id();
        
        // Visual for "a" offset (along X axis of potentially rotated frame)
        if param.a.abs() > 0.01 {
             commands.spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(0.04, param.a as f32)), // length a
                material: material_link.clone(),
                transform: Transform::from_xyz(param.a as f32 / 2.0, 0.0, param.d as f32) // Position is tricky if we want to draw lines
                    .with_rotation(Quat::from_rotation_z(PI / 2.0)), // Align cylinder with X axis
                ..default()
            }).set_parent(joint_entity); // Attached to joint frame, before alpha rotation? 
            // Correct visualization of DH is hard without segmenting.
            // Let's stick to simple spheres at origins for debug first.
        }
        
        commands.spawn(PbrBundle {
            mesh: meshes.add(Sphere::new(0.08).mesh().ico(3).unwrap()),
            material: material_joint.clone(),
            ..default()
        }).set_parent(joint_entity);

        current_entity = next_base;
    }
}

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
