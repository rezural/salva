extern crate nalgebra as na;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use nphysics_testbed3d::objects::FluidRenderingMode;
use nphysics_testbed3d::Testbed;
use salva3d::coupling::{ColliderCouplingSet, CouplingMethod};
use salva3d::object::{Boundary, Fluid};
use salva3d::solver::{DFSPHSolver, XSPHViscosity, Akinci2013SurfaceTension};
use salva3d::LiquidWorld;
use std::f32;
use std::fs::File;
use std::io::Write;
use std::path;
use std::time::{Instant};
use uuid::Uuid;

use std::fs;

use serde_json;
#[path = "./helper.rs"]
mod helper;

pub fn init_world(testbed: &mut Testbed) {

    // Fluid sim world dimensions
    let (width, height, depth, particle_radius) = (100.0, 15.0, 100.0, 1.0 / 8.0);

    let subdivs = ((width/particle_radius) as i32, (height/particle_radius) as i32, (depth/particle_radius) as i32);

    let run_uuid = Uuid::new_v4()
        .to_hyphenated()
        .to_string()
        .to_owned();

    let fluid_density: f32 = 1000.0;

    println!("{{\"width\": {}, \"height\": {}, \"depth\": {},\"particle_radius\": {},\"subdivs\": [{},{},{}], \"total_particles\": \"{}\"}}",
        width, height, depth, particle_radius, subdivs.0, subdivs.1, subdivs.2, subdivs.0 * subdivs.1 * subdivs.2);

    /*
     * World
     */
    let mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0.0, -20.81, 0.0));
    let geometrical_world = DefaultGeometricalWorld::new();
    let mut bodies = DefaultBodySet::new();
    let mut colliders = DefaultColliderSet::new();
    let joint_constraints = DefaultJointConstraintSet::new();
    let force_generators = DefaultForceGeneratorSet::new();

    // Parameters of the ground.
    let ground_thickness: f32 = 0.5;
    // This needs to be slightly larger than half the width
    let ground_half_width: f32 = width * 0.53;
    let ground_half_height: f32 = height;

    /*
     * Liquid world.
     */
    let solver: DFSPHSolver<f32> = DFSPHSolver::new();
    let mut liquid_world = LiquidWorld::new(solver, particle_radius, 2.0);
    let mut coupling_manager = ColliderCouplingSet::new();

    let mut fluid: Fluid<f32> = helper::box_of_fluid(width, height, depth, particle_radius, fluid_density);

    fluid.transform_by(&Isometry3::translation(
        0.0,
        ground_thickness + height / 2.0 + particle_radius * 1.1,
        0.0,
    ));

    println!("size: {}", fluid.size_in_bytes());

    let viscosity = XSPHViscosity::new(0.05, 0.05);
    fluid.nonpressure_forces.push(Box::new(viscosity));

    let tension = Akinci2013SurfaceTension::new(0.1, 1.0);
    fluid.nonpressure_forces.push(Box::new(tension));

    let fluid_handle = liquid_world.add_fluid(fluid);
    testbed.set_fluid_color(fluid_handle, Point3::new(0.8, 0.7, 1.0));

    /*
     * Ground.
     */
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        ground_half_width,
        ground_thickness,
        ground_half_width,
    )));
    let wall_shape = ShapeHandle::new(Cuboid::new(Vector3::new(
        ground_thickness,
        ground_half_height,
        ground_half_width,
    )));

    let ground_handle = bodies.insert(Ground::new());

    let wall_poses = [
        Isometry3::new(
            Vector3::new(0.0, ground_half_height, ground_half_width),
            Vector3::y() * (f32::consts::PI / 2.0),
        ),
        Isometry3::new(
            Vector3::new(0.0, ground_half_height, -ground_half_width),
            Vector3::y() * (f32::consts::PI / 2.0),
        ),
        Isometry3::translation(ground_half_width, ground_half_height, 0.0),
        Isometry3::translation(-ground_half_width, ground_half_height, 0.0),
    ];

    for pose in wall_poses.iter() {
        let co = ColliderDesc::new(wall_shape.clone())
            .position(*pose)
            .build(BodyPartHandle(ground_handle, 0));
        let co_handle = colliders.insert(co);
        let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));

        coupling_manager.register_coupling(
            bo_handle,
            co_handle,
            CouplingMethod::DynamicContactSampling,
        );
    }

    let co = ColliderDesc::new(ground_shape).build(BodyPartHandle(ground_handle, 0));
    let co_handle = colliders.insert(co);
    let bo_handle = liquid_world.add_boundary(Boundary::new(Vec::new()));

    coupling_manager.register_coupling(
        bo_handle,
        co_handle,
        CouplingMethod::DynamicContactSampling,
    );

    let mut last_t = 0.0;
    let mut simulation_step = 0;
    let mut done_first = false;

    let mut wall_times: Vec<Instant> = Vec::new();
    wall_times.push(Instant::now());

    let state_save_path = path::PathBuf::from(format!("./runs/{}", run_uuid));
    // FIXME THis shouldnt be ok(); It should die gracefully
    fs::create_dir_all(&state_save_path).ok();

    println!("{{\"created_run_dir\": \"{}\"}}", state_save_path.as_path().to_str().unwrap());

    testbed.add_callback_with_fluids(move |liquid_world: & mut LiquidWorld<f32>, _, _, _, _, _, _, t| {

        save_state(&state_save_path, liquid_world, simulation_step);
        simulation_step = simulation_step + 1;
        
        let fluid = liquid_world.fluids_mut().get_mut(fluid_handle).unwrap();

        let last_wall_time = wall_times.last().unwrap();
        let duration = last_wall_time.elapsed();
        wall_times.push(Instant::now());

        println!("{{\"duration: \"{:?}\"}},", duration);
        if !done_first {
          if t > 0.1 {
              done_first = true;
          } else {
              return;
          }
        } else if t - last_t < 4.0 {
            return;
        }
        
        last_t = t;

        let shift = Vector3::new( - (width / 2.6 ), height + height / 2.0 , 0.0);

        let particles = &helper::box_of_particles(width / 6.0, height / 2.0, depth, particle_radius,shift);

        let initial_particle_speed = 0.0;

        let mut velocities = Vec::new();
        for _ in 0..particles.to_vec().len() {
            velocities.push(Vector3::y() * initial_particle_speed)
        }

        fluid.add_particles(particles, Some(&velocities));
    });

    /*
     * Set up the testbed.
     */
    testbed.set_body_wireframe(ground_handle, true);
    testbed.set_ground_handle(Some(ground_handle));

    testbed.set_world(
        mechanical_world,
        geometrical_world,
        bodies,
        colliders,
        joint_constraints,
        force_generators,
    );
    testbed.set_number_of_steps_per_frame(8);
    testbed.set_liquid_world(liquid_world, coupling_manager);
    testbed.set_fluid_rendering_mode(FluidRenderingMode::VelocityColor { min: 0.0, max: 5.0 });
    testbed.mechanical_world_mut().set_timestep(1.0 / 200.0);
    testbed.look_at(Point3::new(120.0, 50.0, 120.0), Point3::origin());
}


fn save_state(container_dir: &std::path::PathBuf, fluid: &LiquidWorld<f32>, simulation_step: u64) {

    let body_to_save = fluid.fluids().values().next().unwrap();
    save_particles(container_dir, body_to_save, simulation_step);
    // save_velocities(container_dir, body_to_save, simulation_step);
    // save_accelerations(container_dir, body_to_save, simulation_step);
}

fn save_accelerations(output_dir: &path::Path, fluid: &Fluid<f32>, simulation_step: u64) {
    let file_name = output_dir.clone().join("accelerations").join(format!("{}", simulation_step));
    println!("\"accelerations_path\" = \"{}\",\n", file_name.as_path().to_str().unwrap());

    output_accelerations_to_file(&fluid.accelerations, &file_name);

}

fn output_accelerations_to_file(collection: &Vec<na::Matrix<f32, na::U3, na::U1, na::ArrayStorage<f32, na::U3, na::U1>>>, to_file: &path::PathBuf) {
    let mut file: File = File::create(to_file).unwrap();
    for (index, matrix) in collection.into_iter().enumerate() {
        let serialized = serde_json::to_string(&matrix).unwrap();
        
        file.write(serialized.as_bytes()).ok();
        if index < collection.len() - 1 {
            file.write(",\n".as_bytes()).ok();
        }
    }
    println!("flushing");
    file.flush().ok();
    println!("done");
}

fn save_velocities(output_dir: &path::Path, fluid: &Fluid<f32>, simulation_step: u64) {
    let file_name = output_dir.clone().join("velocities").join(format!("{}", simulation_step));
    println!("\"velocities_path\" = \"{}\",\n", file_name.as_path().to_str().unwrap());

    output_velocities_to_file(&fluid.velocities, &file_name);

}

fn output_velocities_to_file(collection: &Vec<na::Matrix<f32, na::U3, na::U1, na::ArrayStorage<f32, na::U3, na::U1>>>, to_file: &path::PathBuf) {
    let mut file: File = File::create(to_file).unwrap();
    for (index, matrix) in collection.into_iter().enumerate() {
        let serialized = serde_json::to_string(&matrix).unwrap();
        
        file.write(serialized.as_bytes()).ok();
        if index < collection.len() - 1 {
            file.write(",\n".as_bytes()).ok();
        }
    }
    file.flush().ok();
}

fn save_particles(output_dir: &path::Path, fluid: &Fluid<f32>, simulation_step: u64) {
    let file_name = output_dir.clone().join("particles").join(format!("particles-{}.particles", simulation_step));
    println!("\"particles_path\" = \"{}\",\n", file_name.as_path().to_str().unwrap());

    output_positions_to_file(&fluid.positions, &file_name);
}

fn output_positions_to_file(collection: &Vec<Point3<f32>>, to_file: &path::PathBuf) {
    let mut file: File = File::create(to_file).unwrap();
    for point in collection {
        let serialized = format!("{:?},{:?},{:?}\n", point.coords.get(0).unwrap(), point.coords.get(1).unwrap(), point.coords.get(2).unwrap() );
        
        file.write(serialized.as_bytes()).ok();
    }
    file.flush().ok();

}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Wavepool", init_world)]);

    testbed.run()
}