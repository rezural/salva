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
use salva3d::solver::{DFSPHSolver, ArtificialViscosity, XSPHViscosity, Akinci2013SurfaceTension};
use salva3d::LiquidWorld;
use std::f32;

#[path = "./helper.rs"]
mod helper;

pub fn init_world(testbed: &mut Testbed) {

    // Fluid sim world dimensions
    let (width, height, depth, particle_radius) = (100.0, 15.0, 100.0, 1.0/8.0);

    let subdivs = ((width/particle_radius) as i32, (height/particle_radius) as i32, (depth/particle_radius) as i32);

    println!("{{\"width\": {}, \"height\": {}, \"depth\": {},\"particle_radius\": {},\"subdivs\": [{},{},{}]}}",
        width, height, depth, particle_radius, subdivs.0, subdivs.1, subdivs.2);
    println!("(w, h, d, n): {} {} {} {}", subdivs.0, subdivs.1, subdivs.2, subdivs.0 * subdivs.1 * subdivs.2);

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
    // println!("DT: {}, {}", mechanical_world.integration_parameters.dt(), mechanical_world.integration_parameters.inv_dt());
    // mechanical_world.integration_parameters.set_dt(mechanical_world.integration_parameters.dt() / 100.0);
    // println!("DT: {}, {}", mechanical_world.integration_parameters.dt(), mechanical_world.integration_parameters.inv_dt());

    /*
     * Liquid world.
     */
    // let particle_rad = 0.01;
    let solver: DFSPHSolver<f32> = DFSPHSolver::new();
    let mut liquid_world = LiquidWorld::new(solver, particle_radius, 2.0);
    let mut coupling_manager = ColliderCouplingSet::new();

    // Liquid.
    // let mut fluid: Fluid<f32> = helper::cube_fluid(nparticles, nparticles, nparticles, particle_radius, 3000.0);
    // println!("wavepool");
    let mut fluid: Fluid<f32> = helper::volume_of_liquid(width, height, depth, particle_radius, 1000.0);

    fluid.transform_by(&Isometry3::translation(
        0.0,
        ground_thickness + height / 2.0 + particle_radius * 1.1,
        0.0,
    ));

    println!("size: {}", fluid.size_in_bytes());

    // let viscosity = XSPHViscosity::new(0.5, 0.0);
    // let tension = Akinci2013SurfaceTension::new(1.0, 10.0);

    let viscosity = XSPHViscosity::new(0.05, 0.05);
    fluid.nonpressure_forces.push(Box::new(viscosity));

    let tension = Akinci2013SurfaceTension::new(0.1, 1.0);
    fluid.nonpressure_forces.push(Box::new(tension));

    // let viscosity = ArtificialViscosity::new(1.0, 0.0);
    // fluid.nonpressure_forces.push(Box::new(viscosity));
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

    // Callback that will be executed on the main loop to generate new particles every second.
    let mut last_t = 0.0;

    testbed.add_callback_with_fluids(move |liquid_world, _, _, _, _, _, _, t| {
        println!("Done frame");
        let fluid = liquid_world.fluids_mut().get_mut(fluid_handle).unwrap();

        for i in 0..fluid.num_particles() {
            if fluid.positions[i].y < -2.0 {
                fluid.delete_particle_at_next_timestep(i);
            }
        }

        if t - last_t < 2.0 {
            return;
        }

        last_t = t;
        let diam = particle_radius * 2.0;
        let nparticles = 8;
        let mut particles = Vec::new();
        let mut velocities = Vec::new();
        let shift = 30.0;
        let vel = 3.0;

        for i in 0..nparticles {
            for j in 0..nparticles * 2 {
                for k in 0..nparticles {
                    let pos = Point3::new(i as f32 * diam, j as f32 * diam, k as f32 * diam);
                    particles.push(pos + Vector3::new(0.0, shift, 0.0));
                    velocities.push(Vector3::y() * vel);
                }
            }
        }

        fluid.add_particles(&particles, Some(&velocities));
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

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Wavepool", init_world)]);

    testbed.run()
}