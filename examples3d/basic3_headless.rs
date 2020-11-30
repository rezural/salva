extern crate nalgebra as na;
extern crate ply_rs;

use std::fs::File;
use na::{Isometry3, Point3, Vector3};

use ply_rs::ply::{ Ply, DefaultElement, Encoding, ElementDef, PropertyDef, PropertyType, ScalarType, Property, Addable };
use ply_rs::writer::{ Writer };

use rapier3d::pipeline::{ChannelEventCollector, PhysicsPipeline, QueryPipeline};
use rapier3d::{crossbeam, dynamics::{JointSet, RigidBodyBuilder, RigidBodySet}};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, ColliderShape};
use rapier_testbed3d::{PhysicsState};
use salva3d::integrations::rapier::{ColliderSampling, FluidsPipeline, FluidsTestbedPlugin};
use salva3d::object::{Boundary, Fluid};
use salva3d::solver::ArtificialViscosity;
use std::f32;
use std::path;
use std::io::Write;

#[path = "./helper.rs"]
mod helper;

const PARTICLE_RADIUS: f32 = 0.025;
const SMOOTHING_FACTOR: f32 = 2.0;

pub fn init_and_run() {
    /*
     * World
     */
    let gravity = Vector3::y() * -9.81;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    // Parameters of the ground.
    let ground_thickness = 0.2;
    let ground_half_width = 1.5;
    let ground_half_height = 0.7;

    // fluids.
    let nparticles = 15;
    let mut fluid = helper::cube_fluid(nparticles, nparticles, nparticles, PARTICLE_RADIUS, 1000.0);
    fluid.transform_by(&Isometry3::translation(
        0.0,
        ground_thickness + nparticles as f32 * PARTICLE_RADIUS,
        0.0,
    ));
    let viscosity = ArtificialViscosity::new(1.0, 0.0);
    fluid.nonpressure_forces.push(Box::new(viscosity));
    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);

    /*
     * Ground.
     */
    let ground_shape = ColliderShape::cuboid(Vector3::new(
        ground_half_width,
        ground_thickness,
        ground_half_width,
    ));
    let wall_shape = ColliderShape::cuboid(Vector3::new(
        ground_thickness,
        ground_half_height,
        ground_half_width,
    ));

    let ground_body = RigidBodyBuilder::new_static().build();
    let ground_handle = bodies.insert(ground_body);

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
        let samples =
            salva3d::sampling::shape_surface_ray_sample(&*wall_shape, PARTICLE_RADIUS).unwrap();
        let co = ColliderBuilder::new(wall_shape.clone())
            .position(*pose)
            .build();
        let co_handle = colliders.insert(co, ground_handle, &mut bodies);
        let bo_handle = fluids_pipeline
            .liquid_world
            .add_boundary(Boundary::new(Vec::new()));

        fluids_pipeline.coupling.register_coupling(
            bo_handle,
            co_handle,
            ColliderSampling::StaticSampling(samples),
        );
    }

    let samples =
        salva3d::sampling::shape_surface_ray_sample(&*ground_shape, PARTICLE_RADIUS).unwrap();
    let co = ColliderBuilder::new(ground_shape).build();
    let co_handle = colliders.insert(co, ground_handle, &mut bodies);
    let bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new()));

    fluids_pipeline.coupling.register_coupling(
        bo_handle,
        co_handle,
        ColliderSampling::StaticSampling(samples),
    );

    /*
     * Set up physics.
     */

    let mut physics = PhysicsState::new();
    physics.gravity = gravity;
    physics.bodies = bodies;
    physics.colliders = colliders;
    physics.joints = joints;
    
    let time = 0.;

    #[cfg(feature = "parallel")]
    let num_threads = num_cpus::get_physical();
    #[cfg(not(feature = "parallel"))]
    let num_threads = 1;

    #[cfg(feature = "parallel")]
    let thread_pool = rapier::rayon::ThreadPoolBuilder::new()
        .num_threads(num_threads)
        .build()
        .unwrap();

    let contact_channel = crossbeam::channel::unbounded();
    let proximity_channel = crossbeam::channel::unbounded();
    let event_handler = ChannelEventCollector::new(proximity_channel.0, contact_channel.0);

    let mut step = 0;

    loop {
        println!("Stepping");
        #[cfg(feature = "parallel")]
        {
            thread_pool.install(|| {
                physics.pipeline.step(
                    &physics.gravity,
                    &physics.integration_parameters,
                    &mut physics.broad_phase,
                    &mut physics.narrow_phase,
                    &mut physics.bodies,
                    &mut physics.colliders,
                    &mut physics.joints,
                    None,
                    None,
                    &event_handler,
                );
            });
        }

        #[cfg(not(feature = "parallel"))]
        physics.pipeline.step(
            &physics.gravity,
            &physics.integration_parameters,
            &mut physics.broad_phase,
            &mut physics.narrow_phase,
            &mut physics.bodies,
            &mut physics.colliders,
            &mut physics.joints,
            None,
            None,
            &event_handler,
        );

        physics
            .query_pipeline
            .update(&physics.bodies, &physics.colliders);
        
        fluids_pipeline.step(
            &physics.gravity,
            physics.integration_parameters.dt(),
            &physics.colliders,
            &mut physics.bodies,
        );

        let fluid = fluids_pipeline
            .liquid_world
            .fluids()
            .get(fluid_handle)
            .unwrap();
        
        let output_file = String::from(format!("{:05}.ply", step));
        let output_file = path::PathBuf::from(output_file);
        output_particles_to_file(&fluid, &output_file);
        
        step += 1;
    }

}

fn output_particles_to_file(fluid: &Fluid, to_file: &path::PathBuf) {
    let mut file: File = File::create(to_file).unwrap();
    let mut buf = Vec::<u8>::new();

    let mut ply = create_ply();

    let points = fluid
        .positions
        .iter()
        .enumerate()
        .map(|(i, particle)| {
            let mut point = DefaultElement::new();
            let velocity = fluid.velocities[i];

            //first frame will not have velocities
            let velocity = if velocity.x == 0. {
                velocity
            } else {
                Vector3::from(velocity)
            };
            
            point.insert("x".to_string(), Property::Float(particle.x));
            point.insert("y".to_string(), Property::Float(particle.y));
            point.insert("z".to_string(), Property::Float(particle.z));
            point.insert("nx".to_string(), Property::Float(velocity.x));
            point.insert("ny".to_string(), Property::Float(velocity.y));
            point.insert("nz".to_string(), Property::Float(velocity.z));
            point
        })
        .collect();

    ply.payload.insert("vertex".to_string(), points);

    let w = Writer::new();
    let _written = w.write_ply(&mut buf, &mut ply).unwrap();

    file.write_all(&buf);

}

fn create_ply() -> Ply<DefaultElement> {
    let mut ply = Ply::<DefaultElement>::new();
    ply.header.encoding = Encoding::Ascii;
    ply.header.comments.push("A beautiful comment!".to_string());
    let mut point_element = ElementDef::new("vertex".to_string());
    let p = PropertyDef::new("x".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("y".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("z".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("nx".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("ny".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    let p = PropertyDef::new("nz".to_string(), PropertyType::Scalar(ScalarType::Float));
    point_element.properties.add(p);
    ply.header.elements.add(point_element);

    ply
}
fn main() {
    init_and_run();
}
