extern crate nalgebra as na;

use na::{Isometry3, Point3, Vector3};
use rapier3d::dynamics::{JointSet, RigidBodyBuilder, RigidBodySet, RigidBodyHandle};
use rapier3d::geometry::{ColliderBuilder, ColliderSet, ColliderShape, Shape};
use rapier_testbed3d::Testbed;
use salva3d::integrations::rapier::{ColliderSampling, FluidsPipeline, FluidsTestbedPlugin};
use salva3d::object::{Boundary, Fluid};
use salva3d::solver::ArtificialViscosity;
use std::f32;
use ncollide3d::query::Ray;
use nalgebra::{ComplexField, Translation3, DMatrix};

#[path = "./helper.rs"]
mod helper;

const PARTICLE_RADIUS: f32 = 0.01;
const SMOOTHING_FACTOR: f32 = 2.0;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let gravity = Vector3::y() * -9.81;
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let joints = JointSet::new();
    let mut fluids_pipeline = FluidsPipeline::new(PARTICLE_RADIUS, SMOOTHING_FACTOR);

    let ground_size = Vector3::new(2., 0.2, 2.);
    let wall_height = 1.0;
    let (ground_handle, ground_shape) = create_ground(
        ground_size,
        wall_height,
        &mut fluids_pipeline,
        &mut bodies,
        &mut colliders);

    let fluid_depth = 0.1;
    let raycast_from = Point3::new(0., wall_height, 0.);
    let fluid_particles = create_fluid_above_ground(
        ground_size.x - PARTICLE_RADIUS * 6., // remove a few particles width
        ground_size.z - PARTICLE_RADIUS * 6., // remove a few particles depth
        fluid_depth,
        PARTICLE_RADIUS,
        raycast_from,
        &*ground_shape
    );
    let mut fluid = Fluid::new(fluid_particles, PARTICLE_RADIUS, 1000.);

    let viscosity = ArtificialViscosity::new(1.0, 0.0);
    fluid.nonpressure_forces.push(Box::new(viscosity));

    let fluid_handle = fluids_pipeline.liquid_world.add_fluid(fluid);

    /*
     * Set up the testbed.
     */
    let mut plugin = FluidsTestbedPlugin::new();
    plugin.set_pipeline(fluids_pipeline);
    plugin.set_fluid_color(fluid_handle, Point3::new(0.8, 0.7, 1.0));
    testbed.add_plugin(plugin);
    testbed.set_body_wireframe(ground_handle, true);
    testbed.set_world_with_gravity(bodies, colliders, joints, gravity);
    testbed.integration_parameters_mut().set_dt(1.0 / 200.0);
    testbed.look_at(Point3::new(0., -wall_height * 0.6, ground_size.z), Point3::origin());
}

// create a cuboid of water
//
pub fn volume_of_liquid<F>(
    width: f32,
    length: f32,
    height: f32,
    particle_rad: f32,
    translation: Translation3<f32>,
    include_particle_fn: F
)
    -> Vec<Point3<f32>> where
    F: Fn(&Point3<f32>) -> bool
{
    let particle_diam = particle_rad * 2.0;
    let size_x = (width / particle_diam).ceil() as i32;
    let size_y = (height / particle_diam).ceil() as i32;
    let size_z = (length / particle_diam).ceil() as i32;

    let half_extents = Vector3::new(size_x as f32, size_y as f32, size_z as f32) * particle_rad;

    let mut points = Vec::new();

    for i in 0..size_x {
        for j in 0..size_y {
            for k in 0..size_z {
                let x = (i as f32) * particle_diam;
                let y = (j as f32) * particle_diam;
                let z = (k as f32) * particle_diam;
                let point = Point3::new(x, y, z)
                    + Vector3::repeat(particle_rad)
                    - half_extents
                    + translation.vector.clone();

                if include_particle_fn(&point) {
                    points.push(point);
                }
            }
        }
    }
    points
}

pub fn create_ground(
    ground_size: Vector3<f32>,
    wall_height: f32,
    fluids_pipeline: &mut FluidsPipeline,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
) -> (RigidBodyHandle, ColliderShape) {

    let factor = 20.;
    let subdivs = Vector3::new((ground_size.x * factor) as usize, 0, (ground_size.z * factor) as usize);
    let heights = DMatrix::from_fn(subdivs.x + 1, subdivs.z + 1, |i, j| {
        if i == 0 || i == subdivs.x || j == 0 || j == subdivs.z {
            wall_height
        } else {
            let x = i as f32 * ground_size.x / (subdivs.x as f32 / factor);
            let z = j as f32 * ground_size.z / (subdivs.z as f32 / factor);

            // NOTE: make sure we use the sin/cos from simba to ensure
            // cross-platform determinism of the example when the
            // enhanced_determinism feature is enabled.
            (<f32 as ComplexField>::sin(x) + <f32 as ComplexField>::cos(z)) / 4.
        }
    });

    // Setup the ground.
    let ground_handle = bodies.insert(RigidBodyBuilder::new_static().build());
    let ground_shape = ColliderBuilder::heightfield(heights, ground_size);
    let co = ground_shape.build();

    let co_handle = colliders.insert(co, ground_handle, bodies);
    let bo_handle = fluids_pipeline
        .liquid_world
        .add_boundary(Boundary::new(Vec::new()));
    fluids_pipeline.coupling.register_coupling(
        bo_handle,
        co_handle,
        ColliderSampling::DynamicContactSampling,
    );

    (ground_handle, ground_shape.shape)
}

pub fn create_fluid_above_ground(
    width: f32,
    length: f32,
    height: f32,
    particle_rad: f32,
    raycast_from: Point3<f32>,
    ground_shape: &dyn Shape,
) -> Vec<Point3<f32>> {
    let fluid_translation = Isometry3::translation(0., 0., 0.).translation;

    volume_of_liquid(width,
        length,
        height,
        particle_rad,
        fluid_translation,
        |point: &Point3<f32>| {
            // raycast from a point above the ground, if ground is in between,
            // this point is below the ground
            let ray = Ray::new(point - Vector3::y() * particle_rad, -Vector3::y());
            ground_shape.intersects_ray(&Isometry3::identity(), &ray, 10000.)
            // !ground_shape.intersects_ray(&Isometry3::identity(), &ray, 10000.)
            // true
        })

}

fn main() {
    let testbed = Testbed::from_builders(0, vec![("Basic", init_world)]);
    testbed.run()
}
