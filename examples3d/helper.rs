use na::{Point3, Vector3};
use salva3d::object::Fluid;

pub fn cube_fluid(ni: usize, nj: usize, nk: usize, particle_rad: f32, density: f32) -> Fluid<f32> {
    let mut points = Vec::new();
    let half_extents = Vector3::new(ni as f32, nj as f32, nk as f32) * particle_rad;

    for i in 0..ni {
        for j in 0..nj {
            for k in 0..nk {
                let x = (i as f32) * particle_rad * 2.0;
                let y = (j as f32) * particle_rad * 2.0;
                let z = (k as f32) * particle_rad * 2.0;
                points.push(Point3::new(x, y, z) + Vector3::repeat(particle_rad) - half_extents);
            }
        }
    }

    Fluid::new(points, particle_rad, density)
}

pub fn box_of_particles(width: f32, height: f32, depth: f32, particle_rad: f32, translation: Vector3<f32>) -> Vec<Point3<f32>> {
    let particle_diam = particle_rad * 2.0;
    let size_x = (width / particle_diam).ceil() as i32;
    let size_y = (height / particle_diam).ceil() as i32;
    let size_z = (depth / particle_diam).ceil() as i32;

    let half_extents = Vector3::new(size_x as f32, size_y as f32, size_z as f32) * particle_rad;

    let mut points = Vec::new();

    let mut offset = false;
    for i in 0..size_y {

        let row_translation = match offset {
            true => Vector3::new(-particle_rad/2.0, 0.0, -particle_rad/2.0),
            false => Vector3::new(particle_rad/2.0, 0.0, particle_rad/2.0)
        };

        for j in 0..size_x {
            for k in 0..size_z {
                let x = (j as f32) * particle_diam;
                let y = (i as f32) * particle_diam;
                let z = (k as f32) * particle_diam;
                points.push((Point3::new(x, y, z) + Vector3::repeat(particle_rad) - half_extents) + row_translation + translation);

            }
        }
        offset = !offset;
    }

    points
}

pub fn box_of_fluid(width: f32, height: f32, depth: f32, particle_radius: f32, density: f32) -> Fluid<f32> {

    // let half_extents = Vector3::new(ni as f32, nj as f32, nk as f32) * particle_rad;
    // Fluid::new(points, particle_rad, density)
    Fluid::new(box_of_particles(width, height, depth, particle_radius, Vector3::new(0.0, 0.0, 0.0)), particle_radius, density)
}