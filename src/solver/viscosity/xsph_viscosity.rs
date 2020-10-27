#[cfg(feature = "parallel")]
use rayon::prelude::*;

use na::{self, RealField};
use serde::{Serialize, Deserialize};

use crate::geometry::ParticlesContacts;

use crate::math::Vector;
use crate::object::{Boundary, Fluid};
use crate::solver::NonPressureForce;
use crate::TimestepManager;

#[derive(Clone, Serialize, Deserialize)]
/// Implements the viscosity model introduced with the XSPH method.
pub struct XSPHViscosity<N: RealField> {
    /// The viscosity coefficient when interacting with boundaries.
    pub boundary_viscosity_coefficient: N,
    /// The fluid viscosity coefficient.
    pub fluid_viscosity_coefficient: N,
}

impl<N: RealField> XSPHViscosity<N> {
    /// Initializes the XSPH viscosity with the given viscosity coefficients.
    pub fn new(fluid_viscosity_coefficient: N, boundary_viscosity_coefficient: N) -> Self {
        Self {
            boundary_viscosity_coefficient,
            fluid_viscosity_coefficient,
        }
    }
}

impl<N: RealField> NonPressureForce<N> for XSPHViscosity<N> {
    fn solve(
        &mut self,
        timestep: &TimestepManager<N>,
        _kernel_radius: N,
        fluid_fluid_contacts: &ParticlesContacts<N>,
        fluid_boundaries_contacts: &ParticlesContacts<N>,
        fluid: &mut Fluid<N>,
        boundaries: &[Boundary<N>],
        densities: &[N],
    ) {
        let boundary_viscosity_coefficient = self.boundary_viscosity_coefficient;
        let fluid_viscosity_coefficient = self.fluid_viscosity_coefficient;
        let velocities = &fluid.velocities;
        let volumes = &fluid.volumes;
        let density0 = fluid.density0;

        par_iter_mut!(fluid.accelerations)
            .enumerate()
            .for_each(|(i, acceleration)| {
                let mut added_fluid_vel = Vector::zeros();
                let mut added_boundary_vel = Vector::zeros();
                let vi = velocities[i];

                if self.fluid_viscosity_coefficient != N::zero() {
                    for c in fluid_fluid_contacts
                        .particle_contacts(i)
                        .read()
                        .unwrap()
                        .iter()
                    {
                        if c.i_model == c.j_model {
                            added_fluid_vel += (velocities[c.j] - vi)
                                * (fluid_viscosity_coefficient
                                    * c.weight
                                    * volumes[c.j]
                                    * density0
                                    / densities[c.j]);
                        }
                    }
                }

                if self.boundary_viscosity_coefficient != N::zero() {
                    for c in fluid_boundaries_contacts
                        .particle_contacts(i)
                        .read()
                        .unwrap()
                        .iter()
                    {
                        let delta = (boundaries[c.j_model].velocities[c.j] - vi)
                            * (boundary_viscosity_coefficient
                                * c.weight
                                * boundaries[c.j_model].volumes[c.j]
                                * density0
                                / densities[c.i]);
                        added_boundary_vel += delta;

                        let mi = volumes[c.i] * density0;
                        boundaries[c.j_model].apply_force(c.j, delta * (-mi * timestep.inv_dt()));
                    }
                }

                *acceleration +=
                    added_fluid_vel * timestep.inv_dt() + added_boundary_vel * timestep.inv_dt();
            })
    }

    fn apply_permutation(&mut self, _: &[usize]) {}
}
