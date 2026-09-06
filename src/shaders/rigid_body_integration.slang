#pragma once
#include "rigid_body_rotation.slang"

extension RigidBody
{
  [mutating] void advect(Ptr<SimConfig> sim_config)
  {
    // TGS_SOFT integrates velocity per sub-step (h = dt/N); every other solver uses the full dt.
    daxa_f32 h = (sim_config->solver_type == SimSolverType::TGS_SOFT)
                 ? sim_config->dt / daxa_f32(sim_config->tgs_substeps) : sim_config->dt;
    if(has_flag(RigidBodyFlag::GRAVITY)){
      // gravity is an ACCELERATION (m/s^2): apply it DIRECTLY to velocity. The old code routed it
      // through apply_impulse(), which does v += impulse*inv_mass -> gravity got scaled by inv_mass,
      // so a mass-5 cube (inv_mass=0.2) fell at g/5. This hit every advect-based solver (PGS_SOFT,
      // TGS_SOFT); AVBD was unaffected (it folds gravity into its inertial target directly, not here).
      this.velocity += daxa_f32vec3(0, sim_config->gravity, 0) * h;
    }
    // terminal linear-speed clamp (anti-punch-through): cap impact energy so fast rain doesn't
    // tunnel deep into the floor. Resting/settling bodies are far under the cap (untouched).
    daxa_f32 v_len = length(this.velocity);
    if (v_len > BB_MAX_LINEAR_SPEED)
    {
      this.velocity *= BB_MAX_LINEAR_SPEED / v_len;
    }
    advance(h, LINEAR_DAMPING, ANGULAR_DAMPING);
  }

  [mutating] void advance(daxa_f32 dt, daxa_f32 linear_damping, daxa_f32 angular_damping)
  {
    this.velocity *= exp(-dt * linear_damping);
    this.omega *= exp(-dt * angular_damping);
  }

  [mutating] void integrate_positions(daxa_f32 dt) {

    this.position += this.velocity * dt;
    this.rotation = apply_angular_velocity(this.rotation, this.omega, dt);
  }

};
