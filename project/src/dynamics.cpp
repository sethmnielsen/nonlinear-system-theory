#include "spacecraft_sim/dynamics.h"
#include <iostream>
#include <iomanip>

Dynamics::Dynamics(Dynamics::Params dyn_params) :
  RK4_(true),
  gravity_{0, 0, 0}
{
  std::cout.precision(5);
  std::cout << std::fixed;
  
  mass_ = dyn_params.mass;
  inertia_ = dyn_params.inertia;
  inertia_inv_ = inertia_.inverse();
  wind_ = dyn_params.wind;
}

void Dynamics::f(const State &x, const Vector6d &forces_and_torques, dState &dx)
{
  Map<const Vector3d> f(forces_and_torques.data());
  Map<const Vector3d> tau(forces_and_torques.data() + 3);

  dx.dpos = x.vel;  // TODO: WRONG REPRESENTATION; should be rotated into inertial frame, yet dynamics are correct without rotation
  dx.datt = x.omega;
  dx.dvel = f/mass_ + x.pose.q_.rotp(gravity_) - x.omega.cross(x.vel);
  dx.domega = inertia_inv_ * tau - x.omega.cross(inertia_ * x.omega);
  
  // std::cout << "dx.dpos: " <<  dx.dpos.transpose() << std::endl;
}

void Dynamics::stepDynamics(State& x, int* cmds, double dt, State& xp)
{
  x_ = x;
  calculateForces(x, cmds, dt, u_);

  if (RK4_)
  {
    // 4th order Runge-Kutta integration
    x_ = x;
    f(x_, u_, k1_);

    x2_ = x_;
    x2_.pose.t_ += k1_.dpos * dt / 2.0;
    x2_.pose.q_ += k1_.datt * dt / 2.0;
    x2_.omega += k1_.domega * dt / 2.0;
    f(x2_, u_, k2_);

    x3_ = x_;
    x3_.pose.t_ += k2_.dpos * dt / 2.0;
    x3_.pose.q_ += k2_.datt * dt / 2.0;
    x3_.omega += k2_.domega * dt / 2.0;
    f(x3_, u_, k3_);

    x4_ = x_;
    x4_.pose.t_ += k3_.dpos * dt / 2.0;
    x4_.pose.q_ += k3_.datt * dt / 2.0;
    x4_.omega += k3_.domega * dt / 2.0;   
    f(x4_, u_, k4_);

    dx_ = (k1_ + 2.0 * k2_ + 2.0 * k3_ + k4_) * dt / 6.0;
  }
  else
  {
    // Euler integration
    x_ = x;
    f(x_, u_, dx_);
    dx_ = dx_ * dt;
  }

  // Copy output
  xp = x + dx_;
}
