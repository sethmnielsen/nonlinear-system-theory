#include "spacecraft_sim/spacecraft_sim.h"

SpacecraftSim::SpacecraftSim(Ref<Vector16d> x0)
{
  x_.arr = x0;
  State *x_point = &x_;
  
  setMAVParams();
}

SpacecraftSim::~SpacecraftSim()
{
  delete dynamics_;
}

void SpacecraftSim::setMAVParams()
{
  Dynamics::Params dyn_params;
  dyn_params.mass = 2.0;

  vector<double> Ivec {140.0,     0,    0, 
                           0, 100.0,    0, 
                           0,     0, 80.0}; // kg-m**2
  
  dyn_params.inertia = Map<Matrix3d>(Ivec.data());
  dyn_params.inertia_inv = dyn_params.inertia.inverse();
  dyn_params.wind = Vector3d::Zero();

  dynamics_ = new Spacecraft(dyn_params);

  double imu_update_rate = 1000.0;
  imu_update_period_us_ = (uint64_t)(1e6/imu_update_rate);
}

void SpacecraftSim::run()
{
  uint64_t t_end = now_us_ + dt_us_;
  while (t_end > now_us_)
  {
    now_us_ += dt_us_;
    dynamics_->stepDynamics(x_, outputs_, imu_update_period_us_*1e-6, xp_);
    x_ = xp_;
  }
}
