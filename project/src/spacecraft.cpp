#include "spacecraft_sim/dynamics.h"
#include "spacecraft_sim/spacecraft.h"


Spacecraft::Spacecraft(Dynamics::Params dyn_params) :
  Dynamics::Dynamics(dyn_params)
{
  force_allocation_matrix_.resize(4,4);
  torque_allocation_matrix_.resize(4,4);
}

void Spacecraft::calculateForces(State &x, int *cmds, double dt, Vector6d& forces_and_torques)
{
  // Use the allocation matrix to calculate the body-fixed force and torques
  // output_forces = [moment about x, moment about y, 0, thrust in z] expressed in body frame
  Eigen::Vector4d output_forces = force_allocation_matrix_*actual_forces_;
  Eigen::Vector4d output_torques = torque_allocation_matrix_*actual_torques_;
  Eigen::Vector4d output_forces_and_torques = output_forces + output_torques;
  
  output_forces_and_torques.setZero();

  forces_and_torques(2) += output_forces_and_torques(3);
}
