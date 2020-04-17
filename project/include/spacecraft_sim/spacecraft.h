#pragma once

#include <vector>

#include <Eigen/Core>

#include "geometry/xform.h"
#include "spacecraft_sim/state.h"
#include "spacecraft_sim/dynamics.h"

using namespace Eigen;
using namespace xform;

class Spacecraft : public Dynamics
{
public:
    Spacecraft(Dynamics::Params dyn_params);
    void calculateForces(State &x, int *cmds, double t, Vector6d& forces_and_torques) override;

protected:
    Eigen::MatrixXd force_allocation_matrix_;
    Eigen::MatrixXd torque_allocation_matrix_;
    Eigen::VectorXd desired_forces_;
    Eigen::VectorXd desired_torques_;
    Eigen::VectorXd actual_forces_;
    Eigen::VectorXd actual_torques_;
};
