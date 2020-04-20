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
    Matrix4d force_allocation_matrix_ = Matrix4d::Zero();
    Matrix4d torque_allocation_matrix_ = Matrix4d::Zero();
    Vector4d desired_forces_ = Vector4d::Zero();
    Vector4d desired_torques_ = Vector4d::Zero();
    Vector4d actual_forces_ = Vector4d::Zero();
    Vector4d actual_torques_ = Vector4d::Zero();
};
