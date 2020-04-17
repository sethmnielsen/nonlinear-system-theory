#pragma once

#include <vector>

#include <Eigen/Core>

#include "geometry/xform.h"
#include "spacecraft_sim/state.h"

class Dynamics
{
public:

  struct Params {
    double mass;
    Eigen::Matrix3d inertia;
    Eigen::Matrix3d inertia_inv;
    Eigen::Vector3d wind;
  };
  
  Dynamics(Params dyn_params);
  void f(const State &x, const Vector6d &forces_and_torques, dState &dx);
  virtual void calculateForces(State &x, int* cmds, double dt, Vector6d& forces_and_torques) = 0;
  void stepDynamics(State& x, int *cmds, double dt, State& xp);

protected:

  double mass_;
  Eigen::Matrix3d inertia_;
  Eigen::Matrix3d inertia_inv_;
  Eigen::Vector3d wind_;
  Eigen::Vector3d gravity_;

  struct Rotor{
    double max;
    std::vector<double> F_poly;
    std::vector<double> T_poly;
    double tau_up; // time constants for response
    double tau_down;
  };

  struct Motor{
    Rotor rotor;
    Eigen::Vector3d position;
    Eigen::Vector3d normal;
    int direction; // 1 for CW -1 for CCW
  };

  // Container for an Actuator
  struct Actuator{
    double max;
    double tau_up;
    double tau_down;
  };

  State x_, x2_, x3_, x4_;
  dState dx_, k1_, k2_, k3_, k4_;
  Vector6d u_;
  bool RK4_;
};
