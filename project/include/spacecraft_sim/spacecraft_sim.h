#pragma once

#include <random>
#include <fstream>

#include <Eigen/Core>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "spacecraft_sim/state.h"
#include "spacecraft_sim/dynamics.h"
#include "spacecraft_sim/spacecraft.h"
#include "geometry/xform.h"

namespace py = pybind11;
using namespace Eigen;
using namespace xform;
using namespace std;

class SpacecraftSim
{
public:
  SpacecraftSim(Ref<Vector16d> x0, double dt);
  ~SpacecraftSim();

  // python bindings (pybind functions)
  void run();
  void setMAVParams();
  void getState(Ref<Vector16d> x) { x = x_.arr; }

private:

  // Dynamics
  Dynamics *dynamics_ = nullptr;
  State x_;
  State xp_;
  
  uint64_t dt_us_;
  uint64_t now_us_ = 0;

  Vector4d outputs_ = Vector4d::Zero();
  int pwm_outputs_[14];

  double imu_update_period_us_;
};


PYBIND11_MODULE(spacecraft_sim_interface, m) {
    m.doc() = "pybind11 interface to spacecraft simulation plugin";
    // Bind the Interface Class

    py::class_<SpacecraftSim>(m, "SpacecraftSim") // The object will be named SpacecraftSim in python
      .def(py::init<Ref<Vector16d>, double>()) // constructor
      .def("run", &SpacecraftSim::run)
      .def("getState", &SpacecraftSim::getState);
}