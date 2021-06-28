#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <PnC/AtlasPnC/AtlasInterface.hpp>

namespace py = pybind11;

PYBIND11_MODULE(atlas_interface, m) {
  py::module::import("interface");

  py::class_<AtlasInterface, Interface>(m, "AtlasInterface").def(py::init<>());

  py::class_<AtlasSensorData>(m, "AtlasSensorData")
      .def(py::init<>())
      .def_readwrite("base_com_pos", &AtlasSensorData::base_com_pos)
      .def_readwrite("base_com_quat", &AtlasSensorData::base_com_quat)
      .def_readwrite("base_com_lin_vel", &AtlasSensorData::base_com_lin_vel)
      .def_readwrite("base_com_ang_vel", &AtlasSensorData::base_com_ang_vel)
      .def_readwrite("base_joint_pos", &AtlasSensorData::base_joint_pos)
      .def_readwrite("base_joint_quat", &AtlasSensorData::base_joint_quat)
      .def_readwrite("base_joint_lin_vel", &AtlasSensorData::base_joint_lin_vel)
      .def_readwrite("base_joint_ang_vel", &AtlasSensorData::base_joint_ang_vel)
      .def_readwrite("joint_positions", &AtlasSensorData::joint_positions)
      .def_readwrite("joint_velocities", &AtlasSensorData::joint_velocities);

  py::class_<AtlasCommand>(m, "AtlasCommand")
      .def(py::init<>())
      .def_readwrite("joint_positions", &AtlasCommand::joint_positions)
      .def_readwrite("joint_velocities", &AtlasCommand::joint_velocities)
      .def_readwrite("joint_torques", &AtlasCommand::joint_torques);
}
