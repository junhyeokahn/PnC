#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"

class PyInterface : public Interface {
  using Interface::Interface;
  void getCommand(void *_sensor_data, void *_command) override {
    PYBIND11_OVERLOAD_PURE(void, Interface, getCommand, _sensor_data, _command);
  }
};

namespace py = pybind11;

PYBIND11_MODULE(fixed_draco_interface, m) {
  py::module::import("fixed_draco_interrupt_logic");

  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("getCommand", &Interface::getCommand);

  py::class_<FixedDracoInterface, Interface>(m, "FixedDracoInterface")
      .def(py::init<bool>())
      .def_readwrite("interrupt", &FixedDracoInterface::interrupt);

  py::class_<FixedDracoSensorData>(m, "FixedDracoSensorData")
      .def(py::init<>())
      .def_readwrite("joint_positions", &FixedDracoSensorData::joint_positions)
      .def_readwrite("joint_velocities",
                     &FixedDracoSensorData::joint_velocities);

  py::class_<FixedDracoCommand>(m, "FixedDracoCommand")
      .def(py::init<>())
      .def_readwrite("joint_positions", &FixedDracoCommand::joint_positions)
      .def_readwrite("joint_velocities", &FixedDracoCommand::joint_velocities)
      .def_readwrite("joint_torques", &FixedDracoCommand::joint_torques);
}
