#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <PnC/AtlasPnC/AtlasInterface.hpp>

/////////

// class PyInterface : public Interface {
// using Interface::Interface;
// void getCommand(void *_sensor_data, void *_command) override {
// PYBIND11_OVERLOAD_PURE(
// void,       [> Return Type<]
// Interface,  [> Parent class<]
// getCommand, [> Name of function in c++ (must match python name) <]
//_sensor_data, _command [> arguement(s) <]
//);
//}
//};

//////////

namespace py = pybind11;

PYBIND11_MODULE(atlas_interface, m) {
  py::module::import("interface");
  py::module::import("atlas_interrupt_logic");

  // py::object Interface =
  //(py::object)py::module_::import("interface").attr("Interface");

  //////

  // py::class_<Interface, PyInterface>(m, "Interface")
  //.def(py::init<>())
  //.def("getCommand", &Interface::getCommand);

  //////

  py::class_<AtlasInterface, Interface>(m, "AtlasInterface")
      .def(py::init<>())
      .def_readwrite("interrupt", &AtlasInterface::interrupt);

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
