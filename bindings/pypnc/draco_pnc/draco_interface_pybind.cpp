#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pnc/draco_pnc/draco_interface.hpp"

class PyInterface : public Interface {
  using Interface::Interface;
  void getCommand(void *_sensor_data, void *_command) override {
    PYBIND11_OVERLOAD_PURE(void, Interface, getCommand, _sensor_data, _command);
  }
};

namespace py = pybind11;

PYBIND11_MODULE(draco_interface, m) {
  py::module::import("draco_interrupt_logic");

  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("getCommand", &Interface::getCommand);

  py::class_<DracoInterface, Interface>(m, "DracoInterface")
      .def(py::init<bool>())
      .def_readwrite("interrupt", &DracoInterface::interrupt);

  py::class_<DracoSensorData>(m, "DracoSensorData")
      .def(py::init<>())
      .def_readwrite("imu_frame_iso", &DracoSensorData::imu_frame_iso)
      .def_readwrite("imu_frame_vel", &DracoSensorData::imu_frame_vel)
      .def_readwrite("joint_positions", &DracoSensorData::joint_positions)
      .def_readwrite("joint_velocities", &DracoSensorData::joint_velocities)
      .def_readwrite("b_rf_contact", &DracoSensorData::b_rf_contact)
      .def_readwrite("b_lf_contact", &DracoSensorData::b_lf_contact);

  py::class_<DracoCommand>(m, "DracoCommand")
      .def(py::init<>())
      .def_readwrite("joint_positions", &DracoCommand::joint_positions)
      .def_readwrite("joint_velocities", &DracoCommand::joint_velocities)
      .def_readwrite("joint_torques", &DracoCommand::joint_torques);
}
