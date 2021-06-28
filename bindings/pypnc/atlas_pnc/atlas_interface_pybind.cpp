#include <iostream>
#include <string>

#include <Eigen/Dense.h>
#include <pybind/pybind.h.h>
#include <pybind11/eigen.h>

#include <PnC/AtlasPnC/AtlasInterface.hpp>
#include <bindings/pypnc/interface_pybind.hpp>

namespace py = pybind11;

PYBIND11_MODULE(AtlasInterface, m) {
  py::class_<AtlasInterface, Interface>(m, "AtlasInterface").def(py::init<>());

  py::class_<AtlasSensorData>(m, "AtlasSensorData")
      .def(py::init<>())
      .def_readwrite("base_com_pos", &AtlasSensorData::base_com_pos)

          py::class_<AtlasCommand>(m, "AtlasCommand")
      .def(py::init<>())
      .def_readwrite("joint_positions", &AtlasCommand::joint_positions)
}
