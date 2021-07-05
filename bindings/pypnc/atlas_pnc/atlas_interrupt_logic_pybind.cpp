#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <pnc/atlas_pnc/atlas_interrupt_logic.hpp>

namespace py = pybind11;

PYBIND11_MODULE(atlas_interrupt_logic, m) {

  py::class_<AtlasInterruptLogic>(m, "AtlasInterruptLogic")
      //.def(py::init<>())
      .def_readwrite("b_interrupt_button_w",
                     &AtlasInterruptLogic::b_interrupt_button_w)
      .def_readwrite("b_interrupt_button_a",
                     &AtlasInterruptLogic::b_interrupt_button_a)
      .def_readwrite("b_interrupt_button_d",
                     &AtlasInterruptLogic::b_interrupt_button_d)
      .def_readwrite("b_interrupt_button_s",
                     &AtlasInterruptLogic::b_interrupt_button_s)
      .def_readwrite("b_interrupt_button_q",
                     &AtlasInterruptLogic::b_interrupt_button_q)
      .def_readwrite("b_interrupt_button_e",
                     &AtlasInterruptLogic::b_interrupt_button_e)
      .def_readwrite("b_interrupt_button_x",
                     &AtlasInterruptLogic::b_interrupt_button_x);
}
