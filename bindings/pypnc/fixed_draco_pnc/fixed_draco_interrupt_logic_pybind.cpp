#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pnc/fixed_draco_pnc/fixed_draco_interrupt_logic.hpp"

namespace py = pybind11;

PYBIND11_MODULE(fixed_draco_interrupt_logic, m) {

  py::class_<FixedDracoInterruptLogic>(m, "FixedDracoInterruptLogic")
      //.def(py::init<>())
      .def_readwrite("b_interrupt_button_w",
                     &FixedDracoInterruptLogic::b_interrupt_button_w)
      .def_readwrite("b_interrupt_button_a",
                     &FixedDracoInterruptLogic::b_interrupt_button_a)
      .def_readwrite("b_interrupt_button_d",
                     &FixedDracoInterruptLogic::b_interrupt_button_d)
      .def_readwrite("b_interrupt_button_s",
                     &FixedDracoInterruptLogic::b_interrupt_button_s)
      .def_readwrite("b_interrupt_button_q",
                     &FixedDracoInterruptLogic::b_interrupt_button_q)
      .def_readwrite("b_interrupt_button_e",
                     &FixedDracoInterruptLogic::b_interrupt_button_e)
      .def_readwrite("b_interrupt_button_r",
                     &FixedDracoInterruptLogic::b_interrupt_button_r)
      .def_readwrite("b_interrupt_button_f",
                     &FixedDracoInterruptLogic::b_interrupt_button_f)
      .def_readwrite("b_interrupt_button_x",
                     &FixedDracoInterruptLogic::b_interrupt_button_x);
}
