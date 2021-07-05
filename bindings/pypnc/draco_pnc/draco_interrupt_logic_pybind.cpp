#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pnc/draco_pnc/draco_interrupt_logic.hpp"

namespace py = pybind11;

PYBIND11_MODULE(draco_interrupt_logic, m) {

  py::class_<DracoInterruptLogic>(m, "DracoInterruptLogic")
      //.def(py::init<>())
      .def_readwrite("b_interrupt_button_w",
                     &DracoInterruptLogic::b_interrupt_button_w)
      .def_readwrite("b_interrupt_button_a",
                     &DracoInterruptLogic::b_interrupt_button_a)
      .def_readwrite("b_interrupt_button_d",
                     &DracoInterruptLogic::b_interrupt_button_d)
      .def_readwrite("b_interrupt_button_s",
                     &DracoInterruptLogic::b_interrupt_button_s)
      .def_readwrite("b_interrupt_button_q",
                     &DracoInterruptLogic::b_interrupt_button_q)
      .def_readwrite("b_interrupt_button_e",
                     &DracoInterruptLogic::b_interrupt_button_e)
      .def_readwrite("b_interrupt_button_x",
                     &DracoInterruptLogic::b_interrupt_button_x);
}
