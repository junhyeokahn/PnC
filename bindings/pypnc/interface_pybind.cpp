#include <pybind11/pybind11.h>

#include <PnC/Interface.hpp>

class PyInterface : public Interface {
  using Interface::Interface;

  void getCommand(void *_sensor_data, void *_command) override {
    PYBIND11_OVERRIDE_PURE(
        void,       /* Return Type*/
        Interface,  /* Parent class*/
        getCommand, /* Name of function in c++ (must match python name) */
        _sensor_data, _command /* arguement(s) */
    );
  }
};

namespace py = pybind11;

PYBIND11_MODULE(interface, m) {
  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("getCommand", &Interface::getCommand);
}
