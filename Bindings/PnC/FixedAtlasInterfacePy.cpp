#include <iostream>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <Eigen/Dense>

#include <PnC/FixedAtlasPnC/FixedAtlasInterface.hpp>

class PyEnvInterface : public EnvInterface {
    using EnvInterface::EnvInterface;

    // trampoline (one for each virtual function)
    void getCommand(void* _sensor_data, void* _command_data) override {
        PYBIND11_OVERLOAD_PURE(
            void,         /* Return type */
            EnvInterface, /* Parent class */
            getCommand,   /* Name of function in C++ (must match Python name) */
            _sensor_data, _command_data /* Argument(s) */
        );
    }
};

namespace py = pybind11;

PYBIND11_MODULE(FixedAtlasInterface, m) {
    py::class_<EnvInterface, PyEnvInterface>(m, "Interface")
        .def(py::init<>())
        .def("getCommand", &EnvInterface::getCommand);

    py::class_<FixedAtlasInterface, EnvInterface>(m, "FixedAtlasInterface")
        .def(py::init<>());

    py::class_<FixedAtlasSensorData>(m, "FixedAtlasSensorData")
        .def(py::init<>())
        .def_readwrite("q", &FixedAtlasSensorData::q)
        .def_readwrite("qdot", &FixedAtlasSensorData::qdot);

    py::class_<FixedAtlasCommand>(m, "FixedAtlasCommand")
        .def(py::init<>())
        .def_readwrite("jtrq", &FixedAtlasCommand::jtrq);
}
