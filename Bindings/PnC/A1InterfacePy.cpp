#include <iostream>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <Eigen/Dense>

#include <PnC/A1PnC/A1Interface.hpp>


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