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


PYBIND11_MODULE(A1Interface, m) {
    py::class_<EnvInterface, PyEnvInterface>(m, "Interface")
        .def(py::init<>())
        .def("getCommand", &EnvInterface::getCommand);

    py::class_<A1Interface, EnvInterface>(m, "A1Interface")
        .def(py::init<>());

    py::class_<A1SensorData>(m, "A1SensorData")
        .def(py::init<>())
        .def_readwrite("imu_ang_vel", &A1SensorData::imu_ang_vel)
        .def_readwrite("imu_acc", &A1SensorData::imu_acc)
        .def_readwrite("q", &A1SensorData::q)
        .def_readwrite("qdot", &A1SensorData::qdot)
        .def_readwrite("jtrq", &A1SensorData::jtrq)
        .def_readwrite("virtual_q", &A1SensorData::virtual_q)
        .def_readwrite("virtual_qdot", &A1SensorData::virtual_qdot)
        .def_readwrite("flfoot_contact", &A1SensorData::flfoot_contact)
        .def_readwrite("frfoot_contact", &A1SensorData::frfoot_contact)
        .def_readwrite("rlfoot_contact", &A1SensorData::rlfoot_contact)
        .def_readwrite("rrfoot_contact", &A1SensorData::rrfoot_contact);

    py::class_<A1Command>(m, "A1Command")
        .def(py::init<>())
        .def_readwrite("q", &A1Command::q)
        .def_readwrite("qdot", &A1Command::qdot)
        .def_readwrite("jtrq", &A1Command::jtrq);
}
