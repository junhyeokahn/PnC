#include <iostream>
#include <string>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <Eigen/Dense>

#include <PnC/DracoPnC/DracoInterface.hpp>

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

PYBIND11_MODULE(DracoInterface, m) {
    py::class_<EnvInterface, PyEnvInterface>(m, "Interface")
        .def(py::init<>())
        .def("getCommand", &EnvInterface::getCommand);

    py::class_<DracoInterface, EnvInterface>(m, "DracoInterface")
        .def(py::init<>());

    py::class_<DracoSensorData>(m, "DracoSensorData")
        .def(py::init<>())
        .def_readwrite("imu_ang_vel", &DracoSensorData::imu_ang_vel)
        .def_readwrite("imu_acc", &DracoSensorData::imu_acc)
        .def_readwrite("q", &DracoSensorData::q)
        .def_readwrite("qdot", &DracoSensorData::qdot)
        .def_readwrite("jtrq", &DracoSensorData::jtrq)
        .def_readwrite("temperature", &DracoSensorData::temperature)
        .def_readwrite("motor_current", &DracoSensorData::motor_current)
        .def_readwrite("bus_voltage", &DracoSensorData::bus_voltage)
        .def_readwrite("bus_current", &DracoSensorData::bus_current)
        .def_readwrite("rotor_inertia", &DracoSensorData::rotor_inertia)
        .def_readwrite("rfoot_ati", &DracoSensorData::rfoot_ati)
        .def_readwrite("lfoot_ati", &DracoSensorData::lfoot_ati)
        .def_readwrite("rfoot_contact", &DracoSensorData::rfoot_contact)
        .def_readwrite("lfoot_contact", &DracoSensorData::lfoot_contact);

    py::class_<DracoCommand>(m, "DracoCommand")
        .def(py::init<>())
        .def_readwrite("turn_off", &DracoCommand::turn_off)
        .def_readwrite("q", &DracoCommand::q)
        .def_readwrite("qdot", &DracoCommand::qdot)
        .def_readwrite("jtrq", &DracoCommand::jtrq);
}
