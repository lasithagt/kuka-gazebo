#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <vector>

#include <ros/ros.h>
#include "iiwa_ros.h"
#include <iiwa_msgs/JointVelocity.h>

namespace py = pybind11;

PYBIND11_PLUGIN(pybindings) {
    py::module m("pybindings", "Python bindings for c++ code using pybind11");

    using namespace pybind11::literals;

    // py::class_<baseSimulator>(m, "baseSimulator");

    py::class_<ros::NodeHandle>(m, "NodeHandle")
        .def(py::init());

    py::class_<iiwa_ros::iiwaRos>(m, "iiwaRos");

    py::class_<iiwa_ros::iiwaRosGazebo>(m, "iiwaRosGazebo", py::base<iiwa_ros::iiwaRos>())
        .def(py::init())
        .def("init", &iiwa_ros::iiwaRosGazebo::init, "nh"_a)
        .def("initPy", &iiwa_ros::iiwaRosGazebo::initPy, "rate"_a=60.0)
        .def("setJointVelocity", &iiwa_ros::iiwaRosGazebo::setJointVelocity, "velocity"_a)
        .def("setJointVelocityPy", &iiwa_ros::iiwaRosGazebo::setJointVelocityPy, "velocity"_a=(std::vector<double>){0, 0, 0, 0, 0, 0, 0});

    return m.ptr();
}



