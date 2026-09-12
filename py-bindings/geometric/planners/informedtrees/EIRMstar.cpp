#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/informedtrees/EIRMstar.h"
#include "ompl/geometric/planners/informedtrees/EITstar.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtrees_EIRMstar(nb::module_ &m)
{
    nb::class_<og::EIRMstar, og::EITstar>(m, "EIRMstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))

        .def("setStartGoalPruningThreshold", &og::EIRMstar::setStartGoalPruningThreshold, nb::arg("threshold"))
        .def("getStartGoalPruningThreshold", &og::EIRMstar::getStartGoalPruningThreshold);
}
