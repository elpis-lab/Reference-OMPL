#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/rrt/PhaseRRT.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersRrt_PhaseRRT(nb::module_ &m)
{
    // TAG [og::PhaseRRT][Planner]
    nb::class_<PhaseRRT, Planner>(m, "PhaseRRT")
        .def(nb::init<const SpaceInformationPtr &, bool>(), nb::arg("si"),
             nb::arg("addIntermediateStates") = false)
        .def("setReference", &PhaseRRT::setReference, nb::arg("waypoints"))
        .def("setAngularDims", &PhaseRRT::setAngularDims, nb::arg("dims"))
        .def("getAngularDims", &PhaseRRT::getAngularDims)
        .def("setDAlphaMin", &PhaseRRT::setDAlphaMin, nb::arg("d"))
        .def("getDAlphaMin", &PhaseRRT::getDAlphaMin)
        .def("setDAlphaMax", &PhaseRRT::setDAlphaMax, nb::arg("d"))
        .def("getDAlphaMax", &PhaseRRT::getDAlphaMax)
        .def("setPhaseLambda", &PhaseRRT::setPhaseLambda, nb::arg("l"))
        .def("getPhaseLambda", &PhaseRRT::getPhaseLambda)
        .def("setSampleSigma", &PhaseRRT::setSampleSigma, nb::arg("s"))
        .def("getSampleSigma", &PhaseRRT::getSampleSigma)
        .def("setUniformFraction", &PhaseRRT::setUniformFraction, nb::arg("b"))
        .def("getUniformFraction", &PhaseRRT::getUniformFraction)
        .def("setGoalBias", &PhaseRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &PhaseRRT::getGoalBias)
        .def("setIntermediateStates", &PhaseRRT::setIntermediateStates,
             nb::arg("addIntermediateStates"))
        .def("getIntermediateStates", &PhaseRRT::getIntermediateStates)
        .def("setRange", &PhaseRRT::setRange, nb::arg("distance"))
        .def("getRange", &PhaseRRT::getRange)
        .def("setup", &PhaseRRT::setup)
        .def("clear", &PhaseRRT::clear)
        .def("solve",
             [](PhaseRRT &self, nb::object what)
             {
                 if (nb::isinstance<PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def("getPlannerData", &PhaseRRT::getPlannerData, nb::arg("data"));
}