#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/rrt/PhaseRRTstar.h"
#include "../../init.h"

namespace nb = nanobind;
using namespace ompl::geometric;
using namespace ompl::base;

void ompl::binding::geometric::initPlannersRrt_PhaseRRTstar(nb::module_ &m)
{
    // TAG [og::PhaseRRTstar][Planner]
    nb::class_<PhaseRRTstar, Planner>(m, "PhaseRRTstar")
        .def(nb::init<const SpaceInformationPtr &>(), nb::arg("si"))
        .def("setReference", &PhaseRRTstar::setReference, nb::arg("waypoints"))
        .def("setDAlphaMin", &PhaseRRTstar::setDAlphaMin, nb::arg("d"))
        .def("getDAlphaMin", &PhaseRRTstar::getDAlphaMin)
        .def("setDAlphaMax", &PhaseRRTstar::setDAlphaMax, nb::arg("d"))
        .def("getDAlphaMax", &PhaseRRTstar::getDAlphaMax)
        .def("setSampleSigma", &PhaseRRTstar::setSampleSigma, nb::arg("s"))
        .def("getSampleSigma", &PhaseRRTstar::getSampleSigma)
        .def("setFlatAlpha", &PhaseRRTstar::setFlatAlpha, nb::arg("scale"))
        .def("setProjectionConstraint", &PhaseRRTstar::setProjectionConstraint, nb::arg("c"))
        .def("setPhaseGrid", &PhaseRRTstar::setPhaseGrid, nb::arg("levels"))
        .def("getPhaseGrid", &PhaseRRTstar::getPhaseGrid)
        .def("setSampleWeights", &PhaseRRTstar::setSampleWeights, nb::arg("w"))
        .def("getSampleWeights", &PhaseRRTstar::getSampleWeights)
        .def("setUniformFraction", &PhaseRRTstar::setUniformFraction, nb::arg("b"))
        .def("getUniformFraction", &PhaseRRTstar::getUniformFraction)
        .def("setGoalBias", &PhaseRRTstar::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &PhaseRRTstar::getGoalBias)
        .def("setRange", &PhaseRRTstar::setRange, nb::arg("distance"))
        .def("getRange", &PhaseRRTstar::getRange)
        .def("setRewireFactor", &PhaseRRTstar::setRewireFactor, nb::arg("rewireFactor"))
        .def("getRewireFactor", &PhaseRRTstar::getRewireFactor)
        .def("setup", &PhaseRRTstar::setup)
        .def("clear", &PhaseRRTstar::clear)
        .def("solve",
             [](PhaseRRTstar &self, nb::object what)
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
        .def("getPlannerData", &PhaseRRTstar::getPlannerData, nb::arg("data"));
}