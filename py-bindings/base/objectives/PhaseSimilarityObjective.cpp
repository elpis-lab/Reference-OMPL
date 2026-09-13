#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/objectives/PhaseSimilarityObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_PhaseSimilarityObjective(nb::module_ &m)
{
    nb::class_<ob::PhaseSimilarityObjective, ob::StateCostIntegralObjective>(m, "PhaseSimilarityObjective")
        .def(nb::init<const ob::SpaceInformationPtr &, bool>(), nb::arg("si"),
             nb::arg("enableMotionCostInterpolation") = false)
        .def("setAlphaScale", &ob::PhaseSimilarityObjective::setAlphaScale, nb::arg("s"))
        .def("setReference", &ob::PhaseSimilarityObjective::setReference, nb::arg("waypoints"));
}
