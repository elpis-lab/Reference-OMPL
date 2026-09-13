/*
author: @shuaiyy
*/

#ifndef OMPL_BASE_OBJECTIVES_PHASE_SIMILARITY_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_PHASE_SIMILARITY_OBJECTIVE_

#include <memory>
#include <vector>

#include "ompl/base/objectives/StateCostIntegralObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief Similarity-to-demonstration objective for phase-augmented
            states (q, alpha).

            The canonical layout is an outer CompoundStateSpace containing an
            arbitrary configuration StateSpace and a one-dimensional
            RealVectorStateSpace phase. A flattened state with alpha in its
            last value slot remains supported as a compatibility fallback.

            The cost of a state is the deviation from the demonstration at the
            state's own phase: ||q - xi(alpha)||. This is a direct one-to-one
            comparison (alpha indexes the demonstration), unlike the
            order-agnostic min-over-all-waypoints cost it replaces.

            Per-edge cost comes from StateCostIntegralObjective (trapezoid
            rule), so the path cost is the integral of deviation over arc
            length: a path glued to the demonstration costs ~0, detours cost
            deviation x distance. There is no separate length term. */
        class PhaseSimilarityObjective : public StateCostIntegralObjective
        {
        public:
            PhaseSimilarityObjective(const SpaceInformationPtr &si,
                                     bool enableMotionCostInterpolation = false);

            ~PhaseSimilarityObjective() override;

            PhaseSimilarityObjective(const PhaseSimilarityObjective &) = delete;
            PhaseSimilarityObjective &operator=(const PhaseSimilarityObjective &) = delete;

            /** \brief Set the demonstration. Rows are configuration-space
                poses (no alpha column); row i sits at phase i/(N-1). Must
                match the reference given to the planner. */
            void setReference(const std::vector<std::vector<double>> &waypoints);

            /** \brief The state's last value stores alpha * scale (flat-alpha
                constrained layout); default 1.0 = plain alpha. */
            void setAlphaScale(double s) { alphaScale_ = s; }

            /** \brief Deviation of the state from the demonstration at the
                state's own phase: ||q - xi(alpha)||. */
            Cost stateCost(const State *s) const override;

        private:
            /** \brief Release configuration-space states allocated for the reference. */
            void clearReferenceStates();

            /** \brief Piecewise-linear demonstration lookup at phase alpha. */
            std::vector<double> xi(double alpha) const;

            /** \brief Demonstration waypoints; row i sits at phase i/(N-1). */
            std::vector<std::vector<double>> reference_;

            /** \brief Configuration subspace for canonical [configuration, phase] states. */
            StateSpacePtr configurationSpace_;

            /** \brief Topology-aware representation of reference_ in configurationSpace_. */
            std::vector<State *> referenceStates_;

            /** \brief Scratch state used to interpolate the canonical reference. */
            State *interpolatedReference_{nullptr};

            /** \brief True for the canonical outer Compound(configuration, RV(1)) layout. */
            bool canonicalLayout_{false};

            double alphaScale_{1.0};
        };
    }  // namespace base
}  // namespace ompl

#endif
