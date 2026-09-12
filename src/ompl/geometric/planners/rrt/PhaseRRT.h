/*
Copied skeleton RRT.h
author: @shuaiyy`
*/

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_PHASERRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_PHASERRT_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "ompl/base/spaces/RealVectorStateSpace.h"

namespace ompl
{
    namespace geometric
    {


        /**
           @anchor gPhaseRRT
           @par Short description
           PhaseRRT grows a tree over states (q, alpha), where alpha in [0,1]
           indexes a demonstration. A sample draws its phase first and its
           configuration near what the demonstration does at that phase; every
           edge must then advance alpha by a bounded step. Ordering along the
           demonstration therefore holds by construction rather than by a cost
           term. The nearest-neighbour metric is directed, since a candidate
           parent at equal-or-later phase is unreachable, which rules out the
           symmetric datastructures and requires a linear scan.
        */

        /** \brief Phase-monotone Rapidly-exploring Random Trees */
        class PhaseRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            PhaseRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates = false);

            ~PhaseRRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Hand the demonstration to the planner: one waypoint per row,
                config values only (no alpha column). Row i gets alpha = i/(N-1). */
            void setReference(const std::vector<std::vector<double>> &waypoints)
            {
                if (waypoints.size() < 2)
                    throw Exception("PhaseRRT needs at least 2 reference waypoints");
                for (const auto &w : waypoints)
                    if (w.size() != waypoints.front().size())
                        throw Exception("PhaseRRT reference rows have inconsistent sizes");
                reference_ = waypoints;
            }

            /** \brief Indices of the config dimensions that wrap at 2*pi, so that
                xi() interpolates them along the short arc. Empty (the default)
                means no wrapping. */
            void setAngularDims(const std::vector<unsigned int> &dims)
            {
                angularDims_ = dims;
            }

            const std::vector<unsigned int> &getAngularDims() const
            {
                return angularDims_;
            }

            /** \brief Minimum phase advance per extension (> 0: forbids stalling) */
            void setDAlphaMin(double d) { dAlphaMin_ = d; }
            double getDAlphaMin() const { return dAlphaMin_; }

            /** \brief Maximum phase advance per extension (forbids skipping ahead) */
            void setDAlphaMax(double d) { dAlphaMax_ = d; }
            double getDAlphaMax() const { return dAlphaMax_; }

            /** \brief Weight of pose distance vs phase gap in the directed metric */
            void setPhaseLambda(double l) { phaseLambda_ = l; }
            double getPhaseLambda() const { return phaseLambda_; }

            /** \brief Std-dev of the sampling noise around xi(alpha) */
            void setSampleSigma(double s) { sampleSigma_ = s; }
            double getSampleSigma() const { return sampleSigma_; }

            /** \brief Fraction of samples drawn uniformly instead of near the demo */
            void setUniformFraction(double b) { uniformFraction_ = b; }
            double getUniformFraction() const { return uniformFraction_; }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Directed phase metric. `a` is the candidate parent (tree node),
                `b` is the query (new sample). A parent at equal-or-later phase is
                forbidden: infinity means "never choose this". */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                const double alphaParent = getAlpha(a->state);
                const double alphaSample = getAlpha(b->state);
                if (alphaParent >= alphaSample)
                    return std::numeric_limits<double>::infinity();
                return phaseLambda_ * si_->distance(a->state, b->state) +
                       (1.0 - phaseLambda_) * (alphaSample - alphaParent);
            }

            /** \brief The demonstration: reference_[i] = config at alpha = i/(N-1) */
            std::vector<std::vector<double>> reference_;
            /** \brief Config dimensions that wrap at 2*pi (see setAngularDims) */
            std::vector<unsigned int> angularDims_;

            /** \brief Shortest signed step from angle a to angle b, in (-pi, pi] */
            static double angleDiff(double a, double b)
            {
                const double d = b - a;
                return std::atan2(std::sin(d), std::cos(d));
            }

            /** \brief True if config dimension j wraps at 2*pi */
            bool isAngular(std::size_t j) const
            {
                return std::find(angularDims_.begin(), angularDims_.end(), static_cast<unsigned int>(j)) !=
                       angularDims_.end();
            }


            /** \brief Demo pose at progress alpha in [0,1], linear blend of the
                two surrounding waypoints. */
            std::vector<double> xi(double alpha) const
            {
                alpha = std::max(0.0, std::min(1.0, alpha));
                const double position = alpha * (reference_.size() - 1);
                const auto low = static_cast<std::size_t>(position);
                const auto high = std::min(low + 1, reference_.size() - 1);
                const double f = position - static_cast<double>(low);
                std::vector<double> out(reference_[low].size());
                for (std::size_t j = 0; j < out.size(); ++j)
                    out[j] = isAngular(j) ?
                                 reference_[low][j] + f * angleDiff(reference_[low][j], reference_[high][j]) :
                                 (1.0 - f) * reference_[low][j] + f * reference_[high][j];
                return out;
            }

            /** \brief Read alpha from a state. The space MUST be compound with
                exactly two components: [config, RealVector(1) alpha]. */
            static double getAlpha(const base::State *state)
            {
                return state->as<base::CompoundState>()
                    ->as<base::RealVectorStateSpace::StateType>(1)
                    ->values[0];
            }

            /** \brief Write alpha into a state (mirror of getAlpha). */
            static void setAlpha(base::State *state, double alpha)
            {
                state->as<base::CompoundState>()
                    ->as<base::RealVectorStateSpace::StateType>(1)
                    ->values[0] = alpha;
            }

            /** \brief Phase weight in the directed metric */
            double phaseLambda_{0.5};

            /** \brief Per-extension phase advance bounds */
            double dAlphaMin_{0.005};
            double dAlphaMax_{0.05};

            /** \brief Sampling: noise around xi(alpha), and the uniform fraction */
            double sampleSigma_{0.1};
            double uniformFraction_{0.1};

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }  // namespace geometric
}  // namespace ompl

#endif

