/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_PHASE_STATE_SAMPLER_
#define OMPL_GEOMETRIC_PLANNERS_RRT_PHASE_STATE_SAMPLER_

#include "ompl/base/Constraint.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"

#include <cstddef>
#include <string>
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /** \brief Demonstration-conditioned sampler for phase-augmented states.

            The canonical layout is an outer CompoundStateSpace containing an
            arbitrary configuration StateSpace followed by a one-dimensional
            RealVectorStateSpace for phase. A legacy flat real-vector-like
            layout is enabled explicitly with setFlatAlpha(). */
        class PhaseStateSampler : public base::StateSampler
        {
        public:
            explicit PhaseStateSampler(const base::StateSpacePtr &space);
            ~PhaseStateSampler() override;

            void setReference(const std::vector<std::vector<double>> &waypoints);

            void setSampleSigma(double sigma)
            {
                sampleSigma_ = sigma;
            }
            double getSampleSigma() const
            {
                return sampleSigma_;
            }

            void setSampleWeights(const std::vector<double> &weights);
            const std::vector<double> &getSampleWeights() const
            {
                return sampleWeights_;
            }

            void setPhaseGrid(double levels)
            {
                phaseGrid_ = levels;
            }
            double getPhaseGrid() const
            {
                return phaseGrid_;
            }

            void setFlatAlpha(double scale);
            void setProjectionConstraint(const base::ConstraintPtr &constraint)
            {
                projection_ = constraint;
            }

            /** \brief Draw a phase-conditioned sample near the reference. */
            void sampleUniform(base::State *state) override;

            /** \brief Delegate neighborhood sampling to the space's default sampler. */
            void sampleUniformNear(base::State *state, const base::State *near, double distance) override;

            /** \brief Delegate mean-centered sampling to the space's default sampler. */
            void sampleGaussian(base::State *state, const base::State *mean, double stdDev) override;

            double getAlpha(const base::State *state) const;
            void setAlpha(base::State *state, double alpha) const;

            std::size_t referenceSize() const
            {
                return reference_.size();
            }
            std::size_t referenceDimension() const
            {
                return reference_.empty() ? 0u : reference_.front().size();
            }
            bool hasCompatibleLayout() const;

            /** \brief Empty if the space is Compound(configuration, RealVector(1)
                phase) or flat-alpha is enabled; otherwise a human-readable error. */
            std::string diagnoseLayout() const;

        private:
            void clearReferenceStates();
            std::vector<double> flatReferenceAt(double alpha) const;
            void projectIfNeeded(base::State *state) const;

            base::StateSpacePtr stateSpace_;
            base::StateSpacePtr configurationSpace_;
            base::StateSamplerPtr defaultSampler_;
            base::StateSamplerPtr configurationSampler_;
            std::vector<std::vector<double>> reference_;
            std::vector<base::State *> referenceStates_;
            base::State *interpolatedConfiguration_{nullptr};
            std::vector<double> sampleWeights_;
            base::ConstraintPtr projection_;
            double sampleSigma_{0.1};
            double phaseGrid_{100.0};
            bool canonicalLayout_{false};
            bool flatAlpha_{false};
            double alphaScale_{1.0};
            unsigned int alphaIndex_{0u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
