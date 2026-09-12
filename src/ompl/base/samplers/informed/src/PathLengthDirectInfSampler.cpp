/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/util/Exception.h"
#include "ompl/base/OptimizationObjective.h"
// For ompl::base::GoalSampleableRegion, which both GoalState and GoalStates derive from:
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

// For std::make_shared
#include <memory>
// For std::vector
#include <vector>

namespace ompl
{
    namespace base
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:

        // The direct ellipsoid sampling class for path-length:
        PathLengthDirectInfSampler::PathLengthDirectInfSampler(const ProblemDefinitionPtr &probDefn,
                                                               unsigned int maxNumberCalls)
          : InformedSampler(probDefn, maxNumberCalls)
        {
            // Variables
            // The number of start states
            unsigned int numStarts;
            // The number of goal states
            unsigned numGoals;
            // The foci of the PHSs as a std::vector of states. Goals must be nonconst, as we need to allocate them
            // (unfortunately):
            std::vector<const State *> startStates;
            std::vector<State *> goalStates;

            if (!probDefn_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
            {
                throw Exception("PathLengthDirectInfSampler: The direct path-length informed sampler currently only "
                                "supports goals that can be cast to a sampleable goal region (i.e., are countable "
                                "sets).");
            }

            /// Note: We don't check that there is a cost-to-go heuristic set in the optimization objective, as this
            /// direct sampling is only for Euclidean distance.

            // Store the number of starts and goals
            numStarts = probDefn_->getStartStateCount();
            numGoals = probDefn_->getGoal()->as<ompl::base::GoalSampleableRegion>()->maxSampleCount();

            // Sanity check that there is atleast one of each
            if (numStarts < 1u || numGoals < 1u)
            {
                throw Exception("PathLengthDirectInfSampler: There must be at least 1 start and and 1 goal state when "
                                "the informed sampler is created.");
            }

            // Recursively identify RealVector (informed) and SO(2)/SO(3) (uninformed) leaf subspaces. This supports
            // both typed compounds (SE2/SE3/Dubins/Reeds-Shepp) and nested compounds such as Compound(SE2, RealVector).
            std::vector<std::size_t> chain;
            collectSpaceLeaves(InformedSampler::space_, chain);

            if (informedLeaves_.empty())
            {
                throw Exception("PathLengthDirectInfSampler: No RealVector subspace found for informed sampling.");
            }

            // Build the Euclidean informed subspace used by the PHS.
            if (informedLeaves_.size() == 1u)
            {
                informedSubSpace_ = informedLeaves_.front().space;
                if (informedSubSpace_->getType() == STATE_SPACE_UNKNOWN)
                {
                    OMPL_WARN("PathLengthDirectInfSampler: Treating the StateSpace of type \"STATE_SPACE_UNKNOWN\" as "
                              "type \"STATE_SPACE_REAL_VECTOR\".");
                }
            }
            else
            {
                unsigned int informedDim = 0u;
                for (const auto &leaf : informedLeaves_)
                    informedDim += leaf.space->getDimension();

                auto informedRV = std::make_shared<RealVectorStateSpace>(informedDim);
                RealVectorBounds bounds(informedDim);
                unsigned int idx = 0u;
                for (const auto &leaf : informedLeaves_)
                {
                    if (leaf.space->getType() != STATE_SPACE_REAL_VECTOR)
                    {
                        throw Exception("PathLengthDirectInfSampler: Concatenating non-RealVector informed leaves is "
                                        "not supported.");
                    }
                    const auto &leafBounds = leaf.space->as<RealVectorStateSpace>()->getBounds();
                    for (unsigned int i = 0u; i < leaf.space->getDimension(); ++i, ++idx)
                    {
                        bounds.setLow(idx, leafBounds.low[i]);
                        bounds.setHigh(idx, leafBounds.high[i]);
                    }
                }
                informedRV->setBounds(bounds);
                informedSubSpace_ = informedRV;
            }

            // Combined measure of uninformed leaves (SO2/SO3), used when reporting the informed measure.
            uninformedMeasure_ = 1.0;
            for (auto &leaf : uninformedLeaves_)
            {
                leaf.sampler = leaf.space->allocDefaultStateSampler();
                uninformedMeasure_ *= leaf.space->getMeasure();
            }

            // Create a sampler for the whole space that we can use if we have no information
            baseSampler_ = InformedSampler::space_->allocDefaultStateSampler();

            // Store the foci, first the starts:
            for (unsigned int i = 0u; i < numStarts; ++i)
            {
                startStates.push_back(probDefn_->getStartState(i));
            }

            // Extract the state of each goal one and place into the goal vector!
            for (unsigned int i = 0u; i < numGoals; ++i)
            {
                // Allocate a state onto the back of the vector:
                goalStates.push_back(InformedSampler::space_->allocState());

                // Now sample a goal into that state:
                probDefn_->getGoal()->as<ompl::base::GoalSampleableRegion>()->sampleGoal(goalStates.back());
            }

            // Now, iterate create a PHS for each start-goal pair
            // Each start
            for (unsigned int i = 0u; i < numStarts; ++i)
            {
                // Variable
                // The start as a vector
                std::vector<double> startFocusVector = getInformedSubstate(startStates.at(i));

                // Each goal
                for (unsigned int j = 0u; j < numGoals; ++j)
                {
                    // Variable
                    // The goal as a vector
                    std::vector<double> goalFocusVector = getInformedSubstate(goalStates.at(j));

                    // Create the definition of the PHS
                    listPhsPtrs_.push_back(std::make_shared<ProlateHyperspheroid>(
                        informedSubSpace_->getDimension(), &startFocusVector[0], &goalFocusVector[0]));
                }
            }

            // Finally deallocate the states in the goal state vector:
            for (unsigned int i = 0u; i < numGoals; ++i)
            {
                // Free the state in the vector:
                InformedSampler::space_->freeState(goalStates.at(i));
            }

            if (listPhsPtrs_.size() > 100u)
            {
                OMPL_WARN("PathLengthDirectInfSampler: Rejection sampling is used in order to maintain uniform density "
                          "in the presence of overlapping informed subsets. At some number of independent subsets, "
                          "this will become prohibitively expensive. Current number of independent subsets: %d",
                          listPhsPtrs_.size());
            }
        }

        PathLengthDirectInfSampler::~PathLengthDirectInfSampler() = default;

        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            // Variable
            // The persistent iteration counter:
            unsigned int iter = 0u;

            // Call the sampleUniform helper function with my iteration counter:
            return sampleUniform(statePtr, maxCost, &iter);
        }

        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            // Sample from the larger PHS until the sample does not lie within the smaller PHS.
            // Since volume in a sphere/spheroid is proportionately concentrated near the surface, this isn't horribly
            // inefficient, though a direct method would be better

            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Spend numIters_ iterations trying to find an informed sample:
            for (unsigned int i = 0u; i < InformedSampler::numIters_ && !foundSample; ++i)
            {
                // Call the helper function for the larger PHS. It will move our iteration counter:
                foundSample = sampleUniform(statePtr, maxCost, &i);

                // Did we find a sample?
                if (foundSample)
                {
                    // We did, but it only satisfied the upper bound. Check that it meets the lower bound.

                    // Variables
                    // The cost of the sample we found
                    Cost sampledCost = heuristicSolnCost(statePtr);

                    // Check if the sample's cost is greater than or equal to the lower bound
                    foundSample = InformedSampler::opt_->isCostEquivalentTo(minCost, sampledCost) ||
                                  InformedSampler::opt_->isCostBetterThan(minCost, sampledCost);
                }
                // No else, no sample was found.
            }

            // All done, one way or the other.
            return foundSample;
        }

        bool PathLengthDirectInfSampler::hasInformedMeasure() const
        {
            return true;
        }

        double PathLengthDirectInfSampler::getInformedMeasure(const Cost &currentCost) const
        {
            // Variable
            // The measure of the informed set
            double informedMeasure = 0.0;

            // The informed measure is then the sum of the measure of the individual PHSs for the given cost:
            for (const auto &phsPtr : listPhsPtrs_)
            {
                // It is nonsensical for a PHS to have a transverse diameter less than the distance between its foci, so
                // skip those that do
                if (currentCost.value() > phsPtr->getMinTransverseDiameter())
                {
                    informedMeasure = informedMeasure + phsPtr->getPhsMeasure(currentCost.value());
                }
                // No else, this value is better than this ellipse. It will get removed later.
            }

            // And further multiplied by the measure of the uninformed subspaces (SO2/SO3 leaves)
            if (!uninformedLeaves_.empty())
            {
                informedMeasure = informedMeasure * uninformedMeasure_;
            }

            // Return the smaller of the two measures
            return std::min(InformedSampler::space_->getMeasure(), informedMeasure);
        }

        Cost PathLengthDirectInfSampler::heuristicSolnCost(const State *statePtr) const
        {
            // Variable
            // The raw data in the state
            std::vector<double> rawData = getInformedSubstate(statePtr);
            // The Cost, infinity to start
            Cost minCost = InformedSampler::opt_->infiniteCost();

            // Iterate over the separate subsets and return the minimum
            for (const auto &phsPtr : listPhsPtrs_)
            {
                /** \todo Use a heuristic function for the full solution cost defined in OptimizationObjective or some
                 * new Heuristic class once said function is defined. */
                minCost = InformedSampler::opt_->betterCost(minCost, Cost(phsPtr->getPathLength(&rawData[0])));
            }

            return minCost;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        bool PathLengthDirectInfSampler::sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iters)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Whether we successfully returnes
            // Check if a solution path has been found
            if (!InformedSampler::opt_->isFinite(maxCost))
            {
                // We don't have a solution yet, we sample from our basic sampler instead...
                baseSampler_->sampleUniform(statePtr);

                // Up our counter by one:
                ++(*iters);

                // Mark that we sampled:
                foundSample = true;
            }
            else  // We have a solution
            {
                // Update the definitions of the PHSs
                updatePhsDefinitions(maxCost);

                // Sample from the PHSs.

                // When the summed measure of the PHSes are suitably large, it makes more sense to just sample from the
                // entire planning space and keep the sample if it lies in any PHS
                // Check if the average measure is greater than half the domain's measure. Half is an arbitrary number.
                if (informedSubSpace_->getMeasure() < summedMeasure_ / static_cast<double>(listPhsPtrs_.size()))
                {
                    // The measure is large, sample from the entire world and keep if it's in any PHS
                    foundSample = sampleBoundsRejectPhs(statePtr, iters);
                }
                else
                {
                    // The measure is sufficiently small that we will directly sample the PHSes, with the weighting
                    // given by their relative measures
                    foundSample = samplePhsRejectBounds(statePtr, iters);
                }
            }

            // Return:
            return foundSample;
        }

        bool PathLengthDirectInfSampler::sampleBoundsRejectPhs(State *statePtr, unsigned int *iters)
        {
            // Variable
            // Whether we've found a sample:
            bool foundSample = false;

            // Spend numIters_ iterations trying to find an informed sample:
            while (!foundSample && *iters < InformedSampler::numIters_)
            {
                // Generate a random sample
                baseSampler_->sampleUniform(statePtr);

                // The informed substate
                std::vector<double> informedVector = getInformedSubstate(statePtr);

                // Check if the informed state is in any PHS.
                foundSample = isInAnyPhs(informedVector);

                // Increment the provided counter
                ++(*iters);
            }

            // successful?
            return foundSample;
        }

        bool PathLengthDirectInfSampler::samplePhsRejectBounds(State *statePtr, unsigned int *iters)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Due to the possibility of overlap between multiple PHSs, we keep a sample with a probability of 1/K,
            // where K is the number of PHSs the sample is in.
            while (!foundSample && *iters < InformedSampler::numIters_)
            {
                // Variables
                // The informed subset of the sample as a vector
                std::vector<double> informedVector(informedSubSpace_->getDimension());
                // The random PHS in use for this sample.
                ProlateHyperspheroidCPtr phsCPtr = randomPhsPtr();

                // Use the PHS to get a sample in the informed subspace irrespective of boundary
                rng_.uniformProlateHyperspheroid(phsCPtr, &informedVector[0]);

                // Keep with probability 1/K
                foundSample = keepSample(informedVector);

                // If we're keeping it, then check if the state is in the problem domain:
                if (foundSample)
                {
                    // Turn into a state of our full space
                    createFullState(statePtr, informedVector);

                    // Return if the resulting state is in the problem:
                    foundSample = InformedSampler::space_->satisfiesBounds(statePtr);
                }
                // No else

                // Increment the provided counter
                ++(*iters);
            }

            // Successful?
            return foundSample;
        }

        std::vector<double> PathLengthDirectInfSampler::getInformedSubstate(const State *statePtr) const
        {
            std::vector<double> rawData;
            rawData.reserve(informedSubSpace_->getDimension());

            for (const auto &leaf : informedLeaves_)
            {
                std::vector<double> leafData(leaf.space->getDimension());
                leaf.space->copyToReals(leafData, getSubstate(statePtr, leaf.chain));
                rawData.insert(rawData.end(), leafData.begin(), leafData.end());
            }

            return rawData;
        }

        void PathLengthDirectInfSampler::createFullState(State *statePtr, const std::vector<double> &informedVector)
        {
            // Write the informed Euclidean sample into each RealVector leaf.
            unsigned int offset = 0u;
            for (const auto &leaf : informedLeaves_)
            {
                const unsigned int dim = leaf.space->getDimension();
                std::vector<double> leafData(informedVector.begin() + static_cast<std::ptrdiff_t>(offset),
                                             informedVector.begin() + static_cast<std::ptrdiff_t>(offset + dim));
                leaf.space->copyFromReals(getSubstate(statePtr, leaf.chain), leafData);
                offset += dim;
            }

            // Uniformly sample each uninformed (SO2/SO3) leaf.
            for (const auto &leaf : uninformedLeaves_)
                leaf.sampler->sampleUniform(getSubstate(statePtr, leaf.chain));
        }

        void PathLengthDirectInfSampler::collectSpaceLeaves(const StateSpacePtr &space, std::vector<std::size_t> &chain)
        {
            if (!space->isCompound())
            {
                SubspaceLeaf leaf;
                leaf.chain = chain;
                leaf.space = space;

                if (space->getType() == STATE_SPACE_REAL_VECTOR || space->getType() == STATE_SPACE_UNKNOWN)
                {
                    informedLeaves_.push_back(leaf);
                }
                else if (space->getType() == STATE_SPACE_SO2 || space->getType() == STATE_SPACE_SO3)
                {
                    uninformedLeaves_.push_back(leaf);
                }
                else
                {
                    throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2, SE3, Dubins, "
                                    "ReedsShepp, and compound combinations of those spaces. Unsupported subspace "
                                    "type: " +
                                    std::to_string(space->getType()));
                }
                return;
            }

            const int type = space->getType();
            if (type != STATE_SPACE_SE2 && type != STATE_SPACE_SE3 && type != STATE_SPACE_DUBINS &&
                type != STATE_SPACE_REEDS_SHEPP && type != STATE_SPACE_UNKNOWN)
            {
                throw Exception("PathLengthDirectInfSampler only supports RealVector, SE2, SE3, Dubins, ReedsShepp, "
                                "and compound combinations of those spaces. Provided compound state space of type: " +
                                std::to_string(type) + " with " +
                                std::to_string(space->as<CompoundStateSpace>()->getSubspaceCount()) + " subspaces.");
            }

            const CompoundStateSpace *compoundSpace = space->as<CompoundStateSpace>();
            for (unsigned int idx = 0u; idx < compoundSpace->getSubspaceCount(); ++idx)
            {
                chain.push_back(idx);
                collectSpaceLeaves(compoundSpace->getSubspace(idx), chain);
                chain.pop_back();
            }
        }

        const State *PathLengthDirectInfSampler::getSubstate(const State *state, const std::vector<std::size_t> &chain)
        {
            for (std::size_t index : chain)
                state = state->as<CompoundState>()->components[index];
            return state;
        }

        State *PathLengthDirectInfSampler::getSubstate(State *state, const std::vector<std::size_t> &chain)
        {
            for (std::size_t index : chain)
                state = state->as<CompoundState>()->components[index];
            return state;
        }

        void PathLengthDirectInfSampler::updatePhsDefinitions(const Cost &maxCost)
        {
            // Variable
            // The iterator for the list:
            auto phsIter = listPhsPtrs_.begin();

            // Iterate over the list of PHSs, updating the summed measure
            // Reset the sum
            summedMeasure_ = 0.0;
            while (phsIter != listPhsPtrs_.end())
            {
                // Check if the specific PHS can ever be better than the given maxCost, i.e., if the distance between
                // the foci is less than the current max cost
                if ((*phsIter)->getMinTransverseDiameter() < maxCost.value())
                {
                    // It can improve the solution, or it's the only PHS we have, update it

                    // Update the transverse diameter
                    (*phsIter)->setTransverseDiameter(maxCost.value());

                    // Increment the summed measure of the ellipses.
                    summedMeasure_ = summedMeasure_ + (*phsIter)->getPhsMeasure();

                    // Increment the iterator
                    ++phsIter;
                }
                else if (listPhsPtrs_.size() > 1u)
                {
                    // It can't, and it is not the last PHS, remove it

                    // Remove the iterator to delete from the list, this returns the next:
                    /// \todo Make sure this doesn't cause problems for JIT sampling?
                    phsIter = listPhsPtrs_.erase(phsIter);
                }
                else
                {
                    // It can't, but it's the last PHS, so we can't remove it.

                    // Make sure it's transverse diameter is set to something:
                    (*phsIter)->setTransverseDiameter((*phsIter)->getMinTransverseDiameter());

                    // Set the summed measure to 0.0 (as a degenerate PHS is a line):
                    summedMeasure_ = 0.0;

                    // Increment the iterator so we move past this to the end.
                    ++phsIter;
                }
            }
        }

        ompl::ProlateHyperspheroidPtr PathLengthDirectInfSampler::randomPhsPtr()
        {
            // Variable
            // The return value
            ompl::ProlateHyperspheroidPtr rval;

            // If we only have one PHS, this can be simplified:
            if (listPhsPtrs_.size() == 1u)
            {
                // One PHS, keep this simple.

                // Return it
                rval = listPhsPtrs_.front();
            }
            else
            {
                // We have more than one PHS to consider

                // Variables
                // A randomly generated number in the interval [0,1]
                double randDbl = rng_.uniform01();
                // The running measure
                double runningRelativeMeasure = 0.0;

                // The probability of using each PHS is weighted by it's measure. Therefore, if we iterate up the list
                // of PHSs, the first one who's relative measure is greater than the PHS randomly selected
                for (std::list<ompl::ProlateHyperspheroidPtr>::const_iterator phsIter = listPhsPtrs_.begin();
                     phsIter != listPhsPtrs_.end() && !static_cast<bool>(rval); ++phsIter)
                {
                    // Update the running measure
                    runningRelativeMeasure = runningRelativeMeasure + (*phsIter)->getPhsMeasure() / summedMeasure_;

                    // Check if it's now greater than the proportion of the summed measure
                    if (runningRelativeMeasure > randDbl)
                    {
                        // It is, return this PHS:
                        rval = *phsIter;
                    }
                    // No else, continue
                }
            }

            // Return
            return rval;
        }

        bool PathLengthDirectInfSampler::keepSample(const std::vector<double> &informedVector)
        {
            // Variable
            // The return value, do we keep this sample? Start true.
            bool keep = true;

            // Is there more than 1 goal?
            if (listPhsPtrs_.size() > 1u)
            {
                // There is, do work

                // Variable
                // The number of PHSs the sample is in
                unsigned int numIn = numberOfPhsInclusions(informedVector);
                // The random number between [0,1]
                double randDbl = rng_.uniform01();

                // Keep the sample if the random number is less than 1/K
                keep = (randDbl <= 1.0 / static_cast<double>(numIn));
            }
            // No else, keep is true by default.

            return keep;
        }

        bool PathLengthDirectInfSampler::isInAnyPhs(const std::vector<double> &informedVector) const
        {
            // Variable
            // The return value, whether the given state is in any PHS
            bool inPhs = false;

            // Iterate over the list, stopping as soon as we get our first true
            for (auto phsIter = listPhsPtrs_.begin(); phsIter != listPhsPtrs_.end() && !inPhs; ++phsIter)
            {
                inPhs = isInPhs(*phsIter, informedVector);
            }

            return inPhs;
        }

        bool PathLengthDirectInfSampler::isInPhs(const ProlateHyperspheroidCPtr &phsCPtr,
                                                 const std::vector<double> &informedVector) const
        {
            return phsCPtr->isInPhs(&informedVector[0]);
        }

        unsigned int PathLengthDirectInfSampler::numberOfPhsInclusions(const std::vector<double> &informedVector) const
        {
            // Variable
            // The return value, the number of PHSs the vector is in
            unsigned int numInclusions = 0u;

            // Iterate over the list counting
            for (const auto &phsPtr : listPhsPtrs_)
            {
                // Conditionally increment
                if (phsPtr->isInPhs(&informedVector[0]))
                {
                    ++numInclusions;
                }
                // No else
            }

            return numInclusions;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    };  // namespace base
};  // namespace ompl
