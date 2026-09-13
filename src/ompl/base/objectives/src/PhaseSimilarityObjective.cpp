/*
author: @shuaiyy
*/

#include "ompl/base/objectives/PhaseSimilarityObjective.h"

#include <cmath>
#include <stdexcept>

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Console.h"

ompl::base::PhaseSimilarityObjective::PhaseSimilarityObjective(const SpaceInformationPtr &si,
                                                               bool enableMotionCostInterpolation)
  : StateCostIntegralObjective(si, enableMotionCostInterpolation)
{
    description_ = "Phase Similarity";

    const auto *compound = dynamic_cast<const CompoundStateSpace *>(si_->getStateSpace().get());
    if (compound != nullptr && compound->getSubspaceCount() == 2)
    {
        const auto *phase = dynamic_cast<const RealVectorStateSpace *>(compound->getSubspace(1).get());
        if (phase != nullptr && phase->getDimension() == 1)
        {
            canonicalLayout_ = true;
            configurationSpace_ = compound->getSubspace(0);
            interpolatedReference_ = configurationSpace_->allocState();
        }
    }
    if (!canonicalLayout_)
        OMPL_WARN("PhaseSimilarityObjective: state space is not "
                  "Compound(configuration, RealVector(1) phase). "
                  "Using the flattened last-value alpha fallback; wrap the "
                  "configuration space if that was not intended.");
}

ompl::base::PhaseSimilarityObjective::~PhaseSimilarityObjective()
{
    clearReferenceStates();
    if (interpolatedReference_ != nullptr)
        configurationSpace_->freeState(interpolatedReference_);
}

void ompl::base::PhaseSimilarityObjective::clearReferenceStates()
{
    if (configurationSpace_)
        for (auto *state : referenceStates_)
            configurationSpace_->freeState(state);
    referenceStates_.clear();
}

void ompl::base::PhaseSimilarityObjective::setReference(const std::vector<std::vector<double>> &waypoints)
{
    if (waypoints.size() < 2)
        throw std::invalid_argument("PhaseSimilarityObjective: need at least 2 reference waypoints");

    const std::size_t dim = waypoints.front().size();
    for (const auto &row : waypoints)
        if (row.size() != dim)
            throw std::invalid_argument("PhaseSimilarityObjective: reference rows have unequal lengths");

    const unsigned int expectedDimension =
        canonicalLayout_ ? configurationSpace_->getDimension() : si_->getStateDimension() - 1;
    if (dim != expectedDimension)
        throw std::invalid_argument("PhaseSimilarityObjective: reference width does not match the configuration "
                                    "space dimension");

    clearReferenceStates();
    reference_ = waypoints;

    if (canonicalLayout_)
    {
        configurationSpace_->setup();
        try
        {
            for (const auto &waypoint : reference_)
            {
                State *state = configurationSpace_->allocState();
                referenceStates_.push_back(state);
                configurationSpace_->copyFromReals(state, waypoint);
                configurationSpace_->enforceBounds(state);
            }
        }
        catch (...)
        {
            clearReferenceStates();
            reference_.clear();
            throw;
        }
    }
}

std::vector<double> ompl::base::PhaseSimilarityObjective::xi(double alpha) const
{
    alpha = std::min(std::max(alpha, 0.0), 1.0);
    const double position = alpha * static_cast<double>(reference_.size() - 1);
    const auto low = static_cast<std::size_t>(position);
    const auto high = std::min(low + 1, reference_.size() - 1);
    const double frac = position - static_cast<double>(low);

    std::vector<double> out(reference_[low].size());
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = (1.0 - frac) * reference_[low][i] + frac * reference_[high][i];
    return out;
}

ompl::base::Cost ompl::base::PhaseSimilarityObjective::stateCost(const State *s) const
{
    if (reference_.size() < 2)
        throw std::runtime_error("PhaseSimilarityObjective: no reference set. Call setReference() first.");

    if (canonicalLayout_)
    {
        const auto *compound = s->as<CompoundState>();
        const double alpha = compound->as<RealVectorStateSpace::StateType>(1)->values[0];
        const double clampedAlpha = std::min(std::max(alpha, 0.0), 1.0);
        const double position = clampedAlpha * static_cast<double>(referenceStates_.size() - 1);
        const auto low = static_cast<std::size_t>(position);
        const auto high = std::min(low + 1, referenceStates_.size() - 1);
        configurationSpace_->interpolate(referenceStates_[low], referenceStates_[high],
                                         position - static_cast<double>(low), interpolatedReference_);
        return Cost(configurationSpace_->distance(compound->components[0], interpolatedReference_));
    }

    std::vector<double> values;
    si_->getStateSpace()->copyToReals(values, s);

    // Legacy flat-alpha fallback: Euclidean deviation on flattened reals.
    if (values.size() != reference_.front().size() + 1)
        throw std::runtime_error("PhaseSimilarityObjective: state dimension does not match "
                                 "reference width + 1 (config + alpha)");

    const double alpha = values.back() / alphaScale_;
    const std::vector<double> ref = xi(alpha);

    double sq = 0.0;
    for (std::size_t i = 0; i < ref.size(); ++i)
    {
        const double d = values[i] - ref[i];
        sq += d * d;
    }
    return Cost(std::sqrt(sq));
}
