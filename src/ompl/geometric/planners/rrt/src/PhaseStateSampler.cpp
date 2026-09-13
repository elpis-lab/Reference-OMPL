/*
author: @shuaiyy
*/

#include "ompl/geometric/planners/rrt/PhaseStateSampler.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"

#include <cmath>
#include <string>

ompl::geometric::PhaseStateSampler::PhaseStateSampler(const base::StateSpacePtr &space)
  : base::StateSampler(space.get()), stateSpace_(space)
{
    defaultSampler_ = stateSpace_->allocDefaultStateSampler();

    const auto *compound = dynamic_cast<const base::CompoundStateSpace *>(stateSpace_.get());
    if (compound != nullptr && compound->getSubspaceCount() == 2)
    {
        const auto *phase = dynamic_cast<const base::RealVectorStateSpace *>(compound->getSubspace(1).get());
        if (phase != nullptr && phase->getDimension() == 1)
        {
            canonicalLayout_ = true;
            configurationSpace_ = compound->getSubspace(0);
            configurationSampler_ = configurationSpace_->allocDefaultStateSampler();
            interpolatedConfiguration_ = configurationSpace_->allocState();
        }
    }
}

ompl::geometric::PhaseStateSampler::~PhaseStateSampler()
{
    clearReferenceStates();
    if (interpolatedConfiguration_ != nullptr)
        configurationSpace_->freeState(interpolatedConfiguration_);
}

void ompl::geometric::PhaseStateSampler::clearReferenceStates()
{
    if (configurationSpace_)
        for (auto *state : referenceStates_)
            configurationSpace_->freeState(state);
    referenceStates_.clear();
}

void ompl::geometric::PhaseStateSampler::setReference(const std::vector<std::vector<double>> &waypoints)
{
    if (waypoints.size() < 2)
        throw Exception("PhaseStateSampler needs at least 2 reference waypoints");
    const std::size_t dimension = waypoints.front().size();
    for (const auto &waypoint : waypoints)
        if (waypoint.size() != dimension)
            throw Exception("PhaseStateSampler reference rows have inconsistent sizes");

    const unsigned int expectedDimension =
        canonicalLayout_ && !flatAlpha_ ? configurationSpace_->getDimension() : stateSpace_->getDimension() - 1;
    if (dimension != expectedDimension)
        throw Exception("PhaseStateSampler reference width does not match the configuration space dimension");
    if (!sampleWeights_.empty() && sampleWeights_.size() != dimension)
        throw Exception("PhaseStateSampler sample weight count must match the reference width");

    clearReferenceStates();
    reference_ = waypoints;
    if (canonicalLayout_ && !flatAlpha_)
    {
        configurationSpace_->setup();
        try
        {
            for (const auto &waypoint : reference_)
            {
                base::State *state = configurationSpace_->allocState();
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

void ompl::geometric::PhaseStateSampler::setSampleWeights(const std::vector<double> &weights)
{
    if (!reference_.empty() && !weights.empty() && weights.size() != reference_.front().size())
        throw Exception("PhaseStateSampler sample weight count must match the reference width");
    sampleWeights_ = weights;
}

void ompl::geometric::PhaseStateSampler::setFlatAlpha(double scale)
{
    if (scale == 0.0)
        throw Exception("PhaseStateSampler flat alpha scale must be nonzero");
    flatAlpha_ = true;
    alphaScale_ = scale;
    alphaIndex_ = stateSpace_->getDimension() - 1;
    clearReferenceStates();
}

bool ompl::geometric::PhaseStateSampler::hasCompatibleLayout() const
{
    return (canonicalLayout_ && !flatAlpha_) || flatAlpha_;
}

std::string ompl::geometric::PhaseStateSampler::diagnoseLayout() const
{
    if (flatAlpha_)
        return {};
    if (canonicalLayout_)
        return {};

    const auto *compound = dynamic_cast<const base::CompoundStateSpace *>(stateSpace_.get());
    if (compound == nullptr)
        return "state space is not a CompoundStateSpace; wrap the configuration as "
               "Compound(configuration, RealVectorStateSpace(1) phase)";
    if (compound->getSubspaceCount() < 2)
        return "compound space must have a configuration subspace plus a 1-D phase subspace";

    const unsigned int last = compound->getSubspaceCount() - 1;
    const auto *phase = dynamic_cast<const base::RealVectorStateSpace *>(compound->getSubspace(last).get());
    if (phase == nullptr)
        return "last subspace must be RealVectorStateSpace(1) for the phase / alpha";
    if (phase->getDimension() != 1)
        return "last subspace (phase / alpha) must be 1-dimensional";
    if (compound->getSubspaceCount() != 2)
        return "expected exactly two top-level subspaces: [configuration, RealVector(1) phase]";
    return "state space is not a valid phase-augmented layout";
}

std::vector<double> ompl::geometric::PhaseStateSampler::flatReferenceAt(double alpha) const
{
    alpha = std::max(0.0, std::min(1.0, alpha));
    const double position = alpha * static_cast<double>(reference_.size() - 1);
    const auto low = static_cast<std::size_t>(position);
    const auto high = std::min(low + 1, reference_.size() - 1);
    const double fraction = position - static_cast<double>(low);
    std::vector<double> values(reference_[low].size());
    for (std::size_t index = 0; index < values.size(); ++index)
        values[index] =
            (1.0 - fraction) * reference_[low][index] + fraction * reference_[high][index];
    return values;
}

void ompl::geometric::PhaseStateSampler::projectIfNeeded(base::State *state) const
{
    if (projection_)
        projection_->project(state);
}

void ompl::geometric::PhaseStateSampler::sampleUniform(base::State *state)
{
    if (reference_.size() < 2)
        throw Exception("PhaseStateSampler has no reference");
    if (!hasCompatibleLayout())
        throw Exception("PhaseStateSampler state must use Compound(configuration, RealVector(1)) or setFlatAlpha()");
    if (phaseGrid_ <= 0.0)
        throw Exception("PhaseStateSampler phase grid must be positive");

    const double alpha = std::round(rng_.uniform01() * phaseGrid_) / phaseGrid_;
    if (canonicalLayout_ && !flatAlpha_)
    {
        const double position = alpha * static_cast<double>(referenceStates_.size() - 1);
        const auto low = static_cast<std::size_t>(position);
        const auto high = std::min(low + 1, referenceStates_.size() - 1);
        configurationSpace_->interpolate(referenceStates_[low], referenceStates_[high],
                                         position - static_cast<double>(low), interpolatedConfiguration_);

        auto *configuration = state->as<base::CompoundState>()->components[0];
        if (sampleWeights_.empty())
            configurationSampler_->sampleGaussian(configuration, interpolatedConfiguration_, sampleSigma_);
        else
        {
            std::vector<double> values;
            configurationSpace_->copyToReals(values, interpolatedConfiguration_);
            for (std::size_t index = 0; index < values.size(); ++index)
                values[index] += rng_.gaussian(0.0, sampleSigma_ * sampleWeights_[index]);
            configurationSpace_->copyFromReals(configuration, values);
        }
        configurationSpace_->enforceBounds(configuration);
        setAlpha(state, alpha);
    }
    else
    {
        std::vector<double> values = flatReferenceAt(alpha);
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            const double weight = sampleWeights_.empty() ? 1.0 : sampleWeights_[index];
            values[index] += rng_.gaussian(0.0, sampleSigma_ * weight);
        }
        values.push_back(alpha * alphaScale_);
        stateSpace_->copyFromReals(state, values);
        stateSpace_->enforceBounds(state);
    }

    projectIfNeeded(state);
}

void ompl::geometric::PhaseStateSampler::sampleUniformNear(base::State *state, const base::State *near, double distance)
{
    defaultSampler_->sampleUniformNear(state, near, distance);
    projectIfNeeded(state);
}

void ompl::geometric::PhaseStateSampler::sampleGaussian(base::State *state, const base::State *mean, double stdDev)
{
    defaultSampler_->sampleGaussian(state, mean, stdDev);
    projectIfNeeded(state);
}

double ompl::geometric::PhaseStateSampler::getAlpha(const base::State *state) const
{
    if (flatAlpha_)
        return *stateSpace_->getValueAddressAtIndex(const_cast<base::State *>(state), alphaIndex_) / alphaScale_;
    return state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1)->values[0];
}

void ompl::geometric::PhaseStateSampler::setAlpha(base::State *state, double alpha) const
{
    if (flatAlpha_)
    {
        *stateSpace_->getValueAddressAtIndex(state, alphaIndex_) = alpha * alphaScale_;
        return;
    }
    state->as<base::CompoundState>()->as<base::RealVectorStateSpace::StateType>(1)->values[0] = alpha;
}
