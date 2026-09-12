/*
author: @shuaiyy
*/

#include "ompl/base/objectives/PhaseSimilarityObjective.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

ompl::base::PhaseSimilarityObjective::PhaseSimilarityObjective(const SpaceInformationPtr &si,
                                                               bool enableMotionCostInterpolation)
  : StateCostIntegralObjective(si, enableMotionCostInterpolation)
{
    description_ = "Phase Similarity";
}

void ompl::base::PhaseSimilarityObjective::setReference(const std::vector<std::vector<double>> &waypoints)
{
    if (waypoints.size() < 2)
        throw std::invalid_argument("PhaseSimilarityObjective: need at least 2 reference waypoints");

    const std::size_t dim = waypoints.front().size();
    for (const auto &row : waypoints)
        if (row.size() != dim)
            throw std::invalid_argument("PhaseSimilarityObjective: reference rows have unequal lengths");

    for (const auto d : angularDims_)
        if (d >= dim)
            throw std::invalid_argument("PhaseSimilarityObjective: angular dimension index is outside "
                                        "the reference width");

    reference_ = waypoints;
}

void ompl::base::PhaseSimilarityObjective::setAngularDims(const std::vector<unsigned int> &dims)
{
    if (!reference_.empty())
        for (const auto d : dims)
            if (d >= reference_.front().size())
                throw std::invalid_argument("PhaseSimilarityObjective: angular dimension index is outside "
                                            "the reference width");

    angularDims_ = dims;
}

double ompl::base::PhaseSimilarityObjective::angleDiff(double a, double b)
{
    const double d = b - a;
    return std::atan2(std::sin(d), std::cos(d));
}

bool ompl::base::PhaseSimilarityObjective::isAngular(std::size_t j) const
{
    return std::find(angularDims_.begin(), angularDims_.end(), static_cast<unsigned int>(j)) != angularDims_.end();
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
        out[i] = reference_[low][i] +
                 frac * (isAngular(i) ? angleDiff(reference_[low][i], reference_[high][i]) :
                                        reference_[high][i] - reference_[low][i]);
    return out;
}

ompl::base::Cost ompl::base::PhaseSimilarityObjective::stateCost(const State *s) const
{
    if (reference_.size() < 2)
        throw std::runtime_error("PhaseSimilarityObjective: no reference set. Call setReference() first.");

    std::vector<double> values;
    si_->getStateSpace()->copyToReals(values, s);

    // the state is (q, alpha) with alpha as the last value
    if (values.size() != reference_.front().size() + 1)
        throw std::runtime_error("PhaseSimilarityObjective: state dimension does not match "
                                 "reference width + 1 (config + alpha)");

    const double alpha = values.back() / alphaScale_;
    const std::vector<double> ref = xi(alpha);

    double sq = 0.0;
    for (std::size_t i = 0; i < ref.size(); ++i)
    {
        const double d = isAngular(i) ? angleDiff(ref[i], values[i]) : values[i] - ref[i];
        sq += d * d;
    }
    return Cost(std::sqrt(sq));
}
