/*
Copied skeleton RRT.cpp
author: @shuaiyy`
*/


#include "ompl/geometric/planners/rrt/PhaseRRT.h"
#include <cmath>
#include <limits>
#include <string>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::PhaseRRT::PhaseRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "PhaseRRTintermediate" : "PhaseRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &PhaseRRT::setRange, &PhaseRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &PhaseRRT::setGoalBias, &PhaseRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &PhaseRRT::setIntermediateStates, &PhaseRRT::getIntermediateStates,
                                "0,1");
    Planner::declareParam<double>("d_alpha_min", this, &PhaseRRT::setDAlphaMin, &PhaseRRT::getDAlphaMin, "0.:.001:1.");
    Planner::declareParam<double>("d_alpha_max", this, &PhaseRRT::setDAlphaMax, &PhaseRRT::getDAlphaMax, "0.:.01:1.");
    Planner::declareParam<double>("phase_lambda", this, &PhaseRRT::setPhaseLambda, &PhaseRRT::getPhaseLambda, "0.:.05:1.");
    Planner::declareParam<double>("sample_sigma", this, &PhaseRRT::setSampleSigma, &PhaseRRT::getSampleSigma, "0.:.01:10.");
    Planner::declareParam<double>("uniform_fraction", this, &PhaseRRT::setUniformFraction, &PhaseRRT::getUniformFraction,
                                  "0.:.05:1.");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::PhaseRRT::~PhaseRRT()
{
    freeMemory();
}

void ompl::geometric::PhaseRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::PhaseRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    /* angular slots index a reference row, so they must fit inside one */
    if (!angularDims_.empty() && !reference_.empty())
        for (const auto d : angularDims_)
            if (d >= reference_.front().size())
                throw Exception("PhaseRRT: setAngularDims() index " + std::to_string(d) +
                                " is outside the reference width (" +
                                std::to_string(reference_.front().size()) + ")");

    // Linear scan, NOT the auto-selected GNAT: the directed phase metric is
    // asymmetric (infinity one way), which breaks smart NN structures.
    if (!nn_)
        nn_.reset(new NearestNeighborsLinear<Motion *>());
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::PhaseRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::PhaseRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // the sampler reads xi(alpha), so a demonstration is mandatory
    if (reference_.size() < 2)
    {
        OMPL_ERROR("%s: no reference set. Call setReference() before solve().",
                   getName().c_str());
        return base::PlannerStatus::ABORT;
    }

    // a demo row plus alpha must fill the state exactly, or copyFromReals
    // would write past the end
    if (reference_.front().size() + 1 != si_->getStateDimension())
    {
        OMPL_ERROR("%s: reference has %u values per row but the state space "
                   "expects %u (config + 1 alpha).",
                   getName().c_str(), (unsigned int)reference_.front().size(),
                   (unsigned int)si_->getStateDimension());
        return base::PlannerStatus::ABORT;
    }

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else if (rng_.uniform01() < uniformFraction_)
        {
            /* uniform fraction: sample anywhere, independent of the demonstration */
            sampler_->sampleUniform(rstate);
        }
        else
        {
            /* phase-conditioned sampling: draw a phase on the grid first, then
               a configuration near what the demonstration does at that phase */
            const double alpha = std::round(rng_.uniform01() * 100.0) / 100.0;
            std::vector<double> values = xi(alpha);

            /* scatter around the reference pose */
            for (auto &v : values)
                v += rng_.gaussian(0.0, sampleSigma_);

            /* the state carries its phase as the last component */
            values.push_back(alpha);
            si_->getStateSpace()->copyFromReals(rstate, values);

            /* noise may leave the bounds; pull the state back in */
            si_->getStateSpace()->enforceBounds(rstate);
        }

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        /* phase-advance bound: clamp the new phase into
           [parent + dAlphaMin_, parent + dAlphaMax_], cap it at 1, then snap to
           the grid, which also regularises the arbitrary phase of a uniform
           sample. dAlphaMin_ below one grid step rounds back to a stall. */
        const double alphaNear = getAlpha(nmotion->state);
        double alphaNew = std::min(getAlpha(dstate), alphaNear + dAlphaMax_);
        alphaNew = std::max(alphaNew, alphaNear + dAlphaMin_);
        setAlpha(dstate, std::round(std::min(alphaNew, 1.0) * 100.0) / 100.0);

        if (si_->checkMotion(nmotion->state, dstate))
        {
            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;
            }

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::PhaseRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
