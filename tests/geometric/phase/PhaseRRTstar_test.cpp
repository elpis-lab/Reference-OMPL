#define BOOST_TEST_MODULE "PhaseRRTstar"
#include <boost/test/unit_test.hpp>

#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/objectives/PhaseSimilarityObjective.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/terminationconditions/IterationTerminationCondition.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/PhaseRRTstar.h"
#include "ompl/geometric/planners/rrt/PhaseStateSampler.h"
#include "ompl/util/Exception.h"

#include <cmath>
#include <memory>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    struct PhaseSpace
    {
        PhaseSpace()
        {
            se2 = std::make_shared<ob::SE2StateSpace>();
            ob::RealVectorBounds xyBounds(2);
            xyBounds.setLow(-5.0);
            xyBounds.setHigh(5.0);
            se2->setBounds(xyBounds);

            joints = std::make_shared<ob::RealVectorStateSpace>(2);
            ob::RealVectorBounds jointBounds(2);
            jointBounds.setLow(-2.0);
            jointBounds.setHigh(2.0);
            joints->setBounds(jointBounds);

            configuration = std::make_shared<ob::CompoundStateSpace>();
            configuration->addSubspace(se2, 2.0);
            configuration->addSubspace(joints, 0.5);

            phase = std::make_shared<ob::RealVectorStateSpace>(1);
            ob::RealVectorBounds phaseBounds(1);
            phaseBounds.setLow(0.0);
            phaseBounds.setHigh(1.0);
            phase->setBounds(phaseBounds);

            space = std::make_shared<ob::CompoundStateSpace>();
            space->addSubspace(configuration, 1.0);
            space->addSubspace(phase, 0.25);
            space->setup();
        }

        std::shared_ptr<ob::SE2StateSpace> se2;
        std::shared_ptr<ob::RealVectorStateSpace> joints;
        std::shared_ptr<ob::CompoundStateSpace> configuration;
        std::shared_ptr<ob::RealVectorStateSpace> phase;
        std::shared_ptr<ob::CompoundStateSpace> space;
    };

    const std::vector<std::vector<double>> reference{
        {0.0, 0.0, 3.0, -1.0, 0.5},
        {2.0, 1.0, -3.0, 1.0, -0.5},
    };
}

BOOST_AUTO_TEST_CASE(CanonicalSamplerUsesConfigurationTopology)
{
    PhaseSpace fixture;
    og::PhaseStateSampler sampler(fixture.space);
    sampler.setReference(reference);
    sampler.setSampleSigma(0.0);
    sampler.setPhaseGrid(20.0);

    ob::ScopedState<> sampled(fixture.space);
    ob::ScopedState<> low(fixture.configuration);
    ob::ScopedState<> high(fixture.configuration);
    ob::ScopedState<> expected(fixture.configuration);
    fixture.configuration->copyFromReals(low.get(), reference[0]);
    fixture.configuration->copyFromReals(high.get(), reference[1]);

    for (unsigned int attempt = 0; attempt < 20; ++attempt)
    {
        sampler.sampleUniform(sampled.get());
        const double alpha = sampler.getAlpha(sampled.get());
        BOOST_CHECK_GE(alpha, 0.0);
        BOOST_CHECK_LE(alpha, 1.0);
        BOOST_CHECK_SMALL(alpha * 20.0 - std::round(alpha * 20.0), 1e-12);

        fixture.configuration->interpolate(low.get(), high.get(), alpha, expected.get());
        const auto *actual = sampled->as<ob::CompoundState>()->components[0];
        BOOST_CHECK_SMALL(fixture.configuration->distance(actual, expected.get()), 1e-10);
    }
}

BOOST_AUTO_TEST_CASE(CanonicalObjectiveUsesWeightedConfigurationDistance)
{
    PhaseSpace fixture;
    auto si = std::make_shared<ob::SpaceInformation>(fixture.space);
    si->setup();

    ob::PhaseSimilarityObjective objective(si);
    objective.setReference(reference);

    ob::ScopedState<> actual(fixture.space);
    ob::ScopedState<> low(fixture.configuration);
    ob::ScopedState<> high(fixture.configuration);
    ob::ScopedState<> interpolated(fixture.configuration);
    fixture.configuration->copyFromReals(low.get(), reference[0]);
    fixture.configuration->copyFromReals(high.get(), reference[1]);
    fixture.configuration->interpolate(low.get(), high.get(), 0.5, interpolated.get());

    std::vector<double> actualConfiguration;
    fixture.configuration->copyToReals(actualConfiguration, interpolated.get());
    actualConfiguration[0] += 0.4;
    actualConfiguration[3] += 0.6;
    fixture.configuration->copyFromReals(actual->as<ob::CompoundState>()->components[0], actualConfiguration);
    actual->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.5;

    const double expected =
        fixture.configuration->distance(actual->as<ob::CompoundState>()->components[0], interpolated.get());
    BOOST_CHECK_CLOSE(objective.stateCost(actual.get()).value(), expected, 1e-10);
}

BOOST_AUTO_TEST_CASE(ShortCanonicalSolve)
{
    PhaseSpace fixture;
    og::SimpleSetup setup(fixture.space);
    setup.setStateValidityChecker([](const ob::State *) { return true; });

    ob::ScopedState<> start(fixture.space);
    ob::ScopedState<> goal(fixture.space);
    fixture.configuration->copyFromReals(start->as<ob::CompoundState>()->components[0], reference.front());
    fixture.configuration->copyFromReals(goal->as<ob::CompoundState>()->components[0], reference.back());
    start->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
    goal->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 1.0;
    setup.setStartAndGoalStates(start, goal, 0.2);

    auto planner = std::make_shared<og::PhaseRRTstar>(setup.getSpaceInformation());
    planner->setReference(reference);
    planner->setGoalBias(1.0);
    planner->setRange(1.0);
    planner->setDAlphaMin(0.01);
    planner->setDAlphaMax(0.25);
    planner->setPhaseGrid(20.0);
    setup.setPlanner(planner);

    const ob::PlannerStatus status = setup.solve(ob::IterationTerminationCondition(500));
    BOOST_CHECK(status);
    BOOST_CHECK(setup.haveExactSolutionPath());
}

BOOST_AUTO_TEST_CASE(RejectsStateSpaceWithoutPhaseSubspace)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setup();

    og::PhaseRRTstar planner(si);
    BOOST_CHECK_THROW(planner.setup(), ompl::Exception);

    og::PhaseStateSampler sampler(space);
    BOOST_CHECK(!sampler.diagnoseLayout().empty());
}
