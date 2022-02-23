#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/config.h>
#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include <Scenario.h>
#include <RealVectorSpaceState.h>
#include <Eigen/Dense>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *realVectorSpaceState = state->as<ob::RealVectorStateSpace::StateType>();

    std::cout << realVectorSpaceState->values[0] << "\t" << realVectorSpaceState->values[1] << std::endl;

    return true;
}

void plan(std::shared_ptr<scenario::Scenario> scenario)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-M_PI);
    bounds.setHigh(M_PI);

    space->setBounds(bounds);

    ompl::geometric::SimpleSetup ss(space);

    ss.setStateValidityChecker([scenario, space](const ob::State *state)
                                { 
                                    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
                                    const auto *realVectorSpaceState = state->as<ob::RealVectorStateSpace::StateType>();
                                    int dim = space->getDimension();
                                    Eigen::VectorXf stateEigen(dim);
                                    
                                    for (size_t i = 0; i < space->getDimension(); ++i)
                                        stateEigen(i) = (float)realVectorSpaceState->values[i];


                                    std::shared_ptr<base::State> rpmplState = std::make_shared<base::RealVectorSpaceState>(stateEigen);
                                    return ss->isValid(rpmplState);
    });

    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -M_PI/2;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = -M_PI/2;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

    ss.setStartAndGoalStates(start, goal);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    /*auto planner(std::make_shared<og::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;*/

    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "scenario_easy_2dof" );
    b.addPlanner(std::make_shared<ompl::geometric::RRTConnect>(ss.getSpaceInformation()));
    b.benchmark(request);
    b.saveResultsToFile("/tmp/omplBenchmark.log");
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::shared_ptr<scenario::Scenario> scenario = std::make_shared<scenario::Scenario>("data/planar_2dof/scenario_easy.yaml");
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    plan(scenario);

    std::cout << std::endl;

    return 0;
}
