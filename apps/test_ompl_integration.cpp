#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <CommandLine.h>
#include <ConfigurationReader.h>
#include <glog/logging.h>

#include <ompl/config.h>
#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include <Scenario.h>
#include <RealVectorSpaceState.h>
#include <Eigen/Dense>

// OMPL Planners includes ---------------------------------
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>


// --------------------------------------------------------

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *realVectorSpaceState = state->as<ob::RealVectorStateSpace::StateType>();

    std::cout << realVectorSpaceState->values[0] << "\t" << realVectorSpaceState->values[1] << std::endl;

    return true;
}

void plan(std::shared_ptr<scenario::Scenario> scenario, std::string scenario_file_path)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(scenario->getStateSpace()->getDimensions()));

    ob::RealVectorBounds bounds(scenario->getStateSpace()->getDimensions());
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

    // start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);

    for (size_t i = 0; i < scenario->getStateSpace()->getDimensions(); ++i)
    {
        start->as<ob::RealVectorStateSpace::StateType>()->values[i] = scenario->getStart()->getCoord(i);
        goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = scenario->getGoal()->getCoord(i);

    }
    
    ss.setStartAndGoalStates(start, goal);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    double runtime_limit = 20, memory_limit = 1024;
    int run_count = 30;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, scenario_file_path);

    b.addPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRTXstatic>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRTsharp>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::LBTRRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::InformedRRTstar>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::FMT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::BITstar>(ss.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile("/tmp/omplBenchmark.log");
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenario_file_path = "data/planar_2dof/scenario_easy.yaml";
	bool print_help = false;
	CommandLine args("Test RBTConnect command line parser.");
	args.addArgument({"-s", "--scenario"}, &scenario_file_path, "Scenario .yaml description file path");
	args.addArgument({"-h", "--help"},     &print_help,
      "Use --scenario scenario_yaml_file_path to "
      "run with different scenario");

	try
	{
		args.parse(argc, argv);
	}
	catch (std::runtime_error const &e)
	{
		LOG(INFO) << e.what() << std::endl;
		return -1;
	}

	// When oPrintHelp was set to true, we print a help message and exit.
	if (print_help)
	{
		args.printHelp();
		return 0;
	}

	ConfigurationReader::initConfiguration();
	scenario::Scenario scenario(scenario_file_path);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Using scenario: " << scenario_file_path;
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();

	try
	{
		std::shared_ptr<scenario::Scenario> scenario = std::make_shared<scenario::Scenario>(scenario_file_path);
        LOG(INFO) << "OMPL version: " << OMPL_VERSION;

        LOG(INFO) << "Space state size: " << scenario->getStateSpace()->getDimensions();

        plan(scenario, scenario_file_path); 

	}
	catch (std::domain_error &e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}