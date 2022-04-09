#include <AbstractPlanner.h>
#include <RGBMTStar.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	// std::string scenarioFilePath = "data/planar_2dof/scenario_easy.yaml";
	// std::string scenarioFilePath = "data/planar_2dof/scenario1.yaml";
	// std::string scenarioFilePath = "data/planar_2dof/scenario2.yaml";
	// std::string scenarioFilePath = "data/xarm6/scenario_easy.yaml";
	std::string scenarioFilePath = "data/xarm6/scenario1.yaml";
	// std::string scenarioFilePath = "data/xarm6/scenario2.yaml";

	bool printHelp = false;
	CommandLine args("Test RGBMTStar command line parser.");
	args.addArgument({"-s", "--scenario"}, &scenarioFilePath, "Scenario .yaml description file path");
	args.addArgument({"-h", "--help"},     &printHelp,
      "Use --scenario scenario_yaml_file_path to "
      "run with different scenario");

	try
	{
		args.parse(argc, argv);
	}
	catch (std::runtime_error const &e)
	{
		std::cout << e.what() << std::endl;
		return -1;
	}

	// When oPrintHelp was set to true, we print a help message and exit.
	if (printHelp)
	{
		args.printHelp();
		return 0;
	}

	ConfigurationReader::initConfiguration();
	scenario::Scenario scenario(scenarioFilePath);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Using scenario: " << scenarioFilePath;
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();

	try
	{
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBMTStar>(ss, scenario.getStart(), scenario.getGoal());
				
		bool res = planner->solve();
		LOG(INFO) << "RGBMT* planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
		LOG(INFO) << "Number of nodes: " << planner->getPlannerInfo()->getNumStates();
		LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
		LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
			
		if (res)
		{
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			for (int i = 0; i < path.size(); i++)
				std::cout << path.at(i)->getCoord().transpose() << std::endl;			
		}
		planner->outputPlannerData("/tmp/plannerData.log");

	}
	catch (std::domain_error &e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
