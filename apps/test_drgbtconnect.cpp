#include <AbstractPlanner.h>
#include <DRGBTConnect.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

float getMean(std::vector<float> &v);

float getStd(std::vector<float> &v);

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	// std::string scenario_file_path = "data/planar_2dof/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "data/planar_2dof/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "data/planar_2dof/scenario2/scenario2.yaml";
	std::string scenario_file_path = "data/xarm6/scenario_test/scenario_test.yaml";
	// std::string scenario_file_path = "data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = "data/xarm6/scenario2/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RGBTConnect command line parser.");
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

	std::unique_ptr<planning::AbstractPlanner> planner;
	std::vector<float> routine_times, replanning_times;
	int num_test = 0;

	while (num_test++ < 1)
	{
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
			LOG(INFO) << "\n\nNum. test: " << num_test;
			planner = std::make_unique<planning::rbt::DRGBTConnect>(ss, scenario.getStart(), scenario.getGoal());
			bool res = planner->solve();
			LOG(INFO) << "DRGBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of iterations: " << planner->getPlannerInfo()->getNumIterations();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
			// if (res)
			// {
			// 	LOG(INFO) << "Path: ";
			// 	std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			// 	for (int i = 0; i < path.size(); i++)
			// 		std::cout << path.at(i)->getCoord().transpose() << std::endl;
			// }
			planner->outputPlannerData("../" + scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log");

			// std::vector<float> T = planner->getPlannerInfo()->getRoutineTimes();
			// for (float t : T)
			// 	routine_times.emplace_back(t);
			// T = planner->getPlannerInfo()->getReplanningTimes();
			// for (float t : T)
			// 	replanning_times.emplace_back(t);
		}
		catch (std::domain_error &e)
		{
			LOG(ERROR) << e.what();
		}
	}
	// LOG(INFO) << "Planning time of the routine (gBur): " << getMean(routine_times) << " +- " << getStd(routine_times)
	// 		  << ". Size " << routine_times.size();
	// LOG(INFO) << "Planning time of the routine (replanning): " << getMean(replanning_times) << " +- " << getStd(replanning_times)
	// 		  << ". Size " << replanning_times.size();
	
	google::ShutDownCommandLineFlags();
	return 0;
}

float getMean(std::vector<float> &v)
{
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	return sum / v.size();
}

float getStd(std::vector<float> &v)
{
	float mean = getMean(v);
	float sum = 0;
	for (int i = 0; i < v.size(); i++)
		sum += (v[i] - mean) * (v[i] - mean);

	return std::sqrt(sum / v.size());
}
