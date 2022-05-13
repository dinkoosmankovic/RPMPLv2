#include <AbstractPlanner.h>
#include <RGBMTStar.h>
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

	// std::string scenario_file_path = "data/planar_2dof/scenario_easy.yaml";
	std::string scenario_file_path = "data/planar_2dof/scenario1.yaml";
	// std::string scenario_file_path = "data/planar_2dof/scenario2.yaml";
	// std::string scenario_file_path = "data/planar_2dof/scenario3.yaml";
	// std::string scenario_file_path = "data/xarm6/scenario_easy.yaml";
	// std::string scenario_file_path = "data/xarm6/scenario1.yaml";
	// std::string scenario_file_path = "data/xarm6/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RGBMTStar command line parser.");
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

	int max_num_tests = 30;
	int num_test = 0;
	int num_success = 0;
	std::vector<float> path_costs;
	std::vector<float> planning_times;
	std::vector<float> found_states;
	std::vector<float> found_states_times;
	std::unique_ptr<planning::AbstractPlanner> planner;

	while (num_test++ < max_num_tests)
	{
		try
		{
			planner = std::make_unique<planning::rbt::RGBMTStar>(ss, scenario.getStart(), scenario.getGoal());					
			bool res = planner->solve();

			LOG(INFO) << "Test number " << num_test;
			LOG(INFO) << "RGBMT* planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
			LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
			LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
			if (res)
			{
				num_success++;
				LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
				path_costs.emplace_back(planner->getPlannerInfo()->getCostConvergence().back());
				planning_times.emplace_back(planner->getPlannerInfo()->getPlanningTime());
				for (int i = 0; i < planner->getPlannerInfo()->getNumStates(); i++)
				{
					if (planner->getPlannerInfo()->getCostConvergence()[i] < INFINITY)
					{
						LOG(INFO) << "Path is found after " << i << " states:";
						found_states_times.emplace_back(planner->getPlannerInfo()->getStateTimes()[i]);
						found_states.emplace_back(i);
						break;
					}
				}
				std::vector<std::shared_ptr<base::State>> path = planner->getPath();
				for (int i = 0; i < path.size(); i++)
					LOG(INFO) << path.at(i)->getCoord().transpose() << std::endl;			
			}
			LOG(INFO) << "\n--------------------------------------------------------------------\n\n";
			planner->outputPlannerData("/tmp/planner_data" + scenario_file_path.substr(4, scenario_file_path.size()-9) + 
									   "/test" + std::to_string(num_test) + ".log");
		}
		catch (std::domain_error &e)
		{
			LOG(ERROR) << e.what();
		}
	}

	std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	output_file.open("/tmp/planner_data" + scenario_file_path.substr(4, scenario_file_path.size()-9) + "/results.log", mode);

	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Space dimension: " << ss->getDimensions() << std::endl;
		output_file << "Planner type:    " << "RGBMT*" << std::endl;
		output_file << "Using scenario:  " << scenario_file_path << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Success rate [%]:                                    " << (float) num_success / max_num_tests * 100  << std::endl;
		output_file << "\t Average path cost [rad]:                             " << getMean(path_costs) << " +- " << getStd(path_costs) << std::endl;
		output_file << "\t Average planning time [ms]:                          " << getMean(planning_times) << " +- " << getStd(planning_times) << std::endl;
		output_file << "\t Average time when the first path is found [ms]:      " << getMean(found_states_times) << " +- " << getStd(found_states_times) << std::endl;
		output_file << "\t Average num. of states when the first path is found: " << getMean(found_states) << " +- " << getStd(found_states) << std::endl;
		output_file << std::string(75, '-') << std::endl;		
		output_file.close();
	}
	else
		throw "Cannot open file"; // std::something exception perhaps?
	
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
