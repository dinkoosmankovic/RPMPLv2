#include <AbstractPlanner.h>
#include <RRTConnect.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include "Benchmark.h"

#include <glog/logging.h>

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int)time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenario_file_path = "data/planar_2dof/scenario_easy.yaml";
	bool print_help = false;

	CommandLine args("Test RRTConnect command line parser.");
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
		std::cout << e.what() << std::endl;
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
	for (size_t i = 0; i < 10; ++i)
	{
		benchmark::Benchmark benchmark;
		benchmark.addBenchmarkContext(std::make_pair(
			scenario,
			"RRTConnect"
		));
		try
		{
			benchmark.runBenchmark();		
		}
		catch (std::domain_error &e)
		{
			LOG(ERROR) << e.what();
		}
	}
	google::ShutDownCommandLineFlags();
	return 0;
}