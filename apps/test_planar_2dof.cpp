#include <iostream>
#include <memory>

#include "Planar2DOF.h"
#include <glog/logging.h>
#include <RealVectorSpace.h>
#include "Scenario.h"
#include "CommandLine.h"
#include "ConfigurationReader.h"

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int)time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string scenarioFilePath = "data/planar_2dof/scenario_easy.yaml";
	bool printHelp = false;

	CommandLine args("Test Planar2DOF command line parser.");
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

	

	std::shared_ptr<robots::Planar2DOF> robot =  std::dynamic_pointer_cast<robots::Planar2DOF>(scenario.getRobot());

	std::shared_ptr<base::State> testState = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({1.35,-2.0}));
	robot->test(scenario.getEnvironment(), testState);

	google::ShutDownCommandLineFlags();
	return 0;
}
