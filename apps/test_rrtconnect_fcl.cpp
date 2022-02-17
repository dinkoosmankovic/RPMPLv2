#include <RRTConnect.h>
#include <iostream>
#include <Scenario.h>
#include <ConfigurationReader.h>

#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	ConfigurationReader::initConfiguration();
	
	scenario::Scenario scenario("data/planar_2dof/scenario_easy.yaml");
	
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "StateSpace Type: " << ss->getStateSpaceType();
	try
	{
		std::unique_ptr<planning::rrt::RRTConnect> planner = std::make_unique<planning::rrt::RRTConnect>(ss, scenario.getStart(), scenario.getGoal());
		bool res = planner->solve();
		LOG(INFO) << "RRTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
		LOG(INFO) << "number of nodes: " << planner->getPlannerInfo()->getNumNodes();
			
		if (res)
		{
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			/*for (int i = 0; i < path.size(); i++)
			{
				std::cout << path.at(i) << std::endl;
			}*/
			// TODO: read from configuration yaml
			
		}
		planner->outputPlannerData("/tmp/plannerData.log");

	}
	catch (std::domain_error& e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
