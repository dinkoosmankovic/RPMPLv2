#include <RGBMTStar.h>
#include <iostream>
#include <RealVectorSpaceFCL.h>
#include <Environment.h>
#include <Planar2DOF.h>
#include <Scenario.h>
#include <ConfigurationReader.h>

#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
	// std::shared_ptr<robots::Planar2DOF> robot = std::make_shared<robots::Planar2DOF>("data/planar_2dof/planar_2dof.urdf");
	// std::shared_ptr<env::Environment> env = std::make_shared<env::Environment>("data/planar_2dof/obstacles_easy.yaml" );
	// std::shared_ptr<base::StateSpace> ss = std::make_shared<base::RealVectorSpaceFCL>(2, robot, env);
	// std::shared_ptr<base::State> start = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({-M_PI/2, 0}));
	// std::shared_ptr<base::State> goal = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({M_PI/2, 0}));

	ConfigurationReader::initConfiguration();
	// scenario::Scenario scenario("data/planar_2dof/scenario_easy.yaml");
	// scenario::Scenario scenario("data/planar_2dof/scenario1.yaml");
	scenario::Scenario scenario("data/planar_2dof/scenario2.yaml");
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();
	try
	{
		std::unique_ptr<planning::rbt::RGBMTStar> planner = std::make_unique<planning::rbt::RGBMTStar>(ss, scenario.getStart(), scenario.getGoal());
		
		bool res = planner->solve();
		LOG(INFO) << "RGBMT* planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
		LOG(INFO) << "Number of nodes: " << planner->getPlannerInfo()->getNumStates();
		LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
		LOG(INFO) << "Path cost: " << planner->getPlannerInfo()->getCostConvergence().back();
			
		if (res)
		{
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			for (int i = 0; i < path.size(); i++)
			{
				std::cout << path.at(i)->getCoord().transpose() << std::endl;
			}
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
