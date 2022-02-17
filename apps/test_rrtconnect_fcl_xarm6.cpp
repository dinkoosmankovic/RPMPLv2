#include <RRTConnect.h>
#include <iostream>
#include <RealVectorSpaceFCL.h>
#include <Environment.h>
#include <xArm6.h>

#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
	//base::RealVectorSpace *space = new base::RealVectorSpace(2);
	std::shared_ptr<robots::xARM6> robot = std::make_shared<robots::xARM6>("data/xarm6/xarm6.urdf");
	std::shared_ptr<env::Environment> env = std::make_shared<env::Environment>( "data/xarm6/obstacles_easy.yaml" );

	LOG(INFO) << "env  parts: " << env->getParts().size();

	std::shared_ptr<base::StateSpace> ss = std::make_shared<base::RealVectorSpaceFCL>(6, robot, env);
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "StateSpace Type: " << ss->getStateSpaceType();
	Eigen::VectorXf startVec(6); startVec << M_PI/4, 0, 0, 0, 0, 0;
	Eigen::VectorXf goalVec(6); goalVec << -M_PI/4, 0, 0, 0, 0, 0;
	std::shared_ptr<base::State> start = std::make_shared<base::RealVectorSpaceState>(Eigen::VectorXf(startVec));
	std::shared_ptr<base::State> goal = std::make_shared<base::RealVectorSpaceState>(Eigen::VectorXf(goalVec));
	try
	{
		std::unique_ptr<planning::rrt::RRTConnect> planner = std::make_unique<planning::rrt::RRTConnect>(ss, start, goal);
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
