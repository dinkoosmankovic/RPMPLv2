#include <RRTConnect.h>
#include <iostream>
#include <RealVectorSpace.h>
#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
	//base::RealVectorSpace *space = new base::RealVectorSpace(2);
	std::shared_ptr<base::StateSpace> ss = std::make_shared<base::RealVectorSpace>(2);
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "StateSpace Type: " << ss->getStateSpaceType();
	std::shared_ptr<base::State> start = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({0,0}));
	std::shared_ptr<base::State> goal = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({100,100}));
	try
	{
		std::unique_ptr<planning::rrt::RRTConnect> planner = std::make_unique<planning::rrt::RRTConnect>(ss, start, goal);
		LOG(INFO) << "RRTConnect initialized.";
		bool res = planner->solve();
		LOG(INFO) << "RRTConnect planning finished.";
		if (res)
		{
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			for (int i = 0; i < path.size(); i++)
			{
				std::cout << path.at(i) << std::endl;
			}
		}

	}
	catch (std::domain_error& e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
