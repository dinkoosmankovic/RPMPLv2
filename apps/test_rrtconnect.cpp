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
	base::StateSpace *ss = new base::RealVectorSpace(2);
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "StateSpace Type: " << ss->getStateSpaceType();
	base::State* start = new base::RealVectorSpaceState(Eigen::Vector2f({0,0}));
	base::State* goal = new base::RealVectorSpaceState(Eigen::Vector2f({100,100}));
	try
	{
		planning::rrt::RRTConnect *planner = new planning::rrt::RRTConnect(ss, start, goal);
		LOG(INFO) << "RRTConnect initialized.";
		bool res = planner->solve();
		LOG(INFO) << "RRTConnect planning finished.";
		if (res)
		{
			std::vector<base::State*> path = planner->getPath();
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
	return 0;
}
