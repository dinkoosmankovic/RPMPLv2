#include <RRTConnect.h>
#include <iostream>
#include <RealVectorSpace.h>
#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
	//base::RealVectorSpace *space = new base::RealVectorSpace(2);
	base::StateSpace *ss = new base::RealVectorSpace(2);
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "StateSpace Type: " << ss->getStateSpaceType();
	base::State* start = new base::RealVectorSpaceState(Eigen::Vector2f({0,0}));
	base::State* goal = new base::RealVectorSpaceState(Eigen::Vector2f({1,1}));
	planning::rrt::RRTConnect* planner = new planning::rrt::RRTConnect(ss, start, goal);
	LOG(INFO) << "RRTConnect initialized.";
	bool res = planner->solve();
	LOG(INFO) << "RRTConnect planning finished.";
	if (res)
	{
		LOG(INFO) << dynamic_cast<base::RealVectorSpaceState*>(planner->getStartTree().getStates()->at(0))->getCoord().transpose();
	}
	return 0;
}
