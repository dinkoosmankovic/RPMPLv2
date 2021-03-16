#include <RRTConnect.h>
#include <iostream>
#include <RealVectorSpace.h>
#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!" << std::endl;
	//base::RealVectorSpace *space = new base::RealVectorSpace(2);
	base::StateSpace *ss = new base::RealVectorSpace(2);
	planning::rrt::RRTConnect* planner = new planning::rrt::RRTConnect(ss);
	planner->solve();
	return 0;
}
