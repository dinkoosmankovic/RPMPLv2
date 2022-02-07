#include <iostream>
#include <memory>

#include "Planar2DOF.h"
#include <glog/logging.h>

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;
	
	
	std::shared_ptr<robots::Planar2DOF> robot = std::make_shared<robots::Planar2DOF>("data/planar_2dof/planar_2dof.urdf");

	LOG(INFO) << robot->getRobotTree().getNrOfSegments();


	return 0;
}
