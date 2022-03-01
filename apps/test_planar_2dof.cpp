#include <iostream>
#include <memory>

#include "Planar2DOF.h"
#include <glog/logging.h>
#include <RealVectorSpace.h>

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;
	
	
	std::shared_ptr<robots::Planar2DOF> robot = std::make_shared<robots::Planar2DOF>("data/planar_2dof/planar_2dof.urdf");

	LOG(INFO) << "Number of segments: " << robot->getRobotTree().getNrOfSegments();
	LOG(INFO) << "Number of joints: " << robot->getRobotTree().getNrOfJoints();
	LOG(INFO) << "Number of parts: " << robot->getParts().size();
	LOG(INFO) << "Robot volume: " << robot->getParts().at(0)->getCollisionGeometry()->computeVolume();
	LOG(INFO) << "Segment dimensions: " << robot->getParts().at(0)->getAABB().height() << "; " 
										<< robot->getParts().at(0)->getAABB().depth() << "; " 
										<< robot->getParts().at(0)->getAABB().width();


	// std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({0, 0}));
	// std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({M_PI/2, 0}));
	std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({M_PI/2, -M_PI/2}));
	// robot->setState(state);
	// robot->test();
	std::shared_ptr<std::vector<KDL::Frame>> frames = robot->computeForwardKinematics(state);
	for (int i = 0; i < frames->size(); i++)
	{
		std::cout << frames->at(i).p << std::endl;
		// std::cout << frames->at(i).M << std::endl;
	}


	return 0;
}
