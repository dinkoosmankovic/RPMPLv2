//
// Created by dinko on 07.02.22.
//

#include "Planar2DOF.h"

robots::Planar2DOF::~Planar2DOF() {}

void robots::Planar2DOF::computeForwardKinematics(std::shared_ptr<base::State> q)
{
    
}

robots::Planar2DOF::Planar2DOF(std::string robot_desc)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
	{
		throw std::runtime_error("Failed to construct kdl tree");
	}
}

const KDL::Tree& robots::Planar2DOF::getRobotTree() const 
{
    return robot_tree;
}