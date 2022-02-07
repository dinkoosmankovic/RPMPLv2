//
// Created by dinko on 07.02.22.
//

#include <fstream>
#include <iostream>

#include "Planar2DOF.h"
#include "urdf/model.h"

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
	std::ifstream t(robot_desc);
	std::stringstream buffer;
	buffer << t.rdbuf();

	std::shared_ptr<urdf::UrdfModel> model = urdf::UrdfModel::fromUrdfStr(buffer.str());
	std::cout << model->getName();

	robot_tree.getChain("base_link", "tool", robot_chain);
	
}

const KDL::Tree& robots::Planar2DOF::getRobotTree() const 
{
    return robot_tree;
}