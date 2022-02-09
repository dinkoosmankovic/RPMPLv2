//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

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

	urdf::Model model;
	if (!model.initFile(robot_desc))
	{
    	throw std::runtime_error("Failed to parse urdf file");
	}
	std::cout << model.getName() << std::endl;
	std::vector<urdf::LinkSharedPtr > links;
	model.getLinks(links);

	for (size_t i = 0; i < links.size()-1; ++i)
	{
		if (links[i]->visual->geometry->type == urdf::Geometry::BOX)
		{
			auto box = (std::shared_ptr<urdf::Box>&) links[i]->visual->geometry;
			fcl::Box fclBox(box->dim.x, box->dim.y, box->dim.z);
			parts_.emplace_back(new fcl::CollisionObject(
				fclBox, fcl::Transform3f() 
			));
		}
	}
	
	robot_tree.getChain("base_link", "tool", robot_chain);
	
}

const KDL::Tree& robots::Planar2DOF::getRobotTree() const 
{
    return robot_tree;
}