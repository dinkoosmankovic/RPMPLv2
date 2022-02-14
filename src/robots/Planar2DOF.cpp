//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

#include "Planar2DOF.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"

#include <fcl/distance.h>

typedef std::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr;

robots::Planar2DOF::~Planar2DOF() {}

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
			fcl::Vec3f origin(links[i]->visual->origin.position.x, 
							  links[i]->visual->origin.position.y,
							  links[i]->visual->origin.position.z);

			
			CollisionGeometryPtr fclBox(new fcl::Box(box->dim.x, box->dim.y, box->dim.z));
			fcl::Transform3f tf; //tf.setTranslation(origin);
			std::cout << "origin: " << origin << std::endl;
			std::cout << "tf: " << tf.getTranslation() << std::endl;
			
			if (i > 1)
				origin *= -1;

			init_poses.emplace_back(fcl::Transform3f(origin));
			parts_.emplace_back( new fcl::CollisionObject(
				fclBox, tf
			));
		}
	}
	robot_tree.getChain("base_link", "tool", robot_chain);
	setState(std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({0.0, 0.0})));
}

const KDL::Tree& robots::Planar2DOF::getRobotTree() const 
{
    return robot_tree;
}

const std::vector<std::unique_ptr<fcl::CollisionObject> >& robots::Planar2DOF::getParts() const
{
	return parts_;
}

std::vector<KDL::Frame> robots::Planar2DOF::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	KDL::Chain robot_chain;
	KDL::TreeFkSolverPos_recursive treefksolver = KDL::TreeFkSolverPos_recursive(robot_tree);
	std::vector<KDL::Frame> framesFK;
	robot_tree.getChain("base_link", "tool", robot_chain);
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimension() );

	for (size_t i = 0; i < q->getDimension(); ++i)
	{
		jointpositions(i) = q->getCoord()(i);
	}
	
	for (size_t i = 0; i < robot_tree.getNrOfSegments(); ++i)
	{
		KDL::Frame cartpos;
		bool kinematics_status = treefksolver.JntToCart(jointpositions, cartpos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
			framesFK.emplace_back(cartpos);		
	}
	return framesFK;
    
}

void robots::Planar2DOF::setState(std::shared_ptr<base::State> q_)
{
	q = q_;
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimension());

	//computeForwardKinematics()
	std::cout << "+++++++++++++++++++++++++++++++++++++++\n";
	std::vector<KDL::Frame> framesFK = computeForwardKinematics(q_);
	/*for (size_t i = 0; i < parts_.size(); ++i)
	{
		parts_[i]->computeAABB(); 
		// TODO: Have to move this somehow to (0,0,0)
		std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl;
	}*/
	//transform Collision geometries
	std::cout << parts_.size();
	fcl::Transform3f tf;
	for (size_t i = 0; i < parts_.size(); ++i)
	{
		fcl::Transform3f tf_link = KDL2fcl(framesFK[i]);
		std::cout << tf_link.getTranslation() << "\t" << tf_link.getRotation() << std::endl << std::endl;
		std::cout << init_poses[i].getTranslation() << "\t" << init_poses[i].getRotation() << std::endl << std::endl;
		tf *= tf_link * init_poses[i];
		
		parts_[i]->setTransform(tf);
		parts_[i]->computeAABB(); 
		std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}
    
}

void robots::Planar2DOF::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box(1.0, 0.001, 0.001));
	fcl::Transform3f tf; tf.setTranslation(fcl::Vec3f(2.6,0.0,0.0));
	std::unique_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));

	for (size_t i = 0; i < parts_.size(); ++i)
	{
		fcl::DistanceRequest request;
		fcl::DistanceResult result;
		fcl::distance(parts_[i].get(), ob.get(), request, result);
		std::cout << "distance from " << i << ": " << result.min_distance << std::endl;
	}

}

fcl::Transform3f robots::Planar2DOF::KDL2fcl(const KDL::Frame &in)
{
	fcl::Transform3f out;
    double x,y,z,w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vec3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternion3f q(w, x, y, z);
    out.setQuatRotation(q);
    out.setTranslation(t);
    return out;
}