//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

#include "Planar2DOF.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"

#include <fcl/distance.h>
#include <glog/logging.h>

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

	for (size_t i = 0; i < 2; ++i)
	{
		float lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		float upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits_.emplace_back(std::vector<float>({lower, upper}));
	}

	for (size_t i = 0; i < links.size()-1; ++i)
	{
		
		if (links[i]->visual->geometry->type == urdf::Geometry::BOX)
		{
			auto box = (std::shared_ptr<urdf::Box>&) links[i]->visual->geometry;
			KDL::Vector origin(links[i]->visual->origin.position.x, 
							   links[i]->visual->origin.position.y,
							   links[i]->visual->origin.position.z);

			
			CollisionGeometryPtr fclBox(new fcl::Box(box->dim.x, box->dim.y, box->dim.z));
			//std::cout << "origin: " << origin << std::endl;
			
			init_poses.emplace_back( KDL::Frame(origin));
			parts_.emplace_back( new fcl::CollisionObject(
				fclBox, fcl::Transform3f()
			));
		}
	}
	//std::cout << "constructor----------------------\n";
	robot_tree.getChain("base_link", "tool", robot_chain);
	Eigen::Vector2f state; state << 0.0, 0.0;
	setState(std::make_shared<base::RealVectorSpaceState>(state));
}

const KDL::Tree& robots::Planar2DOF::getRobotTree() const 
{
    return robot_tree;
}

const std::vector<std::unique_ptr<fcl::CollisionObject>>& robots::Planar2DOF::getParts() const
{
	return parts_;
}

std::shared_ptr<std::vector<KDL::Frame>> robots::Planar2DOF::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	KDL::Chain robot_chain;
	KDL::TreeFkSolverPos_recursive treefksolver = KDL::TreeFkSolverPos_recursive(robot_tree);
	std::shared_ptr<std::vector<KDL::Frame>> framesFK = std::make_shared<std::vector<KDL::Frame>>();
	robot_tree.getChain("base_link", "tool", robot_chain);
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions() );

	for (size_t i = 0; i < q->getDimensions(); ++i)
	{
		jointpositions(i) = q->getCoord()(i);
	}
	
	for (size_t i = 0; i < robot_tree.getNrOfSegments(); ++i)
	{
		KDL::Frame cartpos;
		bool kinematics_status = treefksolver.JntToCart(jointpositions, cartpos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
			framesFK->emplace_back(cartpos);
	}
	return framesFK;
    
}

std::shared_ptr<Eigen::MatrixXf> robots::Planar2DOF::computeXYZ(std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames = computeForwardKinematics(q);
	std::shared_ptr<Eigen::MatrixXf> XYZ = std::make_shared<Eigen::MatrixXf>(3, getParts().size() + 1);
	for (int k = 0; k <= getParts().size(); k++)
	{
		XYZ->col(k) << frames->at(k).p(0), frames->at(k).p(1), frames->at(k).p(2);
	}
	return XYZ;
}

float robots::Planar2DOF::computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
									  std::shared_ptr<Eigen::MatrixXf> XYZ)
{
	// Assumes that number of links = number of DOFs
	float d = 0;
	float r;
	for (int i = 0; i < getParts().size(); i++)
	{
		r = 0;
		for (int k = i+1; k <= getParts().size(); k++)
		{
			r = std::max(r, (XYZ->col(k) - XYZ->col(i)).norm());
		}
		d += r * std::abs(q2->getCoord(i) - q1->getCoord(i));
	}
	return fi / d;
}

void robots::Planar2DOF::setState(std::shared_ptr<base::State> q_)
{
	q = q_;
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions());

	std::shared_ptr<std::vector<KDL::Frame>> framesFK = computeForwardKinematics(q);
	
	//transform Collision geometries
	//std::cout << parts_.size() << std::endl;
	KDL::Frame tf;
	for (size_t i = 0; i < parts_.size(); ++i)
	{
		tf = framesFK->at(i) * init_poses[i];
		//std::cout << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
						
		//std::cout << "fcl\n";
		parts_[i]->setTransform(KDL2fcl(tf));
		parts_[i]->computeAABB(); 
		//std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}
    
}

void robots::Planar2DOF::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box(1, 1, 0.05));
	fcl::Transform3f tf; tf.setTranslation(fcl::Vec3f(2.5, 0, 0));
	std::unique_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));

	for (size_t i = 0; i < parts_.size(); ++i)
	{
		fcl::DistanceRequest request;
		fcl::DistanceResult result;
		fcl::distance(parts_[i].get(), ob.get(), request, result);
		std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl;
		std::cout << "Distance from " << i << ": " << result.min_distance << std::endl << "-------------------------------" << std::endl;
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

KDL::Frame robots::Planar2DOF::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Quaternion3f q = in.getQuatRotation();
    fcl::Vec3f t = in.getTranslation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());

    return f;
}

const std::vector<std::vector<float>> &robots::Planar2DOF::getLimits() const
{
	return limits_;
}