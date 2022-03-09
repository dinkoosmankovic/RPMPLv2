//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

#include "Planar2DOF.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"

#include <fcl/distance.h>
#include <Eigen/Dense>

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

	std::vector<KDL::Frame> framesFK = computeForwardKinematics(q);
	
	//transform Collision geometries
	//std::cout << parts_.size() << std::endl;
	KDL::Frame tf;
	for (size_t i = 0; i < parts_.size(); ++i)
	{
		tf = framesFK[i] * init_poses[i];
		//std::cout << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";
						
		//std::cout << "fcl\n";
		parts_[i]->setTransform(KDL2fcl(tf));
		parts_[i]->computeAABB(); 
		//std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}
    
}

void robots::Planar2DOF::test(std::shared_ptr<env::Environment> env, std::shared_ptr<base::State> q)
{
	setState(q);
	std::shared_ptr<fcl::CollisionObject> ob = env->getParts()[0];

	for (size_t i = 0; i < parts_.size(); ++i)
	{
		fcl::DistanceRequest request(true, 0.01, 0.02, fcl::GST_LIBCCD);
		fcl::DistanceResult result;
		fcl::distance(parts_[i].get(), ob.get(), request, result);
		//LOG(INFO) << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		LOG(INFO) << "distance from " << i + 1 << ": " << result.min_distance << " p1: " << transformPoint(result.nearest_points[0], parts_[i]->getTransform() ) << 
					"\t p2: " << transformPoint(result.nearest_points[2], ob->getTransform() );
	}

}

fcl::Vec3f robots::Planar2DOF::transformPoint(fcl::Vec3f& v, fcl::Transform3f t)
{
	fcl::Vec3f fclVec = t.getTranslation();
	Eigen::Vector4f trans = Eigen::Vector4f(fclVec[0], fclVec[1], fclVec[2], 1);
	fcl::Matrix3f rot = t.getRotation();
	Eigen::MatrixXf M = Eigen::MatrixXf::Identity(4,4);
	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 3; ++j)	
			M(i,j) = rot(i,j);

	for (size_t i = 0; i < 3; ++i)
		M(i,3) = fclVec[i];

	Eigen::Vector4f newVec = M * trans;
	return fcl::Vec3f(newVec(0), newVec(1), newVec(2));
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

std::vector<robots::LinkLimits> robots::Planar2DOF::getLimits() const
{
	return limits_;
}