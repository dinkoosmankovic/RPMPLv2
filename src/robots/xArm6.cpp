//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

#include "xArm6.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"

#include <fcl/distance.h>
#include <fcl/BVH/BVH_model.h>

#include <glog/logging.h>
#include "stl_reader.h"

typedef std::shared_ptr <fcl::CollisionGeometry> CollisionGeometryPtr;

robots::xARM6::~xARM6() {}

robots::xARM6::xARM6(std::string robot_desc)
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
	const size_t last_slash_idx = robot_desc.rfind('/');
	std::string urdf_root_path = robot_desc.substr(0, last_slash_idx) + "/";
	LOG(INFO) << "Creating " << model.getName();
	std::vector<urdf::LinkSharedPtr > links;
	model.getLinks(links);
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	
	for (size_t i = 0; i < robot_chain.getNrOfJoints(); ++i)
	{
		float lower = model.getJoint("joint"+std::to_string(i+1))->limits->lower;
		float upper = model.getJoint("joint"+std::to_string(i+1))->limits->upper;
		limits_.emplace_back(std::vector<float>({lower, upper}));
	}

	for (size_t i = 0; i < 6; ++i)
	{
		//LOG(INFO) << links[i]->name << "\t" << links[i]->collision->geometry->type;

		urdf::Pose pose = model.getJoint("joint"+std::to_string(i+1))->parent_to_joint_origin_transform;

		KDL::Vector pos(pose.position.x, pose.position.y, pose.position.z);
		double roll, pitch, yaw;
		KDL::Frame link_frame;
		link_frame.p = pos;
		link_frame.M = link_frame.M.RPY(roll, pitch, yaw);
		//LOG(INFO) << link_frame;
		if (links[i]->collision->geometry->type == urdf::Geometry::MESH)
		{
			fcl::Vec3f p[3];
			fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>;
    		model->beginModel();

			const auto mesh_ptr = dynamic_cast<const urdf::Mesh*>(links[i]->collision->geometry.get());
			
			stl_reader::StlMesh <float, unsigned int> mesh (urdf_root_path + mesh_ptr->filename);
			for (int j = 0; j < mesh.num_tris(); ++j)
			{

				for (size_t icorner = 0; icorner < 3; ++icorner) 
				{
					const float* c = mesh.vrt_coords (mesh.tri_corner_ind (j, icorner));
					//LOG(INFO) << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
					p[icorner] = fcl::Vec3f(c[0], c[1], c[2]); 
				}
				//LOG(INFO) << "+++++++++++++++++++++++++++++";
				model->addTriangle(p[0], p[1], p[2]);
			}
			model->endModel();
			CollisionGeometryPtr fclBox(model);
			parts_.emplace_back(new fcl::CollisionObject(
				fclBox, fcl::Transform3f()
			));
			init_poses.emplace_back(link_frame);

		}
	}
	//std::cout << "constructor----------------------\n";
	Eigen::VectorXf vec(6);
	vec << 0,0,0,0,0,0;
	setState(std::make_shared<base::RealVectorSpaceState>(vec));

}

const KDL::Tree& robots::xARM6::getRobotTree() const 
{
    return robot_tree;
}

const std::vector<std::unique_ptr<fcl::CollisionObject> >& robots::xARM6::getParts() const
{
	return parts_;
}

std::vector<KDL::Frame> robots::xARM6::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	//KDL::Chain robot_chain;
	KDL::TreeFkSolverPos_recursive treefksolver = KDL::TreeFkSolverPos_recursive(robot_tree);
	std::vector<KDL::Frame> framesFK;
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions() );

	for (size_t i = 0; i < q->getDimensions(); ++i)
	{
		jointpositions(i) = q->getCoord()(i);
	}

	for (size_t i = 0; i < robot_chain.getNrOfJoints(); ++i)
	{
		KDL::Frame cartpos;
		bool kinematics_status = treefksolver.JntToCart(jointpositions, cartpos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
			framesFK.emplace_back(cartpos);		
	}
	return framesFK;
    
}

void robots::xARM6::setState(std::shared_ptr<base::State> q_)
{
	q = q_;
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions());

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

void robots::xARM6::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box(0.5, 1.0, 3.0));
	fcl::Transform3f tf; tf.setTranslation(fcl::Vec3f(1, 1, 1.5));
	std::unique_ptr<fcl::CollisionObject> ob(new fcl::CollisionObject(fclBox, tf));
	for (size_t i = 0; i < parts_.size(); ++i)
	{
		fcl::DistanceRequest request;
		fcl::DistanceResult result;
		fcl::distance(parts_[i].get(), ob.get(), request, result);
		std::cout << parts_[i]->getAABB().min_ <<"\t;\t" << parts_[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		std::cout << "distance from " << i+1 << ": " << result.min_distance << std::endl;
	}

}

fcl::Transform3f robots::xARM6::KDL2fcl(const KDL::Frame &in)
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

KDL::Frame robots::xARM6::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Quaternion3f q = in.getQuatRotation();
    fcl::Vec3f t = in.getTranslation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());

    return f;
}

const std::vector<std::vector<float>> &robots::xARM6::getLimits() const
{
	return limits_;
}