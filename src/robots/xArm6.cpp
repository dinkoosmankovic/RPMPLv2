//
// Created by dinko on 07.02.22.
//

#include <iostream>
#include <vector>

#include "xArm6.h"
#include "urdf/model.h"
#include "RealVectorSpaceState.h"

#include <glog/logging.h>
#include "stl_reader.h"

typedef std::shared_ptr <fcl::CollisionGeometryf> CollisionGeometryPtr;

robots::xARM6::~xARM6() {}

robots::xARM6::xARM6(std::string robot_desc)
{
    if (!kdl_parser::treeFromFile(robot_desc, robot_tree))
		throw std::runtime_error("Failed to construct kdl tree");

	urdf::Model model;
	if (!model.initFile(robot_desc))
    	throw std::runtime_error("Failed to parse urdf file");
	
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
		limits.emplace_back(std::vector<float>({lower, upper}));
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
			fcl::Vector3f p[3];
			fcl::BVHModel<fcl::OBBRSS<float>>* model = new fcl::BVHModel<fcl::OBBRSS<float>>;
    		model->beginModel();
			const auto mesh_ptr = dynamic_cast<const urdf::Mesh*>(links[i]->collision->geometry.get());			
			stl_reader::StlMesh <float, unsigned int> mesh (urdf_root_path + mesh_ptr->filename);
			for (int j = 0; j < mesh.num_tris(); ++j)
			{
				for (size_t icorner = 0; icorner < 3; ++icorner) 
				{
					const float* c = mesh.vrt_coords (mesh.tri_corner_ind (j, icorner));
					//LOG(INFO) << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
					p[icorner] = fcl::Vector3f(c[0], c[1], c[2]); 
				}
				//LOG(INFO) << "+++++++++++++++++++++++++++++";
				model->addTriangle(p[0], p[1], p[2]);
			}
			model->endModel();
			CollisionGeometryPtr fclBox(model);
			parts.emplace_back(new fcl::CollisionObject(fclBox, fcl::Transform3f()));
			init_poses.emplace_back(link_frame);
		}
	}
	//std::cout << "constructor----------------------\n";
	setState(std::make_shared<base::RealVectorSpaceState>(Eigen::VectorXf::Zero(6)));
}

std::shared_ptr<std::vector<KDL::Frame>> robots::xARM6::computeForwardKinematics(std::shared_ptr<base::State> q)
{
	//KDL::Chain robot_chain;
	KDL::TreeFkSolverPos_recursive treefksolver = KDL::TreeFkSolverPos_recursive(robot_tree);
	std::shared_ptr<std::vector<KDL::Frame>> framesFK = std::make_shared<std::vector<KDL::Frame>>();
	robot_tree.getChain("link_base", "link_eef", robot_chain);
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions() );

	for (size_t i = 0; i < q->getDimensions(); ++i)
		jointpositions(i) = q->getCoord()(i);

	for (size_t i = 0; i < robot_chain.getNrOfJoints(); ++i)
	{
		KDL::Frame cartpos;
		bool kinematics_status = treefksolver.JntToCart(jointpositions, cartpos, robot_chain.getSegment(i).getName());
		if (kinematics_status >= 0)
			framesFK->emplace_back(cartpos);		
	}
	return framesFK;
}

std::shared_ptr<Eigen::MatrixXf> robots::xARM6::computeXYZ(std::shared_ptr<base::State> q)
{
	std::shared_ptr<std::vector<KDL::Frame>> frames = computeForwardKinematics(q);
	std::shared_ptr<Eigen::MatrixXf> XYZ = std::make_shared<Eigen::MatrixXf>(3, getParts().size() + 1);
	XYZ->col(0) << 0, 0, 0;
	XYZ->col(1) << frames->at(1).p(0), frames->at(1).p(1), frames->at(1).p(2);
	XYZ->col(2) << frames->at(2).p(0), frames->at(2).p(1), frames->at(2).p(2);

	KDL::Vector i2(frames->at(2).M(0,0), frames->at(2).M(1,0), frames->at(2).M(2,0));
	KDL::Vector p3 = frames->at(2).p + i2 * 0.0775;
	XYZ->col(3) << p3(0), p3(1), p3(2);
	XYZ->col(4) << frames->at(4).p(0), frames->at(4).p(1), frames->at(4).p(2);

	KDL::Vector i4(frames->at(4).M(0,0), frames->at(4).M(1,0), frames->at(4).M(2,0));
	KDL::Vector p5 = frames->at(4).p + i4 * 0.076;
	XYZ->col(5) << p5(0), p5(1), p5(2);
	XYZ->col(6) << frames->at(5).p(0), frames->at(5).p(1), frames->at(5).p(2);

	return XYZ;
}

float robots::xARM6::computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
								 std::shared_ptr<Eigen::MatrixXf> XYZ)
{
	Eigen::VectorXf r(6);
	r(0) = getEnclosingRadius(XYZ, 2, -2);
	r(1) = getEnclosingRadius(XYZ, 2, -1);
	r(2) = getEnclosingRadius(XYZ, 3, -1);
	r(3) = getEnclosingRadius(XYZ, 5, 3);
	r(4) = getEnclosingRadius(XYZ, 5, -1);
	r(5) = 0;
	float d = r.dot((q1->getCoord() - q2->getCoord()).cwiseAbs());
	return fi / d;
}

float robots::xARM6::getEnclosingRadius(std::shared_ptr<Eigen::MatrixXf> XYZ, int j_start, int j_proj)
{
	float r = 0;
	if (j_proj == -2)	// Special case when all frame origins starting from j_start are projected on {x,y} plane
	{
		for (int j = j_start; j < XYZ->cols(); j++)
			r = std::max(r, XYZ->col(j).head(2).norm() + radii[j-1]);
	}
	else if (j_proj == -1) 	// No projection
	{
		for (int j = j_start; j < XYZ->cols(); j++)
			r = std::max(r, (XYZ->col(j) - XYZ->col(j_start-1)).norm() + radii[j-1]);
	}
	else	// Projection of all frame origins starting from j_start to the link (j_proj, j_proj+1) is needed
	{
		float t;
		Eigen::Vector3f A = XYZ->col(j_proj);
		Eigen::Vector3f B = XYZ->col(j_proj+1);
		Eigen::Vector3f P, P_proj;
		for (int j = j_start; j < XYZ->cols(); j++)
		{
			t = (XYZ->col(j) - A).dot(B - A) / (B - A).squaredNorm();
			P_proj = A + t * (B - A);
			r = std::max(r, (XYZ->col(j) - P_proj).norm() + radii[j-1]);
		}
	}
	return r;
}

void robots::xARM6::setState(std::shared_ptr<base::State> q_)
{
	q = q_;
	KDL::JntArray jointpositions = KDL::JntArray(q->getDimensions());

	std::shared_ptr<std::vector<KDL::Frame>> framesFK = computeForwardKinematics(q);
	//transform Collision geometries
	//std::cout << parts.size() << std::endl;
	KDL::Frame tf;
	for (size_t i = 0; i < parts.size(); ++i)
	{
		tf = framesFK->at(i) * init_poses[i];
		//std::cout << tf.p << "\n" << tf.M << "\n++++++++++++++++++++++++\n";

		//std::cout << "fcl\n";
		parts[i]->setTransform(KDL2fcl(tf));
		parts[i]->computeAABB(); 
		//std::cout << parts[i]->getAABB().min_ <<"\t;\t" << parts[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
	}    
}

fcl::Transform3f robots::xARM6::KDL2fcl(const KDL::Frame &in)
{
	fcl::Transform3f out;
    double x, y, z, w;
    in.M.GetQuaternion(x, y, z, w);
    fcl::Vector3f t(in.p[0], in.p[1], in.p[2]);
    fcl::Quaternionf q(w, x, y, z);
    out.linear() = q.matrix();
    out.translation() = t;
    return out;
}

KDL::Frame robots::xARM6::fcl2KDL(const fcl::Transform3f &in)
{
    fcl::Matrix3f R = in.rotation();
	fcl::Quaternionf q(R);
    fcl::Vector3f t = in.translation();

    KDL::Frame f;
    f.p = KDL::Vector(t[0],t[1],t[2]);
    f.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    return f;
}

void robots::xARM6::test()
{
	CollisionGeometryPtr fclBox(new fcl::Box<float>(0.5, 1.0, 3.0));
	fcl::Transform3f tf; tf.translation() = fcl::Vector3f(1, 1, 1.5);
	std::unique_ptr<fcl::CollisionObject<float>> ob(new fcl::CollisionObject<float>(fclBox, tf));
	for (size_t i = 0; i < parts.size(); ++i)
	{
		fcl::DistanceRequest<float> request;
		fcl::DistanceResult<float> result;
		fcl::distance(parts[i].get(), ob.get(), request, result);
		std::cout << parts[i]->getAABB().min_ <<"\t;\t" << parts[i]->getAABB().max_ << std::endl << "*******************" << std::endl;
		std::cout << "distance from " << i+1 << ": " << result.min_distance << std::endl;
	}
}