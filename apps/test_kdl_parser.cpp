//
// Created by dinko on 6.2.22.
// simple kdl_parser test
//

#include <nanoflann.hpp>

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <glog/logging.h>

bool parse_file(std::string filename)
{
	KDL::Tree robot_tree;
	if (!kdl_parser::treeFromFile(filename, robot_tree))
	{
		LOG(ERROR) << "Failed to construct kdl tree";
		return false;
	}
	LOG(INFO) << "Number of joints: " << robot_tree.getNrOfJoints();
	LOG(INFO) << "Number of segments: " << robot_tree.getNrOfSegments();
	KDL::Chain robot_chain;
	robot_tree.getChain("base_link", "link2", robot_chain);
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot_chain);

	KDL::JntArray jointpositions = KDL::JntArray(2);
	jointpositions(0) = 0.1;
	jointpositions(1) = -0.1;
	KDL::Frame cartpos; 

	bool kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
	if(kinematics_status >= 0)
	{
        LOG(INFO) << "x: " << cartpos.p.x() << ";" << "y: " << cartpos.p.y() << ";" << "z: " << cartpos.p.z() ;
    }
	else
	{
        LOG(INFO) << "Error: could not calculate forward kinematics!";
    }

	return true;
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;

	bool parsed = parse_file("/home/dinko/RPMPLv2/data/planar_2dof/planar_2dof.urdf");

	return 0;
}
