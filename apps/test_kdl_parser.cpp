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
	return true;
}

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;

	bool parsed = parse_file("/home/dinko/RPMPLv2/data/planar_2dof/planar_2dof.urdf");

	return 0;
}
