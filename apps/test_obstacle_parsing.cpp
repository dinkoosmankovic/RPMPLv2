#include <iostream>
#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

#include <glog/logging.h>

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	FLAGS_logtostderr = true;

	YAML::Node config = YAML::LoadFile("data/planar_2dof/obstacles_easy.yaml");
	for (size_t i = 0; i < config["obstacles"].size(); ++i)
	{
		LOG(INFO) << config["obstacles"][i];
		LOG(INFO) << config["obstacles"][i]["box"].IsDefined(); // returns 1
		LOG(INFO) << config["obstacles"][i]["mesh"].IsDefined(); // return 0
	}

	return 0;
}
