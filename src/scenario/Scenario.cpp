//
// Created by dinko on 17.02.22.
//

#include "Scenario.h"
#include "Environment.h"
#include "AbstractRobot.h"
#include "RealVectorSpaceFCL.h"
#include "xArm6.h"
#include "Planar2DOF.h"
#include "RealVectorSpaceState.h"

#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"
#include <stdexcept>

#include <glog/logging.h>

scenario::Scenario::Scenario(std::string configuration_file)
{
    YAML::Node node = YAML::LoadFile(configuration_file);
    std::vector<env::Obstacle> obstacles(node["obstacles"].size());

    for (size_t i = 0; i < node["obstacles"].size(); ++i)
	{
        YAML::Node obstacle = node["obstacles"][i];
        if(obstacle["box"].IsDefined())
        {
            // BOX
            YAML::Node box_size = obstacle["box"]["dim"];
            float bx = box_size[0].as<float>();
            float by = box_size[1].as<float>();
            float bz = box_size[2].as<float>();
            fcl::Box obs(bx, by, bz);
            
            YAML::Node trans = obstacle["box"]["trans"];
            float tx = trans[0].as<float>();
            float ty = trans[1].as<float>();
            float tz = trans[2].as<float>();

            LOG(INFO) << "read trans";

            YAML::Node rot = obstacle["box"]["rot"];
            float rx = rot[1].as<float>();
            float ry = rot[2].as<float>();
            float rz = rot[3].as<float>();
            float rw = rot[0].as<float>();
            
            fcl::Vec3f tr(tx, ty, tz);
            fcl::Quaternion3f quat(rw, rx, ry, rz);

            fcl::Transform3f tf(quat, tr);

            LOG(INFO) << "Object tf: " << tf.getTranslation() << "\n" << tf.getRotation() << "\n------------";
            
            env::Obstacle obs_(std::make_pair(obs, tf));
            obstacles[i] = obs_;
        }
    }

    env = std::make_shared<env::Environment>(obstacles);

    YAML::Node robot_node = node["robot"];
    const int dim = robot_node["dimensions"].as<int>();

    std::string type = robot_node["type"].as<std::string>();
    std::string space_state = robot_node["space"].as<std::string>();
    if (type == "xARM6")
        robot = std::make_shared<robots::xARM6>( robot_node["urdf"].as<std::string>());

    if (type == "planar_2DOF")
        robot = std::make_shared<robots::Planar2DOF>( robot_node["urdf"].as<std::string>());

    if (space_state == "RealVectorSpaceFCL")
    {
        ss = std::make_shared<base::RealVectorSpaceFCL>(dim, robot, env);
        YAML::Node start_node = robot_node["start"];
        YAML::Node goal_node = robot_node["goal"];
        std::vector<float> start_vec;
        std::vector<float> goal_vec;
        if (start_node.size() != goal_node.size())
            throw std::logic_error("Start and goal size mismatch!");
        for (size_t i = 0; i < start_node.size(); ++i)
        {
            start_vec.emplace_back(start_node[i].as<float>());
            goal_vec.emplace_back(goal_node[i].as<float>());
        }
        start = std::make_shared<base::RealVectorSpaceState>(vector2VectorXf(start_vec));
        goal = std::make_shared<base::RealVectorSpaceState>(vector2VectorXf(goal_vec));
    }

}

const std::shared_ptr<robots::AbstractRobot>& scenario::Scenario::getRobot() const 
{
    return robot;
}		

const std::shared_ptr<env::Environment>& scenario::Scenario::getEnvironment() const
{
    return env;
}

const std::shared_ptr<base::StateSpace>& scenario::Scenario::getStateSpace() const
{
    return ss;
}

Eigen::VectorXf scenario::Scenario::vector2VectorXf(std::vector<float>& v) 
{
    Eigen::VectorXf vec = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v.data(), v.size());
    return vec;
}
