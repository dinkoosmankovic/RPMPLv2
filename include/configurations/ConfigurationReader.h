//
// Created by dinko on 17.02.22.
//

#include <string>
#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"
#include "RRTConnectConfig.h"
#include "RBTConnectConfig.h"
#include "RGBTConnectConfig.h"
#include "RGBMTStarConfig.h"
#include "RealVectorSpaceConfig.h"

#include <glog/logging.h>

class ConfigurationReader
{
public:
    static void initConfiguration()
    {
        YAML::Node RealVectorSpaceConfigRoot = YAML::LoadFile("data/configurations/configuration_realvectorspace.yaml");
        YAML::Node RRTConnectConfigRoot = YAML::LoadFile("data/configurations/configuration_rrtconnect.yaml");
        YAML::Node RBTConnectConfigRoot = YAML::LoadFile("data/configurations/configuration_rbtconnect.yaml");
        YAML::Node RGBTConnectConfigRoot = YAML::LoadFile("data/configurations/configuration_rgbtconnect.yaml");
        YAML::Node RGBMTStarConfigRoot = YAML::LoadFile("data/configurations/configuration_rgbmtstar.yaml");

        if (RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].IsDefined())
            RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS = RealVectorSpaceConfigRoot["NUM_INTERPOLATION_VALIDITY_CHECKS"].as<int>();
        if (RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].IsDefined())
            RealVectorSpaceConfig::EQUALITY_THRESHOLD = RealVectorSpaceConfigRoot["EQUALITY_THRESHOLD"].as<float>();

        if (RRTConnectConfigRoot["MAX_ITER"].IsDefined())
            RRTConnectConfig::MAX_ITER = RRTConnectConfigRoot["MAX_ITER"].as<int>();
        if (RRTConnectConfigRoot["MAX_EXTENSION_STEPS"].IsDefined())
            RRTConnectConfig::MAX_EXTENSION_STEPS = RRTConnectConfigRoot["MAX_EXTENSION_STEPS"].as<int>();
        if (RRTConnectConfigRoot["EPS_STEP"].IsDefined())
            RRTConnectConfig::EPS_STEP = RRTConnectConfigRoot["EPS_STEP"].as<float>();
        if (RRTConnectConfigRoot["MAX_NUM_STATES"].IsDefined())
            RRTConnectConfig::MAX_NUM_STATES = RRTConnectConfigRoot["MAX_NUM_STATES"].as<int>();
        if (RRTConnectConfigRoot["MAX_PLANNING_TIME"].IsDefined())
            RRTConnectConfig::MAX_PLANNING_TIME = RRTConnectConfigRoot["MAX_PLANNING_TIME"].as<float>();

        if (RBTConnectConfigRoot["D_CRIT"].IsDefined())
            RBTConnectConfig::D_CRIT = RBTConnectConfigRoot["D_CRIT"].as<float>();
        if (RBTConnectConfigRoot["DELTA"].IsDefined())
            RBTConnectConfig::DELTA = RBTConnectConfigRoot["DELTA"].as<float>();
        if (RBTConnectConfigRoot["NUM_SPINES"].IsDefined())
            RBTConnectConfig::NUM_SPINES = RBTConnectConfigRoot["NUM_SPINES"].as<int>();

        if (RGBTConnectConfigRoot["NUM_LAYERS"].IsDefined())
            RGBTConnectConfig::NUM_LAYERS = RGBTConnectConfigRoot["NUM_LAYERS"].as<int>();
            
        if (RGBMTStarConfigRoot["RETURN_WHEN_PATH_IS_FOUND"].IsDefined())
            RGBMTStarConfig::RETURN_WHEN_PATH_IS_FOUND = RGBMTStarConfigRoot["RETURN_WHEN_PATH_IS_FOUND"].as<bool>();

        LOG(INFO) << "Configuration parameters read successfully!";
        
    }
};

