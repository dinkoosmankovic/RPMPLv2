//
// Created by dinko on 17.02.22.
//

#include <string>
#include <yaml-cpp/yaml.h>
#include "yaml-cpp/parser.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"
#include "RRTConnectConfig.h"
#include "RealVectorSpaceConfig.h"

#include <glog/logging.h>

class ConfigurationReader
{
public:
    static void initConfiguration()
    {
        YAML::Node RealVectorSpaceConfigRoot = YAML::LoadFile("data/configurations/configuration_realvectorspace.yaml");
        YAML::Node RRTConnectConfigRoot = YAML::LoadFile("data/configurations/configuration_rrtconnect.yaml");

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

        LOG(INFO) << "Configuration parameters read successfully!";
        
    }
};

