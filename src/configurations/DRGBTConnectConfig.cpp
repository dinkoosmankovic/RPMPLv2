//
// Created by nermin on 14.04.22.
//

#include "DRGBTConnectConfig.h"

unsigned long DRGBTConnectConfig::MAX_NUM_ITER  = 10000;
float DRGBTConnectConfig::MAX_ITER_TIME         = 100;
float DRGBTConnectConfig::MAX_PLANNING_TIME     = 10e3;
int DRGBTConnectConfig::INIT_HORIZON_SIZE       = 10;
float DRGBTConnectConfig::STEP                  = 0.1;
float DRGBTConnectConfig::WEIGHT_MIN            = 0.5;
float DRGBTConnectConfig::WEIGHT_MEAN_MIN       = 0.5;