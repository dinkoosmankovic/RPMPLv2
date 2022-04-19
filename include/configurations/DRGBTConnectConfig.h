//
// Created by nermin on 14.04.22.
//

class DRGBTConnectConfig
{
public:
    static int INIT_HORIZON_SIZE;       // Initial horizon size
    static float WEIGHT_MIN;            // Treshold 1 for the replanning assessment. Range: between 0 and 1
    static float WEIGHT_MEAN_MIN;       // Treshold 2 for the replanning assessment. Range: between 0 and 1
    static float MAX_ITER_TIME;         // Maximal runtime of a single iteration in [ms] (also 1/f_s, where f_s is the number of frames per second)
};