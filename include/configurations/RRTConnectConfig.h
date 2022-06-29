//
// Created by dinko on 17.02.22.
//

class RRTConnectConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static unsigned long MAX_NUM_STATES;        // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [ms]
    static int MAX_EXTENSION_STEPS;             // Maximal number of extensions in connect procedure
    static float EPS_STEP;                      // Step in C-space used by RRT-based algorithms

};