//
// Created by dinko on 17.02.22.
//

class RRTConnectConfig
{
    public:
        static int MAX_ITER;                // Maximal algorithm iterations
        static int MAX_EXTENSION_STEPS;     // Maximal number of extensions in connect procedure
        static float EPS_STEP;              // Step in C-space used by RRT-based algorithms
        static int MAX_NUM_STATES;          // Max. number of considered states
        static float MAX_PLANNING_TIME;     // Maximal algorithm runtime in [ms]

};