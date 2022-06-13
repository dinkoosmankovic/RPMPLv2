//
// Created by nermin on 28.02.22.
//

class RBTConnectConfig
{
public:
    static unsigned long MAX_NUM_ITER;          // Maximal number of algorithm iterations
    static unsigned long MAX_NUM_STATES;        // Maximal number of considered states
    static float MAX_PLANNING_TIME;             // Maximal algorithm runtime in [ms]
    static float D_CRIT;                        // Critical distance in W-space when RBT uses RRT-mode
    static float DELTA;    	                    // Radius of hypersphere from q to q_e
    static int NUM_SPINES;                      // Number of bur spines

};