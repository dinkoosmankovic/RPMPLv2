//
// Created by nermin on 13.04.22.
//
#ifndef RPMPL_DRGBTCONNECT_H
#define RPMPL_DRGBTCONNECT_H

#include "RGBTConnect.h"

namespace planning
{
    namespace rbt
    {
        class DRGBTConnect : public planning::rbt::RGBTConnect
        {
        public:
            DRGBTConnect(std::shared_ptr<base::StateSpace> ss_);
			DRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~DRGBTConnect();
            bool solve() override;
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;

		protected:
            class Horizon
            {
            public:
                inline static uint size = 0;                                                        // Horizon size that may change during the algorithm execution
                inline static std::vector<std::shared_ptr<base::State>> states = {};                // All states from the horizon
                inline static std::vector<std::shared_ptr<base::State>> states_good = {};           // Good states from the horizon
                inline static std::vector<std::shared_ptr<base::State>> states_bad = {};            // Bad states from the horizon
                inline static std::vector<std::shared_ptr<base::State>> states_reached = {};        // Reached states from the horizon
                inline static std::vector<std::shared_ptr<base::State>> states_not_in_path = {};    // Horizon states that are not in the predefined path
                inline static std::shared_ptr<base::State> q_current = nullptr;                     // Current robot configuration
                inline static std::shared_ptr<base::State> q_next = nullptr;                        // Next robot configuration
                inline static std::vector<float> d_c = {};                                          // Underestimation of distances-to-obstacles for each state from 'states'
                inline static std::vector<float> d_c_previous = {};                                 // 'd_c' from previous iteration
                inline static std::vector<float> weights = {};                                      // State weights
                inline static float d_max_mean = -1;                                                // Averaged maximal distance-to-obstacles through iterations
                inline static int idx_next = -1;                                                    // Index of next node from the horizon
                inline static int idx_previous = -1;                                                // Index of previous node from the horizon
            };
            
            void updateHorizon();
            void addRandomSpines(int num);
            void addLateralSpines(int num);
            void addWeightedSpines(int num, std::vector<std::shared_ptr<base::State>> &Q, bool orientation);
            std::shared_ptr<base::State> getNextState(int idx_previous = -1);
            bool whetherToReplan();

        };
    }
}

#endif RPMPL_DRGBTCONNECT_H