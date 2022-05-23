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
            class HorizonState
            {
            public:
                enum Status {good, bad, critical, goal};

            private:
                std::shared_ptr<base::State> state;
                std::shared_ptr<base::State> state_reached = nullptr;       // Reached state when generating spine from 'q_current' towards 'state'
                HorizonState::Status status = HorizonState::Status::good;   // Status of 'state_reached': 'good', 'bad', 'critical' or 'goal'
                int idx_path;                                               // Index of 'state' in the predefined path. It is -1, if 'state' does not belong to the path
                float d_c = -1;                                             // Underestimation of distance-to-obstacles for 'state_reached'
                float d_c_previous = -1;                                    // 'd_c' from previous iteration
                float weight = -1;                                          // Weight in range [0, 1] for 'state_reached'
            
            public:
                HorizonState(std::shared_ptr<base::State> state_, bool idx_path_)
                {
                    state = state_;
                    idx_path = idx_path_;
                }
                HorizonState() {}
                ~HorizonState() {}

                inline std::shared_ptr<base::State> getState() const { return state; }
                inline std::shared_ptr<base::State> getStateReached() const { return state_reached; }
                inline HorizonState::Status getStatus() const { return status; }
                inline int getIndexInPath() const { return idx_path; }
                inline float getDistance() const { return d_c; }
                inline float getDistancePrevious() const { return d_c_previous; }
                inline float getWeight() const { return weight; }
		        inline const Eigen::VectorXf &getCoord() const { return state->getCoord(); }
		        inline float getCoord(int idx) const { return state->getCoord(idx); }

                inline void setStateReached(std::shared_ptr<base::State> state_reached_) { state_reached = state_reached_; }
                inline void setStatus(HorizonState::Status status_) { status = status_; }
                inline void setIndexInPath(int idx_path_) { idx_path = idx_path_; }
                inline void setDistance(float d_c_) { d_c = d_c_; }
                inline void setDistancePrevious(float d_c_previous_) { d_c_previous = d_c_previous_; }
                inline void setWeight(float weight_) { weight = weight_; }

            };

            std::vector<std::shared_ptr<HorizonState>> horizon;
            std::shared_ptr<base::State> q_current;                 // Current robot configuration
            std::shared_ptr<HorizonState> q_next;                   // Next robot configuration
            std::shared_ptr<HorizonState> q_next_previous;          // Next robot configuration from the previous iteration
            float d_max_mean = 0;                                   // Averaged maximal distance-to-obstacles through iterations
                
            void computeHorizon();
            void shortenHorizon(int num);
            void addRandomStates(int num);
            void addLateralStates(int num);
            void modifyState(std::shared_ptr<HorizonState> &q);
            void computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q);
            void computeNextState();
            bool whetherToReplan();

        };
    }
}

#endif RPMPL_DRGBTCONNECT_H