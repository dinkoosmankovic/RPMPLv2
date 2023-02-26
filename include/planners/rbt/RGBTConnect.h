//
// Created by nermin on 19.02.22.
//
#ifndef RPMPL_RGBTCONNECT_H
#define RPMPL_RGBTCONNECT_H

#include "RBTConnect.h"

namespace planning
{
	namespace rbt
	{
		class RGBTConnect : public planning::rbt::RBTConnect
		{
		public:
			RGBTConnect() {}
			RGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			bool solve() override;
			bool checkTerminatingCondition(base::State::Status status) override;
			void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;

		protected:
            std::tuple<base::State::Status, std::shared_ptr<std::vector<std::shared_ptr<base::State>>>> extendGenSpine
        		(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			std::tuple<base::State::Status, std::shared_ptr<base::State>> extendGenSpineV2
				(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            base::State::Status connectGenSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            float computeDistance(std::shared_ptr<base::State> q) override;
            float computeDistanceUnderestimation(std::shared_ptr<base::State> q, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes);
        };
	}
}
#endif //RPMPL_RGBTCONNECT_H
