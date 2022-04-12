//
// Created by dinko on 16.3.21.
// Modified by nermin on 18.02.22.
//
#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"
#include "Tree.h"
#include <vector>
#include <memory>
#include <chrono>

namespace planning
{
	namespace rrt
	{
		enum Status {Advanced, Trapped, Reached};
		class RRTConnect : public AbstractPlanner
		{
		public:
			RRTConnect(std::shared_ptr<base::StateSpace> ss_);
			RRTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~RRTConnect();
			virtual bool solve() override;
			base::Tree getTree(int tree_idx) const;
			const std::vector<std::shared_ptr<base::State>> &getPath() const override;
			bool checkStoppingCondition(planning::rrt::Status status, std::chrono::steady_clock::time_point &time_start);
			virtual void outputPlannerData(std::string filename, bool output_states_and_paths = true, bool append_output = false) const override;
			void clearPlanner();

		protected:
			std::shared_ptr<base::StateSpace> ss;
			std::shared_ptr<base::State> start;
			std::shared_ptr<base::State> goal;
			std::vector<std::shared_ptr<base::Tree>> trees;
			std::vector<std::shared_ptr<base::State>> path;
			
			void initPlanner();
			std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> extend(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			planning::rrt::Status connect(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			void computePath(std::shared_ptr<base::State> q_con0 = nullptr, std::shared_ptr<base::State> q_con1 = nullptr);
			float getElapsedTime(std::chrono::steady_clock::time_point &time_start);
		};
	}
}
#endif //RPMPL_RRTCONNECT_H
