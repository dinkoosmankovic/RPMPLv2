//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"
#include <vector>
#include <memory>

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
			bool solve() override;
			base::Tree getStartTree() const;
			base::Tree getGoalTree() const;
			Status extend(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q_rand);
			Status connect(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q_rand);
			std::shared_ptr<base::State> get_q_near(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q);
			void addNode(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q);
			const std::vector<std::shared_ptr<base::State>> &getPath() const;
			void outputPlannerData(std::string filename) const override;
			bool isTerminationConditionSatisfied() const override;
		private:
			std::shared_ptr<base::StateSpace> ss;
			std::shared_ptr<base::State> start;
			std::shared_ptr<base::State> goal;
			base::Tree startTree;
			base::Tree goalTree;
			void initPlanner();
			void prepareKdTrees();
			void computePath();
			std::shared_ptr<base::KdTree> startKdTree;
			std::shared_ptr<base::KdTree> goalKdTree;
			//TODO: Read from configuration file
			double step = 0.2;
			std::vector<std::shared_ptr<base::State>> path;
		};
	}
}
#endif //RPMPL_RRTCONNECT_H
