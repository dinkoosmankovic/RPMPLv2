//
// Created by nermin on 20.02.22.
//
#ifndef RPMPL_RGBMTSTAR_H
#define RPMPL_RGBMTSTAR_H

#include "RGBTConnect.h"
#include <random>

namespace planning
{
	namespace rbt
	{
		class RGBMTStar : public planning::rbt::RGBTConnect
		{
		public:
			RGBMTStar(std::shared_ptr<base::StateSpace> ss_);
			RGBMTStar(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~RGBMTStar();
			bool solve() override;
			void outputPlannerData(std::string filename) const override;

		protected:
			//TODO: Read from configuration file
            std::vector<size_t> numStates;              // Total number of states/nodes for each tree
            float costOpt = INFINITY;                   // The cost of the final path
            bool returnWPF = false;                     // Whether to return When Path is Found (default: false)
            
			void initPlanner();
            std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> 
                connectGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            float getCostToCome(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2);
            bool mainTreesReached(std::vector<int> &treesReached);
            std::shared_ptr<base::State> optimize(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree, 
                                                  std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_reached);
            void unifyTrees(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::Tree> tree0, std::shared_ptr<KdTree> kdtree0,
                            std::shared_ptr<base::State> q_con, std::shared_ptr<base::State> q0_con);
            void deleteTrees(std::vector<std::shared_ptr<base::Tree>> &trees, std::vector<int> &treesConnected);
            bool checkStoppingCondition(std::shared_ptr<base::State> q_con0, std::shared_ptr<base::State> q_con1, 
                                        std::chrono::steady_clock::time_point &time_start);

        private:
            void considerChildren(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree0, std::shared_ptr<KdTree> kdtree0,
                                  std::shared_ptr<base::State> q0_con, std::shared_ptr<base::State> q_considered);
        };
	}
}
#endif //RPMPL_RGBMTSTAR_H