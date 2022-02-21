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
            std::vector<size_t> numNodes;               // Total number of nodes for each tree
            float costOpt = INFINITY;                   // The cost of the final path
            bool returnWPF = false;                     // Whether to return When Path is Found (default: false)
            
			void initPlanner();
            std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> 
                connectGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            float getCostToCome(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2);
            bool mainTreesReached(std::vector<int> &treesReached);
            std::shared_ptr<base::State> optimize(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree, 
                                                  std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_reached);
        };
	}
}
#endif //RPMPL_RGBMTSTAR_H