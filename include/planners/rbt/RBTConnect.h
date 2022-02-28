//
// Created by nermin on 18.02.22.
//
#ifndef RPMPL_RBTCONNECT_H
#define RPMPL_RBTCONNECT_H

#include "RRTConnect.h"
#include "AbstractRobot.h"
#include <vector>
#include <memory>

namespace planning
{
	namespace rbt
	{
		class RBTConnect : public planning::rrt::RRTConnect
		{
		public:
			RBTConnect(std::shared_ptr<base::StateSpace> ss_);
			RBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~RBTConnect();
			virtual bool solve() override;
			virtual void outputPlannerData(std::string filename) const override;

		protected:
			float getDistance(std::shared_ptr<base::State> q);
			void saturateSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			void pruneSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> extendSpine
				(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e, float d_c_underest = -1);
			planning::rrt::Status connectSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
											   std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
											   
		private:
			float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, std::vector<KDL::Frame> &frames);
            float getEnclosingRadius(std::vector<KDL::Frame> &frames, int j_start, int j_proj);
		};
	}
}
#endif //RPMPL_RBTCONNECT_H
