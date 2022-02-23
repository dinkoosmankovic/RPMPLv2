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
			//TODO: Read from configuration file
			float d_crit = 0.01;      		// Critical distance in W-space when RBT becomes RRT
			float delta = M_PI;    			// Radius of hypersphere from q to q_e
			uint numSpines = 7;       		// Number of bur spines

			virtual float getDistance(std::shared_ptr<base::State> q);
			void saturateSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			void pruneSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
			std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> extendSpine
				(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e, float d_c_underest = -1);
			planning::rrt::Status connectSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
											   std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
											   
		private:
			float computeStep(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e, float fi, std::vector<KDL::Frame> &frames);
		};
	}
}
#endif //RPMPL_RBTCONNECT_H
