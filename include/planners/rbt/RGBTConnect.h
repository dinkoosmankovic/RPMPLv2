//
// Created by nermin on 19.02.22.
//
#ifndef RPMPL_RGBTCONNECT_H
#define RPMPL_RGBTCONNECT_H

#include "RBTConnect.h"
#include "AbstractRobot.h"
#include <vector>
#include <memory>

namespace planning
{
	namespace rbt
	{
		class RGBTConnect : public planning::rbt::RBTConnect
		{
		public:
			RGBTConnect(std::shared_ptr<base::StateSpace> ss_);
			RGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);
			~RGBTConnect();
			virtual bool solve() override;
			virtual void outputPlannerData(std::string filename) const override;

		protected:
			//TODO: Read from configuration file
            uint numLayers = 5;       // Number of layers (extensions) for building a generalized bur

            std::tuple<planning::rrt::Status, std::shared_ptr<std::vector<std::shared_ptr<base::State>>>> 
                extendGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            planning::rrt::Status connectGenSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
                                                  std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e);
            float getDistanceUnderestimation(std::shared_ptr<base::State> q, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes);
            // float getDistance(std::shared_ptr<base::State> q) override;
        };
	}
}
#endif //RPMPL_RGBTCONNECT_H
