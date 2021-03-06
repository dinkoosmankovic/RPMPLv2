//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_ABSTRACTROBOT_H
#define RPMPL_ABSTRACTROBOT_H

#include <vector> 
#include <memory>
#include "State.h"
#include "fcl/fcl.h"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

namespace robots
{
	class AbstractRobot
	{
	protected:
		std::shared_ptr<base::State> q;
		
	public:
		AbstractRobot() { q = nullptr; }
		virtual ~AbstractRobot() = 0;
								  
		virtual const std::vector<std::unique_ptr<fcl::CollisionObject<float>>> &getParts() const = 0;
		virtual const std::vector<std::vector<float>> &getLimits() const = 0;
		virtual const int getDimensions() = 0;
		virtual const float getRadius(int dim) = 0;

		virtual void setState(std::shared_ptr<base::State> q_) = 0;

		virtual std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<Eigen::MatrixXf> computeXYZ(std::shared_ptr<base::State> q) = 0;
		virtual float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
								  std::shared_ptr<Eigen::MatrixXf> XYZ) = 0;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
