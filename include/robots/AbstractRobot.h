//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_ABSTRACTROBOT_H
#define RPMPL_ABSTRACTROBOT_H

#include <vector> 
#include <memory>
#include "State.h"

#include <fcl/broadphase/broadphase.h>

namespace robots
{
	class AbstractRobot
	{
	public:
		AbstractRobot() { q == nullptr; };
		virtual ~AbstractRobot() = 0;
		//virtual void computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		virtual void setState(std::shared_ptr<base::State> q_) = 0;
		virtual const std::vector<std::unique_ptr<fcl::CollisionObject> >& getParts() const = 0;
	protected:
		std::shared_ptr<base::State> q;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
