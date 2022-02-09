//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_ABSTRACTROBOT_H
#define RPMPL_ABSTRACTROBOT_H

#include <vector> 
#include <memory>
#include "State.h"

namespace robots
{
	class AbstractRobot
	{
	public:
		explicit AbstractRobot() { q == nullptr; };
		virtual ~AbstractRobot() = 0;
		//virtual void computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		virtual void setState(std::shared_ptr<base::State> q_) = 0;
	protected:
		std::shared_ptr<base::State> q;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
