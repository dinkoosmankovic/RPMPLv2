//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"

namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF();
		~Planar2DOF();
		void computeForwardKinematics(std::shared_ptr<base::State> q) override;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
