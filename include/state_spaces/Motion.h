//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_MOTION_H
#define RPMPL_MOTION_H

#include "State.h"

namespace base
{
	class Motion
	{
	public:
		Motion(std::shared_ptr<State> startState, std::shared_ptr<State> endState);
	private:
		std::shared_ptr<State> startState;
		std::shared_ptr<State> endState;
	};
}

#endif //RPMPL_MOTION_H
