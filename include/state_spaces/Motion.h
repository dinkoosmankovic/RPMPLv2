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
		Motion(base::State *startState, base::State *endState);
	public:
	private:
		base::State *startState;
		base::State *endState;
	protected:
	};
}

#endif //RPMPL_MOTION_H
