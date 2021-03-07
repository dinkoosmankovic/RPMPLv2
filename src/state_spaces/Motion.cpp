//
// Created by dinko on 7.3.21..
//

#include "Motion.h"

base::Motion::Motion(base::State *startState, base::State *endState) : startState(startState), endState(endState)
{
	endState->setParent(startState);
}
