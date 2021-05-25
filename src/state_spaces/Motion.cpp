//
// Created by dinko on 7.3.21..
//

#include "Motion.h"

base::Motion::Motion(std::shared_ptr<State> startState, std::shared_ptr<State> endState) : startState(startState), endState(endState)
{
	endState->setParent(startState);
}
