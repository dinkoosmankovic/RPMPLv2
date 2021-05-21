//
// Created by dinko on 7.3.21..
//

#include "StateSpace.h"
#include "StateSpaceType.h"

base::StateSpace::~StateSpace()
{}

StateSpaceType base::StateSpace::getStateSpaceType() const
{
	return stateSpaceType;
}

void base::StateSpace::setStateSpaceType(StateSpaceType stateSpaceType)
{
	StateSpace::stateSpaceType = stateSpaceType;
}
