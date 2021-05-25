//
// Created by dinko on 7.3.21..
//

#include "State.h"

base::State::~State()
{}

base::State *base::State::getParent() const
{
	return parent;
}

void base::State::setParent(base::State *parent_)
{
	parent = parent_;
}

StateSpaceType base::State::getStateSpaceType() const
{
	return stateSpaceType;
}

void base::State::setStateSpaceType(StateSpaceType stateSpaceType)
{
	State::stateSpaceType = stateSpaceType;
}

std::ostream &base::operator<<(std::ostream &os, const base::State* state)
{
	if (state->getParent() == nullptr)
		os << "point: (" << state->getCoord().transpose() << "); parent: NONE" << std::endl;
	else
		os << "point: (" << state->getCoord().transpose() << "); parent: (" <<
		   state->getParent()->getCoord().transpose() << ")" << std::endl;
	return os;
}
