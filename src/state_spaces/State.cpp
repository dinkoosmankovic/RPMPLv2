//
// Created by dinko on 7.3.21..
//

#include "State.h"

base::State::~State()
{}

std::shared_ptr<base::State> base::State::getParent() const
{
	return parent;
}

void base::State::setParent(std::shared_ptr<base::State> parent_)
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
		os << "q: (" << state->getCoord().transpose() << "); parent q: NONE";
	else
		os << "q: (" << state->getCoord().transpose() << "); parent q: (" <<
		   state->getParent()->getCoord().transpose() << ")";
	return os;
}
