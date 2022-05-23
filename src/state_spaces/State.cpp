//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "State.h"

base::State::~State() {}

void base::State::addChild(std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

std::ostream &base::operator<<(std::ostream &os, const base::State *state)
{
	if (state->getParent() == nullptr)
		os << "q: (" << state->getCoord().transpose() << ");\t parent q: NONE";
	else
		os << "q: (" << state->getCoord().transpose() << ");\t parent q: (" << state->getParent()->getCoord().transpose() << ")";
	return os;
}