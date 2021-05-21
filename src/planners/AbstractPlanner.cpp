//
// Created by dinko on 16.3.21..
//

#include "AbstractPlanner.h"

planning::AbstractPlanner::~AbstractPlanner() {}

base::StateSpace *planning::AbstractPlanner::getSs() const
{
	return ss;
}
