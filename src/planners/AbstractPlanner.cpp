//
// Created by dinko on 16.3.21..
//

#include "AbstractPlanner.h"

planning::AbstractPlanner::~AbstractPlanner() {}

std::shared_ptr<base::StateSpace> planning::AbstractPlanner::getSS() const
{
	return ss;
}

std::shared_ptr<PlannerInfo> planning::AbstractPlanner::getPlannerInfo() const
{
	return plannerInfo;
}
