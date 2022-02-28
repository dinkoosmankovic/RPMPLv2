//
// Created by dinko on 26.5.21..
//

#include <PlannerInfo.h>
#include "PlannerInfo.h"

const std::vector<float> &PlannerInfo::getIterationsTimesOfExecution() const
{
	return iterationsTimesOfExecution;
}

void PlannerInfo::setIterationsTimesOfExecution(const std::vector<float> &iterationsTimesOfExecution)
{
	PlannerInfo::iterationsTimesOfExecution = iterationsTimesOfExecution;
}

float PlannerInfo::getPlanningTime() const
{
	return planningTime;
}

void PlannerInfo::setPlanningTime(float planningTime)
{
	PlannerInfo::planningTime = planningTime;
}

size_t PlannerInfo::getNumCollisionQueries() const
{
	return numCollisionQueries;
}

void PlannerInfo::setNumCollisionQueries(size_t numCollisionQueries)
{
	PlannerInfo::numCollisionQueries = numCollisionQueries;
}

size_t PlannerInfo::getNumDistanceQueries() const
{
	return numDistanceQueries;
}

void PlannerInfo::setNumDistanceQueries(size_t numDistanceQueries)
{
	PlannerInfo::numDistanceQueries = numDistanceQueries;
}

size_t PlannerInfo::getNumStates() const
{
	return numStates;
}

void PlannerInfo::setNumStates(size_t numStates)
{
	PlannerInfo::numStates = numStates;
}

size_t PlannerInfo::getNumIterations() const
{
	return numIterations;
}

void PlannerInfo::setNumIterations(size_t numIterations)
{
	PlannerInfo::numIterations = numIterations;
}

void PlannerInfo::addIterationTime(float time)
{
	iterationsTimesOfExecution.emplace_back(time);
}

const std::vector<float> &PlannerInfo::getCostConvergence() const
{
	return statesCosts;
}

void PlannerInfo::addCostConvergence(const std::vector<float> &statesCosts)
{
	for (int i = 0; i < statesCosts.size(); i++)
	{
		PlannerInfo::statesCosts.emplace_back(statesCosts[i]);
	}
}