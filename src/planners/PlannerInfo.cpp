//
// Created by dinko on 26.5.21..
//

#include <PlannerInfo.h>
#include "PlannerInfo.h"

const std::vector<double> &PlannerInfo::getIterationsTimesOfExecution() const
{
	return iterationsTimesOfExecution;
}

void PlannerInfo::setIterationsTimesOfExecution(const std::vector<double> &iterationsTimesOfExecution)
{
	PlannerInfo::iterationsTimesOfExecution = iterationsTimesOfExecution;
}

double PlannerInfo::getPlanningTime() const
{
	return planningTime;
}

void PlannerInfo::setPlanningTime(double planningTime)
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

size_t PlannerInfo::getNumNodes() const
{
	return numNodes;
}

void PlannerInfo::setNumNodes(size_t numNodes)
{
	PlannerInfo::numNodes = numNodes;
}

size_t PlannerInfo::getNumIterations() const
{
	return numIterations;
}

void PlannerInfo::setNumIterations(size_t numIterations)
{
	PlannerInfo::numIterations = numIterations;
}

void PlannerInfo::addIterationTime(double time)
{
	iterationsTimesOfExecution.emplace_back(time);
}
