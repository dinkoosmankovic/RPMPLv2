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

int PlannerInfo::getNumCollisionQueries() const
{
	return numCollisionQueries;
}

void PlannerInfo::setNumCollisionQueries(int numCollisionQueries)
{
	PlannerInfo::numCollisionQueries = numCollisionQueries;
}

int PlannerInfo::getNumDistanceQueries() const
{
	return numDistanceQueries;
}

void PlannerInfo::setNumDistanceQueries(int numDistanceQueries)
{
	PlannerInfo::numDistanceQueries = numDistanceQueries;
}

int PlannerInfo::getNumNodes() const
{
	return numNodes;
}

void PlannerInfo::setNumNodes(int numNodes)
{
	PlannerInfo::numNodes = numNodes;
}

int PlannerInfo::getNumIterations() const
{
	return numIterations;
}

void PlannerInfo::setNumIterations(int numIterations)
{
	PlannerInfo::numIterations = numIterations;
}

void PlannerInfo::addIterationTime(double time)
{
	iterationsTimesOfExecution.emplace_back(time);
}
