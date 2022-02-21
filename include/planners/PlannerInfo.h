//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<float> iterationsTimesOfExecution;
	float planningTime;
	size_t numCollisionQueries;
	size_t numDistanceQueries;
	size_t numNodes;
	size_t numIterations;
public:
	void addIterationTime(float time);
	const std::vector<float> &getIterationsTimesOfExecution() const;
	void setIterationsTimesOfExecution(const std::vector<float> &iterationsTimesOfExecution);
	float getPlanningTime() const;
	void setPlanningTime(float planningTime);
	size_t getNumCollisionQueries() const;
	void setNumCollisionQueries(size_t numCollisionQueries);
	size_t getNumDistanceQueries() const;
	void setNumDistanceQueries(size_t numDistanceQueries);
	size_t getNumNodes() const;
	void setNumNodes(size_t numNodes);
	size_t getNumIterations() const;
	void setNumIterations(size_t numIterations);

};

#endif //RPMPL_PLANNERINFO_H
