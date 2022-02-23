//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<double> iterationsTimesOfExecution;
	double planningTime;
	int numCollisionQueries;
	int numDistanceQueries;
	int numNodes;
	int numIterations;
public:
	void clearPlannerInfo();
	void addIterationTime(double time);
	const std::vector<double> &getIterationsTimesOfExecution() const;
	void setIterationsTimesOfExecution(const std::vector<double> &iterationsTimesOfExecution);
	double getPlanningTime() const;
	void setPlanningTime(double planningTime);
	int getNumCollisionQueries() const;
	void setNumCollisionQueries(int numCollisionQueries);
	int getNumDistanceQueries() const;
	void setNumDistanceQueries(int numDistanceQueries);
	int getNumNodes() const;
	void setNumNodes(int numNodes);
	int getNumIterations() const;
	void setNumIterations(int numIterations);

};

#endif //RPMPL_PLANNERINFO_H
