//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<float> iterations_times_of_execution;
	float planning_time;
	size_t num_collision_queries;
	size_t num_distance_queries;
	size_t num_states;
	size_t num_iterations;
	std::vector<float> cost_convergence;		// Cost convergence rate (cost-state curve)
	bool success_state = false;					// Did planner succeed to find a solution?

public:
	void setIterationsTimesOfExecution(const std::vector<float> &iterations_times_of_execution_) 
		{ iterations_times_of_execution = iterations_times_of_execution_; }
	void setPlanningTime(float planning_time_) { planning_time = planning_time_; }
	void setNumCollisionQueries(size_t num_collision_queries_) { num_collision_queries = num_collision_queries_; }
	void setNumDistanceQueries(size_t num_distance_queries_) { num_distance_queries = num_distance_queries_; }
	void setNumStates(size_t num_states_) { num_states = num_states_; }
	void setNumIterations(size_t num_iterations_) { num_iterations = num_iterations_; }
	void setSuccessState(bool success_state_) { success_state = success_state_; }

	const std::vector<float> &getIterationsTimesOfExecution() const { return iterations_times_of_execution; }
	float getPlanningTime() const { return planning_time; }
	size_t getNumCollisionQueries() const { return num_collision_queries; }
	size_t getNumDistanceQueries() const { return num_distance_queries; }
	size_t getNumStates() const { return num_states; }
	size_t getNumIterations() const { return num_iterations; }
	const std::vector<float> &getCostConvergence() const { return cost_convergence; }
	bool getSuccessState() const { return success_state; }

	void addIterationTime(float time);
	void addCostConvergence(const std::vector<float> &cost_convergence);
	void clearPlannerInfo();

};

#endif //RPMPL_PLANNERINFO_H
