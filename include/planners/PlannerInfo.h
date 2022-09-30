//
// Created by dinko on 26.5.21..
//

#ifndef RPMPL_PLANNERINFO_H
#define RPMPL_PLANNERINFO_H

#include <vector>

class PlannerInfo
{
protected:
	std::vector<float> iteration_times;
	std::vector<float> state_times;
	std::vector<float> cost_convergence;		// Cost vs state convergence rate (cost-state curve)
	std::vector<float> routine_times; 			// Running times for the specified routine
	std::vector<float> replanning_times; 		// Running times for the replanning routine
	float planning_time;
	size_t num_collision_queries;
	size_t num_distance_queries;
	size_t num_states;
	size_t num_iterations;
	bool success_state = false;					// Did planner succeed to find a solution?

public:
	void addIterationTime(float time);
	void addStateTimes(const std::vector<float> &state_times);
	void addCostConvergence(const std::vector<float> &cost_convergence);
	void addRoutineTime(float time);
	void addReplanningTime(float time);
	void setPlanningTime(float planning_time_) { planning_time = planning_time_; }
	void setNumCollisionQueries(size_t num_collision_queries_) { num_collision_queries = num_collision_queries_; }
	void setNumDistanceQueries(size_t num_distance_queries_) { num_distance_queries = num_distance_queries_; }
	void setNumStates(size_t num_states_) { num_states = num_states_; }
	void setNumIterations(size_t num_iterations_) { num_iterations = num_iterations_; }
	void setSuccessState(bool success_state_) { success_state = success_state_; }

	const std::vector<float> &getIterationTimes() const { return iteration_times; }
	const std::vector<float> &getStateTimes() const { return state_times; }
	const std::vector<float> &getCostConvergence() const { return cost_convergence; }
	const std::vector<float> &getRoutineTimes() const {return routine_times; }
	const std::vector<float> &getReplanningTimes() const {return replanning_times; }
	float getPlanningTime() const { return planning_time; }
	size_t getNumCollisionQueries() const { return num_collision_queries; }
	size_t getNumDistanceQueries() const { return num_distance_queries; }
	size_t getNumStates() const { return num_states; }
	size_t getNumIterations() const { return num_iterations; }
	bool getSuccessState() const { return success_state; }

	void clearPlannerInfo();

};

#endif //RPMPL_PLANNERINFO_H
