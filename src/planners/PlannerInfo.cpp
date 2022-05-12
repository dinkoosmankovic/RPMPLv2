//
// Created by dinko on 26.5.21..
//

#include <PlannerInfo.h>
#include "PlannerInfo.h"


void PlannerInfo::addIterationTime(float time)
{
	iterations_times.emplace_back(time);
}

void PlannerInfo::addStateTimes(const std::vector<float> &state_times)
{
	for (int i = 0; i < state_times.size(); i++)
		PlannerInfo::state_times.emplace_back(state_times[i]);
}

void PlannerInfo::addCostConvergence(const std::vector<float> &cost_convergence)
{
	for (int i = 0; i < cost_convergence.size(); i++)
		PlannerInfo::cost_convergence.emplace_back(cost_convergence[i]);
}

void PlannerInfo::clearPlannerInfo()
{
	iterations_times.clear();
	planning_time = 0;
	num_collision_queries = 0;
	num_distance_queries = 0;
	num_states = 0;
	num_iterations = 0;
}
