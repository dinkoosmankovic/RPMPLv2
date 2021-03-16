//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"

planning::rrt::RRTConnect::RRTConnect(base::StateSpace *ss_) : AbstractPlanner(ss_)
{

}

planning::rrt::RRTConnect::RRTConnect(base::StateSpace *ss_, base::State *start_,
									  base::State *goal_) : AbstractPlanner(ss_)
{
	start = start_;
	goal = goal_;
}

planning::rrt::RRTConnect::~RRTConnect() {}

bool planning::rrt::RRTConnect::solve()
{
	return false;
}

