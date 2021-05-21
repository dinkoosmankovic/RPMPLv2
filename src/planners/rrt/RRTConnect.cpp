//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <RealVectorSpaceState.h>

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
	base::State* q_rand = getSs()->randomState();
	q_rand->setParent(start);
	/*switch(q_rand->getStateSpaceType())
	{
		case StateSpaceType::RealVectorSpace:
			LOG(INFO) << (dynamic_cast<base::RealVectorSpaceState*>(q_rand))->getCoord();
			break;
	}*/
	return true;
}

base::State *planning::rrt::RRTConnect::getStartTree() const
{
	return start;
}

base::State *planning::rrt::RRTConnect::getGoalTree() const
{
	return goal;
}

