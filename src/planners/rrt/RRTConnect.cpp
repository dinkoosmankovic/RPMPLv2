//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <RealVectorSpaceState.h>
#include <nanoflann.hpp>

planning::rrt::RRTConnect::RRTConnect(base::StateSpace *ss_) : AbstractPlanner(ss_)
{
	initPlanner();
}

planning::rrt::RRTConnect::RRTConnect(base::StateSpace *ss_, base::State *start_,
									  base::State *goal_) : AbstractPlanner(ss_), ss(ss_)
{
	initPlanner();
	start = start_;
	goal = goal_;
	startTree.getStates()->emplace_back(start);
	goalTree.getStates()->emplace_back(goal);
}

planning::rrt::RRTConnect::~RRTConnect() {}

void planning::rrt::RRTConnect::initPlanner()
{
	//startTree = base::Tree();
	//goalTree = base::Tree();
	startTree.emptyTree();
	goalTree.emptyTree();
	prepareKdTrees();
}

bool planning::rrt::RRTConnect::solve()
{
	// T_start and T_goal are initialized
	// TODO: FLANN should be used!!!
	base::State* q_rand = getSs()->randomState();
	q_rand->setParent(start);
	int MAX_ITER = 1000; // TODO: Really man...
	for (size_t i = 0; i < MAX_ITER; ++i)
	{
		base::State* q_rand = getSs()->randomState();
		q_rand->setParent(start);

	}


	/*switch(q_rand->getStateSpaceType())
	{
		case StateSpaceType::RealVectorSpace:
			LOG(INFO) << (dynamic_cast<base::RealVectorSpaceState*>(q_rand))->getCoord();
			break;
	}*/
	return true;
}

base::Tree planning::rrt::RRTConnect::getStartTree() const
{
	return startTree;
}

base::Tree planning::rrt::RRTConnect::getGoalTree() const
{
	return goalTree;
}

planning::rrt::Status planning::rrt::RRTConnect::extend(base::Tree tree, base::State *q_rand)
{

	return planning::rrt::Trapped;
}

base::State *planning::rrt::RRTConnect::get_q_near(base::State* q)
{

	return nullptr;
}

void planning::rrt::RRTConnect::prepareKdTrees()
{
	int dim = ss->getDimensions();
	startKdTree = new KdTree(dim, startTree, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
	goalKdTree = new KdTree(dim, goalTree, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
}