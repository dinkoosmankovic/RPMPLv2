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
	if ( !ss->isValid(start) || !ss->isValid(goal))
		throw std::domain_error("Start or goal positions are invalid!");
	startTree.emptyTree();
	goalTree.emptyTree();
	start = start_;
	goal = goal_;
	startTree.getStates()->emplace_back(start);
	goalTree.getStates()->emplace_back(goal);
	prepareKdTrees();
	startKdTree->addPoints(1, 2);
	goalKdTree->addPoints(1, 2);
	//addNode(&startTree, startKdTree, start);
	//addNode(&goalTree, goalKdTree, goal);
	//prepareKdTrees();
	LOG(INFO) << "Planner constructed!";
}

planning::rrt::RRTConnect::~RRTConnect() {}

void planning::rrt::RRTConnect::initPlanner()
{
	//startTree = new base::Tree();
	//goalTree = new base::Tree();
	//startTree.emptyTree();
	//goalTree.emptyTree();
	prepareKdTrees();
	LOG(INFO) << "Planner initialized!";
}

bool planning::rrt::RRTConnect::solve()
{
	// T_start and T_goal are initialized
	// TODO: FLANN should be used!!!
	LOG(INFO) << "Finding path...";
	base::Tree* currentTree = &startTree;
	KdTree* currentKdTree = startKdTree;
	int MAX_ITER = 1; // TODO: needs to be obtained from configuration file
	for (size_t i = 0; i < MAX_ITER; ++i)
	{
		base::State* q_rand = getSs()->randomState();
		LOG(INFO) << "Extending tree...";
		if (extend(currentTree, currentKdTree, q_rand) != Trapped)
		{
			LOG(INFO) << "Not Trapped! Trying connect...";
			base::State* q_new = currentTree->getStates()->back();
			if (connect(currentTree, currentKdTree, q_new) == Reached)
			{
				// TODO: Now we need to compute path
				return true;
			}
		}
		if (i % 2 == 0)
		{
			currentTree = &startTree;
			currentKdTree = startKdTree;
		}
		else
		{
			currentTree = &goalTree;
			currentKdTree = goalKdTree;
		}
	}
	return false;

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

void planning::rrt::RRTConnect::addNode(base::Tree* tree, KdTree* kdtree, base::State* q)
{
	int K = tree->getStates()->size();
	tree->getStates()->emplace_back(q);
	kdtree->addPoints(K - 1, K);
}

planning::rrt::Status planning::rrt::RRTConnect::extend(base::Tree* tree, KdTree* kdtree, base::State *q_rand)
{
	base::State* q_near = get_q_near(q_rand);
	base::State* q_new = ss->interpolate(q_near, q_rand, step);
	if (q_new != nullptr)
	{
		q_new->setParent(q_near);
		addNode(tree, kdtree, q_new);
		if (ss->equal(q_new, q_rand))
			return planning::rrt::Reached;
		else
			return planning::rrt::Advanced;
	}
	return planning::rrt::Trapped;
}

planning::rrt::Status planning::rrt::RRTConnect::connect(base::Tree *tree, KdTree* kdtree, base::State* q)
{
	Status s = planning::rrt::Advanced;
	while (s == planning::rrt::Advanced)
	{
		s = extend(tree, kdtree, q);
	}
	return s;
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
	startKdTree->addPoints(0, 1);
	goalKdTree->addPoints(0, 1);
}



