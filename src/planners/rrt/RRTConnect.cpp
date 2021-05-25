//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include <iostream>

planning::rrt::RRTConnect::RRTConnect(std::shared_ptr<base::StateSpace> ss_) : AbstractPlanner(ss_)
{
	initPlanner();
}

planning::rrt::RRTConnect::RRTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
									  std::shared_ptr<base::State> goal_) : AbstractPlanner(ss_), ss(ss_)
{
	start = start_;
	goal = goal_;
	if (!ss->isValid(start) || !ss->isValid(goal))
		throw std::domain_error("Start or goal positions are invalid!");
	initPlanner();
}

planning::rrt::RRTConnect::~RRTConnect()
{
	startTree.emptyTree();
	goalTree.emptyTree();
	path.empty();
}

void planning::rrt::RRTConnect::initPlanner()
{
	LOG(INFO) << "Initializing planner...";
	startTree.emptyTree();
	goalTree.emptyTree();
	startTree.getStates()->emplace_back(start);
	goalTree.getStates()->emplace_back(goal);
	prepareKdTrees();
	LOG(INFO) << "Planner initialized!";
}

bool planning::rrt::RRTConnect::solve()
{
	// T_start and T_goal are initialized
	std::shared_ptr<base::Tree> Ta = std::make_shared<base::Tree>(startTree);
	std::shared_ptr<base::Tree> Tb = std::make_shared<base::Tree>(goalTree);
	std::shared_ptr<KdTree> Kd_Ta = startKdTree;
	std::shared_ptr<KdTree> Kd_Tb = goalKdTree;
	int MAX_ITER = 3; // TODO: needs to be obtained from configuration file
	for (size_t i = 0; i < MAX_ITER; ++i)
	{
		std::shared_ptr<base::State> q_rand = getSs()->randomState();
		if (extend(Ta, Kd_Ta, q_rand) != Trapped)
		{
			std::shared_ptr<base::State> q_new = Ta->getStates()->back();
			if (connect(Tb, Kd_Tb, q_new) == Reached)
			{
				computePath();
				return true;
			}
		}
		if (i % 2 == 0)
		{
			Ta = std::make_shared<base::Tree>(startTree);
			Tb = std::make_shared<base::Tree>(goalTree);
			Kd_Ta = startKdTree;
			Kd_Tb = goalKdTree;
		} else
		{
			Tb = std::make_shared<base::Tree>(startTree);
			Ta = std::make_shared<base::Tree>(goalTree);
			Kd_Ta = goalKdTree;
			Kd_Tb = startKdTree;
		}
	}
	return false;
}

base::Tree planning::rrt::RRTConnect::getStartTree() const
{
	return startTree;
}

base::Tree planning::rrt::RRTConnect::getGoalTree() const
{
	return goalTree;
}

void planning::rrt::RRTConnect::addNode(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q)
{
	int K = tree->getStates()->size();
	tree->getStates()->emplace_back(q);
	kdtree->addPoints(K - 1, K - 1);
}

planning::rrt::Status planning::rrt::RRTConnect::extend(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_rand)
{
	std::shared_ptr<base::State> q_near = get_q_near(tree, kdtree, q_rand);
	std::shared_ptr<base::State> q_new = ss->interpolate(q_near, q_rand, step);
	if (q_new != nullptr)
	{
		q_new->setParent(q_near);
		addNode(tree, kdtree, q_new);
		if (ss->equal(q_new, q_rand))
		{
			return planning::rrt::Reached;
		}
		else
		{
			return planning::rrt::Advanced;
		}
	}
	return planning::rrt::Trapped;
}

planning::rrt::Status planning::rrt::RRTConnect::connect(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q)
{
	Status s = planning::rrt::Advanced;
	while (s == planning::rrt::Advanced)
	{
		s = extend(tree, kdtree, q);
	}
	return s;
}

std::shared_ptr<base::State> planning::rrt::RRTConnect::get_q_near(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q)
{
	const size_t num_results = 1;
	size_t ret_index;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr);
	std::vector<double> vec(q->getCoord().data(),
							q->getCoord().data() + q->getCoord().rows() *
												   q->getCoord().cols());
	double *vec_c = &vec[0];
	kdtree->findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	vec_c = nullptr;
	return tree->getStates()->at(ret_index);
}

void planning::rrt::RRTConnect::prepareKdTrees()
{
	int dim = ss->getDimensions();
	startKdTree = std::make_shared<KdTree>( dim, startTree, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
	goalKdTree = std::make_shared<KdTree>( dim, goalTree, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
	startKdTree->addPoints(0, 0);
	goalKdTree->addPoints(0, 0);
}

void planning::rrt::RRTConnect::computePath()
{
	path.empty();
	std::shared_ptr<base::State> current = startTree.getStates()->back();
	while (current->getParent() != nullptr)
	{
		path.emplace_back(current);
		current = current->getParent();
	}
	path.emplace_back(current);
	std::reverse(path.begin(), path.end());
	current = goalTree.getStates()->back();
	while (current != nullptr)
	{
		path.emplace_back(current);
		current = current->getParent();
	}
}

const std::vector<std::shared_ptr<base::State>> &planning::rrt::RRTConnect::getPath() const
{
	return path;
}



