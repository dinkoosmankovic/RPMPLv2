//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include <chrono>
#include <fstream>

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
	plannerInfo = std::make_shared<PlannerInfo>();
	startTree.setTreeName("start");
	goalTree.setTreeName("goal");
	startTree.emptyTree();
	goalTree.emptyTree();
	startTree.getStates()->emplace_back(start);
	goalTree.getStates()->emplace_back(goal);
	prepareKdTrees();
	LOG(INFO) << "Planner initialized!";
}

bool planning::rrt::RRTConnect::solve()
{
	// start the clock
	auto start = std::chrono::steady_clock::now();
	// T_start and T_goal are initialized
	std::shared_ptr<base::Tree> Ta = std::make_shared<base::Tree>(startTree);
	std::shared_ptr<base::Tree> Tb = std::make_shared<base::Tree>(goalTree);
	std::shared_ptr<KdTree> Kd_Ta = startKdTree;
	std::shared_ptr<KdTree> Kd_Tb = goalKdTree;
	int MAX_ITER = 3; // TODO: read from configuration file
	for (size_t i = 0; i < MAX_ITER; ++i)
	{
		std::shared_ptr<base::State> q_rand = getSs()->randomState();
		if (extend(Ta, Kd_Ta, q_rand) != Trapped)
		{
			std::shared_ptr<base::State> q_new = Ta->getStates()->back();
			if (connect(Tb, Kd_Tb, q_new) == Reached)
			{
				computePath();
				auto end = std::chrono::steady_clock::now();
				double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
				plannerInfo->setPlanningTime(elapsed);
				return true;
			}
		}
		if (i % 2 == 0)
		{
			Ta = std::make_shared<base::Tree>(startTree);
			Tb = std::make_shared<base::Tree>(goalTree);
			Kd_Ta = startKdTree;
			Kd_Tb = goalKdTree;
		}
		else
		{
			Tb = std::make_shared<base::Tree>(startTree);
			Ta = std::make_shared<base::Tree>(goalTree);
			Kd_Ta = goalKdTree;
			Kd_Tb = startKdTree;
		}
		auto end = std::chrono::steady_clock::now();
		double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		plannerInfo->addIterationTime(elapsed);
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
	if (q_new != nullptr && ss->isValid(q_near, q_new))
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

void planning::rrt::RRTConnect::outputPlannerData(std::string filename) const
{
	std::ofstream outputFile;
	outputFile.open(filename);
	if (outputFile.is_open())
	{
		outputFile << "Space Type: " << ss->getStateSpaceType() << std::endl;
		outputFile << "Space dimension: " << ss->getDimensions() << std::endl;
		outputFile << "Planner type:\t" << "RRTConnect" << std::endl;
		outputFile << startTree;
		outputFile << goalTree;
		outputFile << "Path:" << std::endl;
		for (int i = 0; i < path.size(); i++)
		{
			outputFile << path.at(i) << std::endl;
		}
		outputFile.close();
	}
	else
	{
		throw "Cannot open file"; // std::something exception perhaps?
	}
}



