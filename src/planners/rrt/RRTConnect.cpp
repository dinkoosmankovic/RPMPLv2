//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <nanoflann.hpp>
#include <fstream>
#include "RealVectorSpaceState.h"
#include "ConfigurationReader.h"
#include <glog/log_severity.h>
#include <glog/logging.h>

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
	TREES[0].clearTree();
	TREES[1].clearTree();
	path.clear();
}

void planning::rrt::RRTConnect::initPlanner()
{
	LOG(INFO) << "Initializing planner...";
	plannerInfo = std::make_shared<PlannerInfo>();
	TREES = {base::Tree("start", 0), 
			 base::Tree("goal",  1)};
	kdtrees = {std::make_shared<KdTree>(ss->getDimensions(), TREES[0], nanoflann::KDTreeSingleIndexAdaptorParams(10)),
			   std::make_shared<KdTree>(ss->getDimensions(), TREES[1], nanoflann::KDTreeSingleIndexAdaptorParams(10))};
	TREES[0].upgradeTree(kdtrees[0], start, nullptr);
	TREES[1].upgradeTree(kdtrees[1], goal, nullptr);
	LOG(INFO) << "Planner initialized!";
}

bool planning::rrt::RRTConnect::solve()
{
	// start the clock
	auto time_start = std::chrono::steady_clock::now();
	// T_start and T_goal are initialized
	std::vector<std::shared_ptr<base::Tree>> trees = {std::make_shared<base::Tree>(TREES[0]),
													  std::make_shared<base::Tree>(TREES[1])};
	int treeIdx = 0;  // Determines the tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal
	std::shared_ptr<base::State> q_rand, q_near, q_new;
	planning::rrt::Status status;
	plannerInfo->setNumIterations(0);

	while (true)
	{
		/* Extend */
		q_rand = getSS()->randomState();
		// LOG(INFO) << q_rand->getCoord().transpose();
		q_near = trees[treeIdx]->getNearestState(kdtrees[treeIdx], q_rand);
		// LOG(INFO) << "Iteration: " << plannerInfo->getNumIterations();
		// LOG(INFO) << "Tree: " << trees[treeIdx]->getTreeName();
		tie(status, q_new) = extend(q_near, q_rand);

		if (status != planning::rrt::Trapped)
		{	
			// LOG(INFO) << "Not Trapped";
			trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new, q_near);

			/* Connect */
            treeIdx = 1 - treeIdx; 	// Swapping trees
			// LOG(INFO) << "Trying to connect to: ";// << q_new->getCoord().transpose() << " from " << trees[treeIdx]->getTreeName();
			q_near = trees[treeIdx]->getNearestState(kdtrees[treeIdx], q_new);
			status = connect(trees[treeIdx], kdtrees[treeIdx], q_near, q_new);
		}
		else 
		{
			treeIdx = 1 - treeIdx; 	// Swapping trees
		}
		
        plannerInfo->setNumIterations(plannerInfo->getNumIterations() + 1);
		plannerInfo->addIterationTime(getElapsedTime(time_start));
		plannerInfo->setNumStates(trees[0]->getStates()->size() + trees[1]->getStates()->size());
		if (checkStoppingCondition(status, time_start))
		{
			plannerInfo->setPlanningTime(getElapsedTime(time_start));
			return status == planning::rrt::Reached ? true : false;
		}
	}
}

base::Tree planning::rrt::RRTConnect::getTree(int treeIdx) const
{
	return TREES[treeIdx];
}

std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> planning::rrt::RRTConnect::extend(std::shared_ptr<base::State> q, 
																								  std::shared_ptr<base::State> q_e)
{
	std::shared_ptr<base::State> q_new = ss->interpolate(q, q_e, RRTConnectConfig::EPS_STEP);
	// LOG(INFO) << q_new->getCoord().transpose();
	if (q_new != nullptr && ss->isValid(q, q_new))
	{
		if (ss->equal(q_new, q_e))
		{
			// LOG(INFO) << "Reached";
			return {planning::rrt::Reached, q_new};
		}
		else
		{
			// LOG(INFO) << "Advanced.";
			return {planning::rrt::Advanced, q_new};
		}
	}
	return {planning::rrt::Trapped, q};
}

planning::rrt::Status planning::rrt::RRTConnect::connect(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
														 std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	// LOG(INFO) << "Inside connect.";
	std::shared_ptr<base::State> q_new = q;
	planning::rrt::Status status = planning::rrt::Advanced;
	int numExt = 0;  // TODO: should be read from configuration
	while (status == planning::rrt::Advanced && numExt++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp = ss->newState(q_new);
		tie(status, q_new) = extend(q_temp, q_e);
		if (status != planning::rrt::Trapped)
		{
			tree->upgradeTree(kdtree, q_new, q_temp);
		}
	}
	// LOG(INFO) << "extended.";
	return status;
}

void planning::rrt::RRTConnect::computePath(std::shared_ptr<base::State> q_con0, std::shared_ptr<base::State> q_con1)
{
	path.clear();
	if (q_con0 == nullptr)
	{
		q_con0 = TREES[0].getStates()->back();
	}
	while (q_con0->getParent() != nullptr)
	{
		path.emplace_back(q_con0->getParent());
		q_con0 = q_con0->getParent();
	}
	std::reverse(path.begin(), path.end());

	if (q_con1 == nullptr)
	{
		q_con1 = TREES[1].getStates()->back();
	}
	while (q_con1 != nullptr)
	{
		path.emplace_back(q_con1);
		q_con1 = q_con1->getParent();
	}
}

const std::vector<std::shared_ptr<base::State>> &planning::rrt::RRTConnect::getPath() const
{
	return path;
}

float planning::rrt::RRTConnect::getElapsedTime(std::chrono::steady_clock::time_point &time_start)
{
	auto end = std::chrono::steady_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(end - time_start).count();
}

bool planning::rrt::RRTConnect::checkStoppingCondition(Status status, std::chrono::steady_clock::time_point &time_start)
{
	if (status == planning::rrt::Reached)
	{
		computePath();
		return true;
	}
	else if (plannerInfo->getNumStates() >= RRTConnectConfig::MAX_NUM_STATES || 
			 getElapsedTime(time_start) >= RRTConnectConfig::MAX_PLANNING_TIME ||
			 plannerInfo->getNumIterations() >= RRTConnectConfig::MAX_ITER)
	{
		return true;
	}
	else
	return false;
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
		outputFile << "Planner info: \n";
		outputFile << "\t\t Number of nodes:\t" << plannerInfo->getNumStates() << std::endl;
		outputFile << "\t\t Planning time(ms):\t" << plannerInfo->getPlanningTime() << std::endl;
		outputFile << TREES[0];
		outputFile << TREES[1];
		if (path.size() > 0)
		{
			outputFile << "Path:" << std::endl;
			for (int i = 0; i < path.size(); i++)
			{
				outputFile << path.at(i) << std::endl;
			}
		}
		outputFile.close();
	}
	else
	{
		throw "Cannot open file"; // std::something exception perhaps?
	}
}