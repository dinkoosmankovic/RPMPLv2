//
// Created by dinko on 16.3.21..
//

#include "RRTConnect.h"
#include <nanoflann.hpp>
#include <fstream>
#include "RealVectorSpaceState.h"
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
			 base::Tree("goal", 1)};
	kdtrees = {std::make_shared<KdTree>(ss->getDimensions(), TREES[0], nanoflann::KDTreeSingleIndexAdaptorParams(10)),
			   std::make_shared<KdTree>(ss->getDimensions(), TREES[1], nanoflann::KDTreeSingleIndexAdaptorParams(10))};
	TREES[0].upgradeTree(kdtrees[0], start, nullptr);
	TREES[1].upgradeTree(kdtrees[1], goal, nullptr);

	//std::shared_ptr<base::State> test = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({M_PI/2,0}));

	//LOG(INFO) << "Min distance start: " << ss->getDistance(start);
	//LOG(INFO) << "Min distance goal: " << ss->getDistance(goal);
	//LOG(INFO) << "Min distance test: " << ss->getDistance(test);
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
	size_t iter = 1;
	planning::rrt::Status status;

	while (true)
	{
		/* Extend */
		q_rand = getSS()->randomState();
		// LOG(INFO) << q_rand->getCoord().transpose();
		q_near = trees[treeIdx]->getNearestState(kdtrees[treeIdx], q_rand);
		// LOG(INFO) << "Iteration: " << iter;
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
		
		// LOG(INFO) << "End of iteration " << iter;
		iter++;
		plannerInfo->addIterationTime(getElapsedTime(time_start));
		plannerInfo->setNumNodes(trees[0]->getStates()->size() + trees[1]->getStates()->size());
		if (checkStoppingCondition(status, time_start))
		{
			plannerInfo->setPlanningTime(getElapsedTime(time_start));
			plannerInfo->setNumIterations(iter);
			return status == planning::rrt::Reached ? true : false;
		}
	}
}

base::Tree planning::rrt::RRTConnect::getTree(int TN) const
{
	return TREES[TN];
}

std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> planning::rrt::RRTConnect::extend(std::shared_ptr<base::State> q, 
																								  std::shared_ptr<base::State> q_e)
{
	std::shared_ptr<base::State> q_new = ss->interpolate(q, q_e, epsilon);
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
	return {planning::rrt::Trapped, q_new};
}

planning::rrt::Status planning::rrt::RRTConnect::connect(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
														 std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	// LOG(INFO) << "Inside connect.";
	std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q);
	std::shared_ptr<base::State> q_new;
	planning::rrt::Status status = planning::rrt::Advanced;
	int num_ext = 0;  // TODO: should be read from configuration
	while (status == planning::rrt::Advanced && num_ext++ < 50)
	{
		tie(status, q_new) = extend(q_temp, q_e);
		if (status != planning::rrt::Trapped)
		{
			tree->upgradeTree(kdtree, q_new, q_temp);
			std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q_new);
		}
	}
	// LOG(INFO) << "extended.";
	return status;
}

void planning::rrt::RRTConnect::computePath()
{
	path.empty();
	std::shared_ptr<base::State> current = TREES[0].getStates()->back();
	while (current->getParent() != nullptr)
	{
		path.emplace_back(current);
		current = current->getParent();
	}
	path.emplace_back(current);
	std::reverse(path.begin(), path.end());

	current = TREES[1].getStates()->back();
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
	else if (plannerInfo->getNumNodes() >= maxNumNodes || getElapsedTime(time_start) > maxPlanningTime)
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



