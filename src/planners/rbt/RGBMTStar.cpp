//
// Created by nermin on 20.02.22.
//

#include "RGBMTStar.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include <fstream>

planning::rbt::RGBMTStar::RGBMTStar(std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_)
{ 
	initPlanner();
}

planning::rbt::RGBMTStar::RGBMTStar(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                    std::shared_ptr<base::State> goal_) : RGBTConnect(ss_, start_, goal_)
{ 
	initPlanner();
}

planning::rbt::RGBMTStar::~RGBMTStar()
{
    for (int i = 0; i < TREES.size(); i++)
    {
        TREES[i].clearTree();
    }
    path.clear();
}

void planning::rbt::RGBMTStar::initPlanner()
{
    start->setCost(0);
    goal->setCost(0);
    numNodes = {1, 1};
}

bool planning::rbt::RGBMTStar::solve()
{
    // start the clock
	auto time_start = std::chrono::steady_clock::now();
	// T_start and T_goal are initialized
	std::vector<std::shared_ptr<base::Tree>> trees = {std::make_shared<base::Tree>(TREES[0]),
													  std::make_shared<base::Tree>(TREES[1])};
	int treeIdx;            // Determines the tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal; >1: local trees
    int treeNewIdx = 2;     // Index of the new tree
    int treeMainIdx;        // Index of the chosen main tree when the connection of two main trees occured 
	std::shared_ptr<base::State> q_rand, q_near, q_new;
	size_t iter = 1;
    float cost;
	planning::rrt::Status status;
    std::vector<int> treesExist;            // List of trees for which a new tree is extended to
    std::vector<int> treesReached;          // List of reached trees
    std::vector<int> treesConnected;        // List of connected trees
    std::vector<std::shared_ptr<base::State>> statesReached;       // Reached nodes from other trees

    while (true)
    {
        // Adding a new tree rooted in 'q_rand'
		q_rand = ss->randomState();
        TREES.emplace_back(base::Tree("local", treeNewIdx));
        trees.emplace_back(std::make_shared<base::Tree>(TREES[treeIdx]));
        kdtrees.emplace_back(std::make_shared<KdTree>(ss->getDimensions(), TREES[treeNewIdx], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
        trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_rand, nullptr, -1, nullptr, 0);

        treesExist.clear();
        treesReached.clear();
        treesConnected.clear();
        statesReached.clear();

        // Considering all previous trees
        for (int idx = 0; idx < treeNewIdx; idx++)
        {
            // If the connection with 'q_near' is not possible, attempt to connect with 'parent(q_near)', etc.
            q_near = trees[idx]->getNearestState(kdtrees[treeIdx], q_rand);
            std::shared_ptr<base::State> q_nearNew = ss->randomState(); q_nearNew->makeCopy(q_near);
            while (true)
            {
                tie(status, q_new) = connectGenSpine(q_rand, q_nearNew);
                if (status == planning::rrt::Reached || q_nearNew->getParent() == nullptr)
                {
                    break;
                }
                else
                {
                    q_nearNew = q_nearNew->getParent();
                }
            }

            // Whether currently considering tree ('treeNewIdx'-th tree) is reached
            cost = getCostToCome(q_rand, q_new);
            if (status == planning::rrt::Reached)
            {
                // If 'idx-th' tree is reached
                trees[treeNewIdx]->upgradeTree(kdtrees[treeNewIdx], q_new, q_rand, q_nearNew->getDistance(), q_nearNew->getPlanes(), cost);
                treesExist.emplace_back(idx);
                treesReached.emplace_back(idx);
                statesReached.emplace_back(q_nearNew);
            }
            else if (cost > epsilon)
            {
                trees[treeNewIdx]->upgradeTree(kdtrees[treeNewIdx], q_new, q_rand);
                treesExist.emplace_back(idx);
            }
        }

        // Find the optimal edge towards each reached tree
        if (!treesReached.empty())
        {
            // The connection of 'q_rand' with both main trees exists
            treeIdx = treesReached[0];      // Considering the first reached tree
            if (mainTreesReached(treesReached))
            {
                std::default_random_engine generator;
                std::uniform_real_distribution<float> distribution(0.0, 1.0);
                if (distribution(generator) > (float) numNodes[1] / (numNodes[0] + numNodes[1]))
                {
                    treeIdx = treesReached[1];     // 'q_rand' will be joined to the second main tree
                }
            }

            // Considering all edges from the new tree

        }
    }
}

// Connect 'q' with 'q_e'
// 'q_new' is the reached node
std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> 
    planning::rbt::RGBMTStar::connectGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q);
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	std::shared_ptr<base::State> q_new;
	planning::rrt::Status status = planning::rrt::Advanced;
    float d_c = getDistance(q_temp);
	int num_ext = 0;  // TODO: should be read from configuration
	while (status == planning::rrt::Advanced && num_ext++ < 50)
	{
		if (d_c > d_crit)
		{
			tie(status, q_new_list) = extendGenSpine(q_temp, q_e);
            q_new = q_new_list->back();
            d_c = getDistance(q_new);
            std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q_new);	
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
            if (status != planning::rrt::Trapped)
            {
                std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q_new);	
            }
		}	
	}
	return {status, q_new};
}

// Returns cost-to-come from 'q1' to 'q2'
inline float planning::rbt::RGBMTStar::getCostToCome(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2)
{
    return (q1->getCoord() - q2->getCoord()).norm();
}

// Whether both main trees are reached
inline bool planning::rbt::RGBMTStar::mainTreesReached(std::vector<int> &treesReached)
{
    return (treesReached.size() > 1 && treesReached[0] == 0 && treesReached[1] == 1) ? true : false;
}

// Node 'q' is optimally connected to 'tree'
// 'q_reached' is a node from 'tree' that is reached by 'q'
std::shared_ptr<base::State> planning::rbt::RGBMTStar::optimize(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree,
                                                                std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_reached)
{
    // Finding the optimal connection to the predecessors of 'q_reached' until the collision occurs
    std::shared_ptr<base::State> q_reachedNew = ss->randomState(); q_reachedNew->makeCopy(q_reached);
    while (q_reachedNew->getParent() != nullptr)
    {
        if (std::get<0>(connectGenSpine(q, q_reachedNew->getParent())) == planning::rrt::Reached)
        {
            q_reachedNew = q_reachedNew->getParent();
        }
        else
        {
            break;
        }
    }
    std::shared_ptr<base::State> q_opt = ss->randomState(); q_opt->makeCopy(q_reachedNew);     // It is surely collision-free. It will become an optimal node
        
    if (q_reachedNew->getParent() != nullptr)
    {
        std::shared_ptr<base::State> q_par = ss->randomState(); q_par->makeCopy(q_reachedNew->getParent());   // Needs to be collision-checked
        std::shared_ptr<base::State> q_middle = ss->randomState();
        bool update = false;
        for (int i = 0; i < floor(log2(10 * (q_opt->getCoord() - q_par->getCoord()).norm())); i++)
        {
            q_middle->setCoord((q_opt->getCoord() + q_par->getCoord()) / 2);
            if (std::get<0>(connectGenSpine(q, q_middle)))
            {
                q_opt->setCoord(q_middle->getCoord());
                update = true;
            }
            else
            {
                q_par->setCoord(q_middle->getCoord());
            }
        }
        if (update)
        {
            float cost = q_reachedNew->getParent()->getCost() + getCostToCome(q_reachedNew->getParent(), q_opt);
            tree->upgradeTree(kdtree, q_opt, q_reachedNew->getParent(), -1, nullptr, cost);
            q_opt->addChild(q_reachedNew);
            std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children = q_reachedNew->getParent()->getChildren();
            // std::vector<std::shared_ptr<base::State>>::iterator child = find(children->begin(), children->end(), q_reachedNew);
            for (int i = 0; i < children->size(); i++)
            {
                if (children->at(i) == q_reachedNew)
                {
                    children->at(i) = q_opt;   // Modifying the child
                    break;
                }
            }
            q_reachedNew->setParent(q_opt);
        }
    }
    std::shared_ptr<base::State> q_new = ss->randomState(); q_new->makeCopy(q); 
    float cost = q_new->getCost() + getCostToCome(q_opt, q_new);
    tree->upgradeTree(kdtree, q_new, q_opt, q_new->getDistance(), q_new->getPlanes(), cost);
    return q_new;
}

void planning::rbt::RGBMTStar::outputPlannerData(std::string filename) const
{
	std::ofstream outputFile;
	outputFile.open(filename);
	if (outputFile.is_open())
	{
		outputFile << "Space Type: " << ss->getStateSpaceType() << std::endl;
		outputFile << "Space dimension: " << ss->getDimensions() << std::endl;
		outputFile << "Planner type:\t" << "RGBMTStar" << std::endl;
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