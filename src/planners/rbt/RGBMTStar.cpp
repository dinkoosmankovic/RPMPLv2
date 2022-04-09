//
// Created by nermin on 20.02.22.
//

#include "RGBMTStar.h"
#include "ConfigurationReader.h"
#include <nanoflann.hpp>
#include <fstream>
#include <glog/log_severity.h>
#include <glog/logging.h>

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
    numStates = {1, 1};
    costOpt = INFINITY;
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
    std::shared_ptr<base::State> q_con0;    // State that is connecting the start tree with the goal tree
    std::shared_ptr<base::State> q_con1;    // State that is connecting the goal tree with the start tree
	std::shared_ptr<base::State> q_rand, q_near, q_new;
	planning::rrt::Status status;
    std::vector<int> treesExist;            // List of trees for which a new tree is extended to
    std::vector<int> treesReached;          // List of reached trees
    std::vector<int> treesConnected;        // List of connected trees
    std::vector<std::shared_ptr<base::State>> statesReached;       // Reached states from other trees
    float cost;
    plannerInfo->setNumIterations(0);
    plannerInfo->setNumStates(2);

    while (true)
    {
		// LOG(INFO) << "Iteration: " << plannerInfo->getNumIterations();
		LOG(INFO) << "Num states: " << plannerInfo->getNumStates();
        std::cout << "Num main: " << numStates[0] + numStates[1] << " Num local: " << plannerInfo->getNumStates() - numStates[0] - numStates[1] << std::endl;
        
		q_rand = getRandomState();
        if (plannerInfo->getNumStates() > 2 * (numStates[0] + numStates[1]))     // If local trees contain more states than main trees
        {
            std::cout << "Standard RGBT extension ............ " << std::endl;
            treeIdx = (numStates[0] < numStates[1]) ? 0 : 1;
            q_near = trees[treeIdx]->getNearestStateV2(q_rand);
            tie(status, q_new) = connectGenSpine(q_near, q_rand);
            if (status != planning::rrt::Status::Trapped)
                optimize(q_new, trees[treeIdx], kdtrees[0], q_near);
        }
        else
        {
            std::cout << "Adding a new local tree" << std::endl;
            // Adding a new local tree rooted in 'q_rand'
            TREES.emplace_back(base::Tree("local", treeNewIdx));
            // kdtrees.emplace_back(std::make_shared<KdTree>(ss->getDimensions(), TREES[treeNewIdx], nanoflann::KDTreeSingleIndexAdaptorParams(10)));
            // TREES[treeNewIdx].upgradeTree(kdtrees[treeNewIdx], q_rand, nullptr, -1, nullptr, 0);
            TREES[treeNewIdx].upgradeTree(q_rand, nullptr, -1, nullptr, 0);
            trees.emplace_back(std::make_shared<base::Tree>(TREES[treeNewIdx]));
            treesExist.clear();
            treesReached.clear();
            treesConnected.clear();
            statesReached.clear();
            statesReached = std::vector<std::shared_ptr<base::State>>(treeNewIdx, nullptr);

            // Considering all previous trees
            for (int idx = 0; idx < treeNewIdx; idx++)
            {
                // If the connection with 'q_near' is not possible, attempt to connect with 'parent(q_near)', etc.
                // q_near = trees[idx]->getNearestState(kdtrees[idx], q_rand);
                q_near = trees[idx]->getNearestStateV2(q_rand);
                std::shared_ptr<base::State> q_nearNew = ss->newState(q_near);
                while (true)
                {
                    tie(status, q_new) = connectGenSpine(q_rand, q_nearNew);
                    if (status == planning::rrt::Reached || q_nearNew->getParent() == nullptr)
                        break;
                    else
                        q_nearNew = q_nearNew->getParent();
                }

                // Whether currently considering tree ('treeNewIdx'-th tree) is reached
                cost = getCostToCome(q_rand, q_new);
                if (status == planning::rrt::Reached)
                {
                    // If 'idx-th' tree is reached
                    // trees[treeNewIdx]->upgradeTree(kdtrees[treeNewIdx], q_new, q_rand, q_nearNew->getDistance(), q_nearNew->getPlanes(), cost);
                    trees[treeNewIdx]->upgradeTree(q_new, q_rand, q_nearNew->getDistance(), q_nearNew->getPlanes(), cost);
                    treesExist.emplace_back(idx);
                    treesReached.emplace_back(idx);
                    statesReached[idx] = q_nearNew;
                }
                else if (cost > RRTConnectConfig::EPS_STEP)
                {
                    // trees[treeNewIdx]->upgradeTree(kdtrees[treeNewIdx], q_new, q_rand);
                    trees[treeNewIdx]->upgradeTree(q_new, q_rand, -1, nullptr, cost);
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
                    std::random_device rd;
                    std::mt19937 generator(rd());
                    std::uniform_real_distribution<float> distribution(0.0, 1.0);
                    if (distribution(generator) > (float) numStates[1] / (numStates[0] + numStates[1]))
                    {
                        treeIdx = treesReached[1];     // 'q_rand' will be joined to the second main tree
                    }
                }

                // Considering all edges from the new tree
                // q_rand = optimize(q_rand, trees[treeIdx], kdtrees[treeIdx], statesReached[treeIdx]);
                q_rand = optimize(q_rand, trees[treeIdx], kdtrees[0], statesReached[treeIdx]);
                int k = 0;  // Index of the state from the new tree 'treeNewIdx'
                for (int idx : treesExist)
                {
                    k += 1;
                    if (idx == treeIdx)  // It was considered previously, so just skip now
                    {
                        continue;
                    }

                    // Unification of tree 'idx' with 'treeIdx'. Main trees are never unified mutually
                    // q_new = optimize(trees[treeNewIdx]->getState(k), trees[treeIdx], kdtrees[treeIdx], q_rand); // From 'k'-th reached state to tree 'treeIdx'
                    q_new = optimize(trees[treeNewIdx]->getState(k), trees[treeIdx], kdtrees[0], q_rand);
                    
                    if (idx > 1 && std::find(treesReached.begin(), treesReached.end(), idx) != treesReached.end())
                    {                 
                        // unifyTrees(trees[idx], trees[treeIdx], kdtrees[treeIdx], statesReached[idx], q_new);
                        unifyTrees(trees[idx], trees[treeIdx], kdtrees[0], statesReached[idx], q_new);
                        treesConnected.emplace_back(idx);
                    }

                    // The connection of 'q_rand' with both main trees exists
                    else if (idx < 2 && mainTreesReached(treesReached))
                    {
                        cost = q_new->getCost() + statesReached[idx]->getCost();
                        if (cost < costOpt)    // The optimal connection between main trees is stored
                        {
                            costOpt = cost;      
                            LOG(INFO) << "************************************** Cost: " << costOpt;
                            if (treeIdx == 0)
                            {
                                q_con0 = q_new;
                                q_con1 = statesReached[idx];
                            }
                            else
                            {
                                q_con0 = statesReached[idx];
                                q_con1 = q_new;
                            }
                        }
                    }
                }

                // Deleting trees that have been connected
                treesConnected.emplace_back(treeNewIdx);
                deleteTrees(trees, treesConnected);
                treeNewIdx -= treesConnected.size() - 1;
            }
            else    // If there are no reached trees, then the new tree is added to 'trees'
                treeNewIdx += 1;
        }

        plannerInfo->setNumIterations(plannerInfo->getNumIterations() + 1);
		plannerInfo->addIterationTime(getElapsedTime(time_start));
		size_t numStatesTotal = 0;
        numStates.resize(treeNewIdx);
        for(int idx = 0; idx < treeNewIdx; idx++)
        {
            numStates[idx] = trees[idx]->getStates()->size();
		    numStatesTotal += numStates[idx];
        }
        plannerInfo->addCostConvergence(std::vector<float>(numStatesTotal - plannerInfo->getNumStates(), costOpt));
        plannerInfo->setNumStates(numStatesTotal);
		if (checkStoppingCondition(q_con0, q_con1, time_start))
		{
			plannerInfo->setPlanningTime(getElapsedTime(time_start));
			return (costOpt < INFINITY) ? true : false;
		}
    }
}

std::shared_ptr<base::State> planning::rbt::RGBMTStar::getRandomState(){
    while (true)
    {
        std::shared_ptr<base::State> q_rand = ss->randomState();    // Uniform distribution
        // if (TREES.size() > 2)
        // {
        //     int idx = (numStates[0] < numStates[1]) ? 0 : 1;
        //     int numLocal = plannerInfo->getNumStates() - numStates[0] - numStates[1];
        //     if (numLocal > numStates[0] + numStates[1])
        //     {
        //         // std::cout << "Gaussian dist." << std::endl;
        //         // std::cout << "Num local / Num main: " << numLocal << " / " << numStates[idx] << std::endl;
        //         std::random_device rd;
        //         std::mt19937 generator(rd());
        //         std::uniform_int_distribution<int> dist_idx(0, TREES[idx].getStates()->size() - 1);
        //         std::shared_ptr<base::State> q = TREES[idx].getState(dist_idx(generator));
	    //         std::vector<std::vector<float>> limits = ss->robot->getLimits();
        //         float coeff = (numStates[0] + numStates[1]) / (6.0 * numLocal);
                
        //         // Gaussian distribution
        //         for (int i = 0; i < ss->getDimensions(); i++)
        //         {   
        //             std::normal_distribution<float> distribution(q->getCoord(i), (limits[i][1] - limits[i][0]) * coeff);
        //             q_rand->setCoord(distribution(generator), i);
                    
        //             if (q_rand->getCoord(i) < limits[i][0])
        //                 q_rand->setCoord(limits[i][0], i);
        //             else if (q_rand->getCoord(i) > limits[i][1])
        //                 q_rand->setCoord(limits[i][1], i);
        //         }
        //     }
        // }
        if (ss->isValid(q_rand))    // If 'q_rand' is collision-free, it is accepted
            return q_rand;
    }
}

// Connect state 'q' with state 'q_e'
// Return 'Status'
// Return 'q_new': the reached state
std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> 
    planning::rbt::RGBMTStar::connectGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	float d_c = getDistance(q);
	std::shared_ptr<base::State> q_new = q;
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	planning::rrt::Status status = planning::rrt::Advanced;
	int num_ext = 0;
	while (status == planning::rrt::Advanced)
	{
		std::shared_ptr<base::State> q_temp = ss->newState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new_list) = extendGenSpine(q_temp, q_e);
			q_new = q_new_list->back();
            d_c = getDistance(q_new);
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
            if (++num_ext > 10)
            {
                d_c = getDistance(q_new);   
                num_ext = 0;
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

// State 'q' from another tree is optimally connected to 'tree'
// 'q_reached' is a state from 'tree' that is reached by 'q'
// return 'q_new': state 'q' which now belongs to 'tree'
std::shared_ptr<base::State> planning::rbt::RGBMTStar::optimize(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree,
                                                                std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_reached)
{
    // Finding the optimal connection to the predecessors of 'q_reached' until the collision occurs
    std::shared_ptr<base::State> q_reachedNew = ss->newState(q_reached);
    while (q_reachedNew->getParent() != nullptr)
    {
        if (std::get<0>(connectGenSpine(q, q_reachedNew->getParent())) == planning::rrt::Reached)
            q_reachedNew = q_reachedNew->getParent();
        else
            break;
    }

    std::shared_ptr<base::State> q_opt; 
    if (q_reachedNew->getParent() != nullptr)
    {
        Eigen::VectorXf q_opt_ = q_reachedNew->getCoord();  // It is surely collision-free. It will become an optimal state
        Eigen::VectorXf q_parent = q_reachedNew->getParent()->getCoord();   // Needs to be collision-checked
        std::shared_ptr<base::State> q_middle = ss->randomState();
        float D = (q_opt_ - q_parent).norm();
        bool update = false;
        for (int i = 0; i < floor(log2(10 * D)); i++)
        {
            q_middle->setCoord((q_opt_ + q_parent) / 2);
            if (std::get<0>(connectGenSpine(q, q_middle)) == planning::rrt::Reached)
            {
                q_opt_ = q_middle->getCoord();
                update = true;
            }
            else
                q_parent = q_middle->getCoord();
        }
        if (update)
        {
            q_opt = ss->newState(q_opt_);
            float cost = q_reachedNew->getParent()->getCost() + getCostToCome(q_reachedNew->getParent(), q_opt);
            // tree->upgradeTree(kdtree, q_opt, q_reachedNew->getParent(), -1, nullptr, cost);
            tree->upgradeTree(q_opt, q_reachedNew->getParent(), -1, nullptr, cost);
            q_opt->addChild(q_reachedNew);
            std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children = q_reachedNew->getParent()->getChildren();
            for (int i = 0; i < children->size(); i++)
            {
                if (ss->equal(children->at(i), q_reachedNew))
                {
                    children->erase(children->begin() + i); // Deleting the child, since it is previously added in upgradeTree when adding 'q_opt'
                    break;
                }
            }
            q_reachedNew->setParent(q_opt);
        }
        else
            q_opt = q_reachedNew;
    }
    else
        q_opt = q_reachedNew;

    std::shared_ptr<base::State> q_new = ss->newState(q->getCoord());
    float cost = q_opt->getCost() + getCostToCome(q_opt, q_new);
    // tree->upgradeTree(kdtree, q_new, q_opt, q_new->getDistance(), q_new->getPlanes(), cost);
    tree->upgradeTree(q_new, q_opt, q_new->getDistance(), q_new->getPlanes(), cost);
    return q_new;
}

// Optimally unifies a local tree 'tree' with 'tree0'
// 'q_con' - a state that connects 'tree' with 'tree0'
// 'q0_con' - a state that connects 'tree0' with 'tree'
void planning::rbt::RGBMTStar::unifyTrees(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::Tree> tree0, 
    std::shared_ptr<KdTree> kdtree0, std::shared_ptr<base::State> q_con, std::shared_ptr<base::State> q0_con)
{
    std::shared_ptr<base::State> q_considered = nullptr;
    std::shared_ptr<base::State> q_conNew = ss->newState(q_con);
    std::shared_ptr<base::State> q0_conNew = ss->newState(q0_con);
    while (true)
    {
        considerChildren(q_conNew, tree0, kdtree0, q0_conNew, q_considered);
        if (q_conNew->getParent() == nullptr)
            break;

        q0_conNew = optimize(q_conNew->getParent(), tree0, kdtree0, q0_conNew);
        q_considered = q_conNew;
        q_conNew = q_conNew->getParent();
    }
}

// Consider all children (except 'q_considered', that has already being considered) of 'q', and connect them optimally to 'tree0' 
void planning::rbt::RGBMTStar::considerChildren(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree0,
    std::shared_ptr<KdTree> kdtree0, std::shared_ptr<base::State> q0_con, std::shared_ptr<base::State> q_considered)
{
    // Remove child 'q_considered' of 'q' from 'tree'
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children = q->getChildren();
    if (q_considered != nullptr)
    {
        for (int i = 0; i < children->size(); i++)
        {
            if (ss->equal(children->at(i), q_considered))
            {
                children->erase(children->begin() + i);
                break;
            }
        }
    }

    for (int i = 0; i < children->size(); i++)
    {
        std::shared_ptr<base::State> q0_conNew = optimize(children->at(i), tree0, kdtree0, q0_con);
        if (children->at(i)->getChildren()->size() > 0)   // child has its own children
            considerChildren(children->at(i), tree0, kdtree0, q0_conNew, nullptr);  // 'nullptr' means that no children will be removed
    }
}

// Delete all trees with indices 'idx'
void planning::rbt::RGBMTStar::deleteTrees(std::vector<std::shared_ptr<base::Tree>> &trees, std::vector<int> &treesConnected)
{
    for (int i = treesConnected.size()-1; i >= 0; i--)
    {
        trees.erase(trees.begin() + treesConnected[i]);
        TREES.erase(TREES.begin() + treesConnected[i]);
    }
}

bool planning::rbt::RGBMTStar::checkStoppingCondition(std::shared_ptr<base::State> q_con0, std::shared_ptr<base::State> q_con1, 
                                                      std::chrono::steady_clock::time_point &time_start)
{
    if(RGBMTStarConfig::RETURN_WHEN_PATH_IS_FOUND && costOpt < INFINITY || 
       plannerInfo->getNumStates() >= RRTConnectConfig::MAX_NUM_STATES || 
       getElapsedTime(time_start) >= RRTConnectConfig::MAX_PLANNING_TIME ||
       plannerInfo->getNumIterations() >= RRTConnectConfig::MAX_ITER)
    {
        if(costOpt < INFINITY)
        {
		    computePath(q_con0, q_con1);
            return true;
        }
        else
            return true;
    }        
    return false;
}

void planning::rbt::RGBMTStar::outputPlannerData(std::string filename, bool outputStatesAndPaths, bool appendOutput) const
{
	std::ofstream outputFile;
	outputFile.open(filename);
	if (outputFile.is_open())
	{
		outputFile << "Space Type: " << ss->getStateSpaceType() << std::endl;
		outputFile << "Space dimension: " << ss->getDimensions() << std::endl;
		outputFile << "Planner type:\t" << "RGBMTStar" << std::endl;
        for (int i = 0; i < TREES.size(); i++)
        {
            outputFile << TREES[i];
        }
		outputFile << "Cost convergence: " << std::endl;
        for (int i = 0; i < plannerInfo->getNumStates(); i++)
        {
            outputFile << i << "\t" << plannerInfo->getCostConvergence()[i] << std::endl;
        }
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
