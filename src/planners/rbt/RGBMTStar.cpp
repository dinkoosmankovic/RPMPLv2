//
// Created by nermin on 20.02.22.
//

#include "RGBMTStar.h"
#include "ConfigurationReader.h"
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
    for (int i = 2; i < trees.size(); i++)
        trees[i]->clearTree();
}

void planning::rbt::RGBMTStar::initPlanner()
{
    start->setCost(0);
    goal->setCost(0);
    num_states = {1, 1};
    cost_opt = INFINITY;
}

bool planning::rbt::RGBMTStar::solve()
{
	auto time_start = std::chrono::steady_clock::now();     // Start the clock
    auto time_current = time_start;
	int tree_idx;                           // Determines the tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal; >1: local trees
    int tree_new_idx = 2;                   // Index of the new tree
    std::shared_ptr<base::State> q_con0;    // State that is connecting the start tree with the goal tree
    std::shared_ptr<base::State> q_con1;    // State that is connecting the goal tree with the start tree
	std::shared_ptr<base::State> q_rand, q_near, q_new;
	base::State::Status status;
    std::vector<int> trees_exist;                               // List of trees for which a new tree is extended to
    std::vector<int> trees_reached;                             // List of reached trees
    std::vector<int> trees_connected;                           // List of connected trees
    std::vector<std::shared_ptr<base::State>> states_reached;   // Reached states from other trees
    float cost;
    planner_info->setNumIterations(0);
    planner_info->setNumStates(2);
    planner_info->addCostConvergence({INFINITY, INFINITY});
    planner_info->addStateTimes({0, 0});

    while (true)
    {
		// LOG(INFO) << "Iteration: " << planner_info->getNumIterations();
		// LOG(INFO) << "Num. states: " << planner_info->getNumStates();
        // LOG(INFO) << "Num. main: " << num_states[0] + num_states[1] << "\t "
        //           << "Num. local: " << planner_info->getNumStates() - num_states[0] - num_states[1] << std::endl;
        
		q_rand = getRandomState();
        if (planner_info->getNumStates() > 2 * (num_states[0] + num_states[1]))     // If local trees contain more states than main trees
        {
            // LOG(INFO) << "Standard RGBT extension ............ " << std::endl;
            tree_idx = (num_states[0] < num_states[1]) ? 0 : 1;
            q_near = trees[tree_idx]->getNearestState(q_rand);
            // q_near = trees[tree_idx]->getNearestStateV2(q_rand);
            tie(status, q_new) = connectGenSpine(q_near, q_rand);
            if (status != base::State::Status::Trapped)
                optimize(q_new, trees[tree_idx], q_near);
        }
        else
        {
            // Adding a new local tree rooted in 'q_rand'
            // LOG(INFO) << "Adding a new local tree" << std::endl;
            trees.emplace_back(std::make_shared<base::Tree>(base::Tree("local", tree_new_idx)));
            trees[tree_new_idx]->setKdTree(std::make_shared<base::KdTree>(ss->getDimensions(), *trees[tree_new_idx], 
                                                                          nanoflann::KDTreeSingleIndexAdaptorParams(10)));
            trees[tree_new_idx]->upgradeTree(q_rand, nullptr, -1, nullptr, 0);
            trees_exist.clear();
            trees_reached.clear();
            trees_connected.clear();
            states_reached.clear();
            states_reached = std::vector<std::shared_ptr<base::State>>(tree_new_idx, nullptr);

            // Considering all previous trees
            for (int idx = 0; idx < tree_new_idx; idx++)
            {
                // If the connection with 'q_near' is not possible, attempt to connect with 'parent(q_near)', etc.
                q_near = trees[idx]->getNearestState(q_rand);
                // q_near = trees[idx]->getNearestStateV2(q_rand);
                std::shared_ptr<base::State> q_nearNew = ss->newState(q_near);
                while (true)
                {
                    tie(status, q_new) = connectGenSpine(q_rand, q_nearNew);
                    if (status == base::State::Status::Reached || q_nearNew->getParent() == nullptr)
                        break;
                    else
                        q_nearNew = q_nearNew->getParent();
                }

                // Whether currently considering tree ('tree_new_idx'-th tree) is reached
                cost = getCostToCome(q_rand, q_new);
                if (status == base::State::Status::Reached)
                {
                    // If 'idx-th' tree is reached
                    trees[tree_new_idx]->upgradeTree(q_new, q_rand, q_nearNew->getDistance(), q_nearNew->getPlanes(), cost);
                    trees_exist.emplace_back(idx);
                    trees_reached.emplace_back(idx);
                    states_reached[idx] = q_nearNew;
                }
                else if (status == base::State::Status::Advanced)
                {
                    trees[tree_new_idx]->upgradeTree(q_new, q_rand, -1, nullptr, cost);
                    trees_exist.emplace_back(idx);
                }
            }

            // Find the optimal edge towards each reached tree
            if (!trees_reached.empty())
            {
                // The connection of 'q_rand' with both main trees exists
                tree_idx = trees_reached[0];      // Considering the first reached tree
                if (mainTreesReached(trees_reached))
                {
                    std::random_device rd;
                    std::mt19937 generator(rd());
                    std::uniform_real_distribution<float> distribution(0.0, 1.0);
                    if (distribution(generator) > (float) num_states[1] / (num_states[0] + num_states[1]))
                        tree_idx = trees_reached[1];     // 'q_rand' will be joined to the second main tree
                }

                // Considering all edges from the new tree
                q_rand = optimize(q_rand, trees[tree_idx], states_reached[tree_idx]);
                int k = 0;  // Index of the state from the new tree 'tree_new_idx'
                for (int idx : trees_exist)
                {
                    k += 1;
                    if (idx == tree_idx)  // It was considered previously, so just skip now
                        continue;

                    // Unification of tree 'idx' with 'tree_idx'. Main trees are never unified mutually
                    q_new = optimize(trees[tree_new_idx]->getState(k), trees[tree_idx], q_rand);   // From 'k'-th reached state to tree 'tree_idx'
                    
                    if (idx > 1 && std::find(trees_reached.begin(), trees_reached.end(), idx) != trees_reached.end())
                    {
                        unifyTrees(trees[idx], trees[tree_idx], states_reached[idx], q_new);
                        trees_connected.emplace_back(idx);
                    }

                    // The connection of 'q_rand' with both main trees exists
                    else if (idx < 2 && mainTreesReached(trees_reached))
                    {
                        cost = q_new->getCost() + states_reached[idx]->getCost();
                        if (cost < cost_opt)    // The optimal connection between main trees is stored
                        {
                            cost_opt = cost;      
                            // LOG(INFO) << "Cost after " << planner_info->getNumStates() << " states is: " << cost_opt;
                            if (tree_idx == 0)
                            {
                                q_con0 = q_new;
                                q_con1 = states_reached[idx];
                            }
                            else
                            {
                                q_con0 = states_reached[idx];
                                q_con1 = q_new;
                            }
                        }
                    }
                }

                // Deleting trees that have been connected
                trees_connected.emplace_back(tree_new_idx);
                deleteTrees(trees_connected);
                tree_new_idx -= trees_connected.size() - 1;
            }
            else    // If there are no reached trees, then the new tree is added to 'trees'
                tree_new_idx += 1;
        }

		/* Planner info and terminating condition */
		time_current = std::chrono::steady_clock::now();
        planner_info->setNumIterations(planner_info->getNumIterations() + 1);
		planner_info->addIterationTime(getElapsedTime(time_start, time_current));
		size_t numStatesTotal = 0;
        num_states.resize(tree_new_idx);
        for(int idx = 0; idx < tree_new_idx; idx++)
        {
            num_states[idx] = trees[idx]->getNumStates();
		    numStatesTotal += num_states[idx];
        }
        planner_info->addCostConvergence(std::vector<float>(numStatesTotal - planner_info->getNumStates(), cost_opt));
        planner_info->addStateTimes(std::vector<float>(numStatesTotal - planner_info->getNumStates(), planner_info->getIterationsTimes().back()));
        planner_info->setNumStates(numStatesTotal);
		if (checkTerminatingCondition(q_con0, q_con1))
		{
			planner_info->setPlanningTime(planner_info->getIterationsTimes().back());
			return planner_info->getSuccessState();
		}
    }
}

std::shared_ptr<base::State> planning::rbt::RGBMTStar::getRandomState(){
    while (true)
    {
        std::shared_ptr<base::State> q_rand = ss->randomState();    // Uniform distribution
        if (ss->isValid(q_rand))    // If 'q_rand' is collision-free, it is accepted
            return q_rand;
    }
}

// Connect state 'q' with state 'q_e'
// Return 'Status'
// Return 'q_new': the reached state
std::tuple<base::State::Status, std::shared_ptr<base::State>> planning::rbt::RGBMTStar::connectGenSpine
    (std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	float d_c = getDistance(q);
	std::shared_ptr<base::State> q_new = q;
	base::State::Status status = base::State::Status::Advanced;
	int num_ext = 0;
	while (status == base::State::Status::Advanced)
	{
		std::shared_ptr<base::State> q_temp = ss->newState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new) = extendGenSpineV2(q_temp, q_e);
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
inline bool planning::rbt::RGBMTStar::mainTreesReached(std::vector<int> &trees_reached)
{
    return (trees_reached.size() > 1 && trees_reached[0] == 0 && trees_reached[1] == 1) ? true : false;
}

// State 'q' from another tree is optimally connected to 'tree'
// 'q_reached' is a state from 'tree' that is reached by 'q'
// return 'q_new': state 'q' which now belongs to 'tree'
std::shared_ptr<base::State> planning::rbt::RGBMTStar::optimize
    (std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree, std::shared_ptr<base::State> q_reached)
{
    // Finding the optimal connection to the predecessors of 'q_reached' until the collision occurs
    std::shared_ptr<base::State> q_reachedNew = ss->newState(q_reached);
    while (q_reachedNew->getParent() != nullptr)
    {
        if (std::get<0>(connectGenSpine(q, q_reachedNew->getParent())) == base::State::Status::Reached)
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
            if (std::get<0>(connectGenSpine(q, q_middle)) == base::State::Status::Reached)
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
            tree->upgradeTree(q_opt, q_reachedNew->getParent(), -1, nullptr, cost);
            q_opt->addChild(q_reachedNew);
            std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children = q_reachedNew->getParent()->getChildren();
            for (int i = 0; i < children->size(); i++)
            {
                if (ss->isEqual(children->at(i), q_reachedNew))
                {
                    children->erase(children->begin() + i);     // Deleting the child, since it is previously added in upgradeTree when adding 'q_opt'
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
    tree->upgradeTree(q_new, q_opt, q_new->getDistance(), q_new->getPlanes(), cost);
    return q_new;
}

// Optimally unifies a local tree 'tree' with 'tree0'
// 'q_con' - a state that connects 'tree' with 'tree0'
// 'q0_con' - a state that connects 'tree0' with 'tree'
void planning::rbt::RGBMTStar::unifyTrees(std::shared_ptr<base::Tree> tree, std::shared_ptr<base::Tree> tree0, 
                                          std::shared_ptr<base::State> q_con, std::shared_ptr<base::State> q0_con)
{
    std::shared_ptr<base::State> q_considered = nullptr;
    std::shared_ptr<base::State> q_conNew = ss->newState(q_con);
    std::shared_ptr<base::State> q0_conNew = ss->newState(q0_con);
    while (true)
    {
        considerChildren(q_conNew, tree0, q0_conNew, q_considered);
        if (q_conNew->getParent() == nullptr)
            break;

        q0_conNew = optimize(q_conNew->getParent(), tree0, q0_conNew);
        q_considered = q_conNew;
        q_conNew = q_conNew->getParent();
    }
}

// Consider all children (except 'q_considered', that has already being considered) of 'q', and connect them optimally to 'tree0' 
void planning::rbt::RGBMTStar::considerChildren(std::shared_ptr<base::State> q, std::shared_ptr<base::Tree> tree0,
                                                std::shared_ptr<base::State> q0_con, std::shared_ptr<base::State> q_considered)
{
    // Remove child 'q_considered' of 'q' from 'tree'
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> children = q->getChildren();
    if (q_considered != nullptr)
    {
        for (int i = 0; i < children->size(); i++)
        {
            if (ss->isEqual(children->at(i), q_considered))
            {
                children->erase(children->begin() + i);
                break;
            }
        }
    }

    for (int i = 0; i < children->size(); i++)
    {
        std::shared_ptr<base::State> q0_conNew = optimize(children->at(i), tree0, q0_con);
        if (children->at(i)->getChildren()->size() > 0)   // child has its own children
            considerChildren(children->at(i), tree0, q0_conNew, nullptr);  // 'nullptr' means that no children will be removed
    }
}

// Delete all trees with indices 'idx'
void planning::rbt::RGBMTStar::deleteTrees(std::vector<int> &trees_connected)
{
    for (int i = trees_connected.size()-1; i >= 0; i--)
        trees.erase(trees.begin() + trees_connected[i]);
}

bool planning::rbt::RGBMTStar::checkTerminatingCondition(std::shared_ptr<base::State> q_con0, std::shared_ptr<base::State> q_con1)
{
    if (RGBMTStarConfig::RETURN_WHEN_PATH_IS_FOUND && cost_opt < INFINITY || 
        planner_info->getNumStates() >= RRTConnectConfig::MAX_NUM_STATES || 
        planner_info->getIterationsTimes().back() >= RRTConnectConfig::MAX_PLANNING_TIME ||
        planner_info->getNumIterations() >= RRTConnectConfig::MAX_NUM_ITER)
    {
        if (cost_opt < INFINITY)
        {
		    planner_info->setSuccessState(true);
		    computePath(q_con0, q_con1);
            return true;
        }
        else
        {
		    planner_info->setSuccessState(false);
            return true;
        }
    }        
    return false;
}

void planning::rbt::RGBMTStar::outputPlannerData(std::string filename, bool output_states_and_paths, bool append_output) const
{
    std::ofstream output_file;
	std::ios_base::openmode mode = std::ofstream::out;
	if (append_output)
		mode = std::ofstream::app;

	output_file.open(filename, mode);
	if (output_file.is_open())
	{
		output_file << "Space Type:      " << ss->getStateSpaceType() << std::endl;
		output_file << "Space dimension: " << ss->getDimensions() << std::endl;
		output_file << "Planner type:    " << "RGBMT*" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		output_file << "\t Path cost [rad]:      " << planner_info->getCostConvergence().back() << std::endl;
		if (output_states_and_paths)
		{
            for (int i = 0; i < trees.size(); i++)
                output_file << *trees[i];

            output_file << "Cost convergence: \n" 
                        << "Num. states\tCost [rad]\t\tTime [ms]" << std::endl;
            for (int i = 0; i < planner_info->getNumStates(); i++)
                output_file << i+1 << "\t\t" << planner_info->getCostConvergence()[i] << "\t\t" << planner_info->getStateTimes()[i] << std::endl;

			if (path.size() > 0)
			{
				output_file << "Path:" << std::endl;
				for (int i = 0; i < path.size(); i++)
					output_file << path.at(i) << std::endl;
			}
		}
		output_file << std::string(25, '-') << std::endl;
		output_file.close();
	}
	else
		throw "Cannot open file"; // std::something exception perhaps?
}
