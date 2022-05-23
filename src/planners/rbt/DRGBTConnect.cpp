//
// Created by nermin on 13.04.22.
//

#include "DRGBTConnect.h"
#include "ConfigurationReader.h"
#include <glog/log_severity.h>
#include <glog/logging.h>

planning::rbt::DRGBTConnect::DRGBTConnect(std::shared_ptr<base::StateSpace> ss_) : RGBTConnect(ss_) {}

planning::rbt::DRGBTConnect::DRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                          std::shared_ptr<base::State> goal_) : RGBTConnect(ss_, start_, goal_) {}

planning::rbt::DRGBTConnect::~DRGBTConnect() {}

bool planning::rbt::DRGBTConnect::solve()
{
    auto time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    q_current = start;
    path.emplace_back(start);                                   // State 'start' is added to the realized path
    bool replanning;                                            // Whether path replanning is required
    base::State::Status status;
    int horizon_size = DRGBTConnectConfig::INIT_HORIZON_SIZE;
    planner_info->setNumIterations(0);
    planner_info->setNumStates(1);

    // Getting the inital path
    std::unique_ptr<planning::AbstractPlanner> RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
    RGBTConnect_planner->solve();
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> predefined_path = 
        std::make_shared<std::vector<std::shared_ptr<base::State>>>(RGBTConnect_planner->getPath());
    int idx = 1;    // Index of the next available state from 'predefined_path'. It is starting from 1 since 0-th state is start.
	
    while (true)
    {
        auto time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
        replanning = false;

        // Generating horizon
        if (idx + horizon_size <= predefined_path->size())
        {
            for (int i = idx; i < idx + horizon_size; i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path->at(i), i));
        }
        else if (idx < predefined_path->size())
        {
            for (int i = idx; i < predefined_path->size(); i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path->at(i), i));
            horizon[horizon.size() - 1]->setStatus(HorizonState::Status::goal);
            addRandomStates(horizon_size - horizon.size());
        }
        else
        {
            idx = predefined_path->size();
            if (predefined_path->empty())   // If the initial path was not found, random states are added
                addRandomStates(horizon_size);
            else                            // At least 'goal' remains in the horizon
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path->back(), predefined_path->size() - 1));
        }

        computeHorizon();

        if (idx == predefined_path->size() && q_next->getStatus() != HorizonState::Status::goal)
            replanning = true;

        bool clock_enabled = false;
        status = base::State::Status::Advanced;
        while (status == base::State::Status::Advanced)
        {
            ss->env->updateObstacles();
            
            planner_info->addIterationTime(getElapsedTime(time_iter_start));
            if (clock_enabled)
                time_iter_start = std::chrono::steady_clock::now();
            clock_enabled = true;

            // Update robot current position
            tie(status, q_current) = ss->interpolate(q_current, q_next->getState(), RRTConnectConfig::EPS_STEP);
            path.emplace_back(q_current);
            if (status == base::State::Status::Trapped)
            {
                LOG(INFO) << "Collision occured! ";
                return false;
            }

            if (ss->isEqual(q_current, goal))
            {
                LOG(INFO) << "Goal configuration has been successfully reached! ";
                return true;
            }
            
            computeHorizon();
            
            // Replanning procedure assessment
            if (whetherToReplan() || replanning)
            {
                // Horizon::idx_next = 0;
                RRTConnectConfig::MAX_PLANNING_TIME = 100;
                RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, q_current, goal);
                RGBTConnect_planner->solve();

                if (!RGBTConnect_planner->getPath().empty())   // New path is found, thus update path to the goal
                {
                    LOG(INFO) << "The path has been replanned. ";
                    replanning = false;
                    status = base::State::Status::Trapped;
                    predefined_path = std::make_shared<std::vector<std::shared_ptr<base::State>>>(RGBTConnect_planner->getPath());
                    idx = 1;
                }
                else    // New path is not found
                {
                    LOG(INFO) << "New path is not found. ";
                    replanning = true;
                }
            }

            // Real-time checking
            float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start);
            LOG(INFO) << "Remaining iteration time [ms]: " << time_iter_remain;
            if (time_iter_remain < 0)
                LOG(INFO) << "Real-time is broken !!!";

            // add planner_info ...
        }

        horizon_size = horizon.size();
        if (status == base::State::Status::Reached && q_next->getIndexInPath() != -1)
        {
            // Only states from predefined path that come after 'q_next' are remained in the horizon
            for (int i = horizon_size; i >= 0; i--)
                if (horizon[i]->getIndexInPath() == -1 || horizon[i]->getIndexInPath() <= q_next->getIndexInPath())
                    horizon.erase(horizon.begin() + i);

            idx = q_next->getIndexInPath() + 1;
        }
        else
            horizon.clear();

    }
}

// Compute and update the horizon such that it contains possibly better states 'states'.
// Bad states will be replaced with "better" states.
// Also, next state is computed.
// 'time_max' is the maximal runtime for this function.
void planning::rbt::DRGBTConnect::computeHorizon()
{
    float d_c = getDistance(q_current);
    int new_horizon_size = std::min((int) std::floor(DRGBTConnectConfig::INIT_HORIZON_SIZE * (1 + RBTConnectConfig::D_CRIT / d_c)),
                                    5 * ss->getDimensions() * DRGBTConnectConfig::INIT_HORIZON_SIZE);
    
    if (new_horizon_size < horizon.size())
        shortenHorizon(horizon.size() - new_horizon_size);
    else    // If 'new_horizon_size' has increased, or little states exist, then random states are added
        addRandomStates(new_horizon_size - horizon.size());

    // Lateral spines are added
    int num_lateral_spines = 2 * ss->getDimensions() - 2;
    addLateralStates(num_lateral_spines);

    for (int i = 0; i < horizon.size(); i++)
    {
        computeReachedState(q_current, horizon[i]);

        // Bad and critical states are modified
        if (horizon[i]->getStatus() == HorizonState::Status::bad || 
            horizon[i]->getStatus() == HorizonState::Status::critical)
        {
            modifyState(horizon[i]);
            computeReachedState(q_current, horizon[i]);
        }
    }

    computeNextState();
}

// Shorten the horizon by removing 'num' states. Surplus states are deleted, and best states holds priority.
void planning::rbt::DRGBTConnect::shortenHorizon(int num)
{
    int num_deleted = 0;
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getStatus() == HorizonState::Status::bad)
        {
            horizon.erase(horizon.begin() + i);
            num_deleted++;
        }
        if (num_deleted == num)
            break;
    }
}

void planning::rbt::DRGBTConnect::addRandomStates(int num)
{
    std::shared_ptr<base::State> q_rand;
    for (int i = 0; i < num; i++)
    {
        q_rand = ss->randomState(q_current);
        saturateSpine(q_current, q_rand);
        pruneSpine(q_current, q_rand);
        horizon.emplace_back(std::make_shared<HorizonState>(q_rand, -1));
    }
}

void planning::rbt::DRGBTConnect::addLateralStates(int num)
{
    if (ss->getDimensions() == 2)   // In 2D C-space only two possible lateral spines exist
    {
        std::shared_ptr<base::State> q_new;
        Eigen::Vector2f new_vec;
        for (int coord = -1; coord <= 2; coord += 2)
        {
            new_vec << -coord, coord * (q_next->getCoord(0) - q_current->getCoord(0)) / 
                                       (q_next->getCoord(1) - q_current->getCoord(1));
            q_new = ss->newState(q_current->getCoord() + new_vec);
            saturateSpine(q_current, q_new);
            pruneSpine(q_current, q_new);
            horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
        }
        if (num > 2)    // If more than two spines are required, add ('num' - 2) random spines
            addRandomStates(num - 2); 
    }
    else
    {
        for (int k = 0; k < ss->getDimensions(); k++)
        {
            if (q_next->getCoord(k) != q_current->getCoord(k))
            {
                std::shared_ptr<base::State> q_new;
                float coord;
                for (int i = 0; i < num; i++)
                {
                    q_new = ss->randomState(q_current);
                    coord = q_current->getCoord(k) + q_new->getCoord(k) -
                            (q_next->getCoord() - q_current->getCoord()).dot(q_new->getCoord()) /
                            (q_next->getCoord(k) - q_current->getCoord(k));
                    q_new->setCoord(coord, k);
                    saturateSpine(q_current, q_new);
                    pruneSpine(q_current, q_new);
                    horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
                }
                break;
            }                
        }
    }   
}

// Modify state by replacing it with a random state, which is generated using biased distribution,
// i.e., oriented weight around 'q' ('q->getStatus()' == bad), or around '-q' ('q->getStatus()' == critical).
void planning::rbt::DRGBTConnect::modifyState(std::shared_ptr<HorizonState> &q)
{
    std::shared_ptr<base::State> q_new;
    std::shared_ptr<base::State> q_reached = q->getStateReached();
    float norm = (q_reached->getCoord() - q_current->getCoord()).norm();
    Eigen::VectorXf vec = Eigen::VectorXf::Random(ss->getDimensions()) * norm / std::sqrt(ss->getDimensions() - 1);
    vec(0) = (vec(0) > 0) ? 1 : -1;
    vec(0) *= std::sqrt(norm * norm - vec.tail(ss->getDimensions() - 1).squaredNorm());
    if (q->getStatus() == HorizonState::Status::bad)
        q_new = ss->newState(q_reached->getCoord() + vec);
    else if (q->getStatus() == HorizonState::Status::critical)
        q_new = ss->newState(2 * q_current->getCoord() - q_reached->getCoord() + vec);
    
    saturateSpine(q_current, q_new);
    pruneSpine(q_current, q_new);
    q = std::make_shared<HorizonState>(q_new, -1);
}

// Compute reached state when generating a generalized spine from 'q_current' towards 'q'.
void planning::rbt::DRGBTConnect::computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q)
{
    base::State::Status status;
    std::shared_ptr<base::State> q_reached;
    tie(status, q_reached) = extendGenSpineV2(q_current, q->getState());
    q->setStateReached(q_reached);

    float d_c = getDistanceUnderestimation(q_reached, q_current->getPlanes());
    if (q->getDistancePrevious() == -1)
        q->setDistancePrevious(d_c);
    else
        q->setDistancePrevious(q->getDistance());
    
    q->setDistance(d_c);
    
    // Set status of the reached state
    if (q->getIndexInPath() != -1 && ss->isEqual(q_reached, goal))
        q->setStatus(HorizonState::Status::goal);
    else if (d_c < RBTConnectConfig::D_CRIT)
        q->setStatus(HorizonState::Status::critical);
    else if (q->getWeight() == 0)
        q->setStatus(HorizonState::Status::bad);
    else
        q->setStatus(HorizonState::Status::good);
}

// Compute the next state from the horizon
void planning::rbt::DRGBTConnect::computeNextState()
{
    std::vector<float> dist_to_goal(horizon.size());
    float d_goal_min = INFINITY;
    int d_goal_min_idx = -1;
    float d_c_max = 0;
    for (int i = 0; i < horizon.size(); i++)
    {
        dist_to_goal[i] = (goal->getCoord() - horizon[i]->getStateReached()->getCoord()).norm();
        if (dist_to_goal[i] < d_goal_min)
        {
            d_goal_min = dist_to_goal[i];
            d_goal_min_idx = i;
        }
        if (horizon[i]->getDistance() > d_c_max)
            d_c_max = horizon[i]->getDistance();
    }
    d_max_mean = ((planner_info->getNumIterations() - 1) * d_max_mean + d_c_max) / planner_info->getNumIterations();
    
    if (d_goal_min == 0)      // 'goal' lies in the horizon
    {
        d_goal_min = 1e-3;    // Only to avoid "0/0" when 'dist_to_goal[i] == 0'
        dist_to_goal[d_goal_min_idx] = d_goal_min;
    }

    std::vector<float> weights_dist(horizon.size());
    for (int i = 0; i < horizon.size(); i++)
        weights_dist[i] = d_goal_min / dist_to_goal[i];        
    
    float weights_dist_mean = std::accumulate(weights_dist.begin(), weights_dist.end(), 0.0) / horizon.size();
    float max_weight = 0;
    float weight;
    float d_min;
    for (int i = 0; i < horizon.size(); i++)
    {
        if (horizon[i]->getDistance() > RBTConnectConfig::D_CRIT)
        {
            weight = (2 * horizon[i]->getDistance() - horizon[i]->getDistancePrevious()) / d_max_mean 
                     + weights_dist[i] - weights_dist_mean;
            if (weight > 1)
                weight = 1;
            else if (weight < 0)
                weight = 0;
        }
        else
            weight = 0;
        
        if (horizon[i]->getIndexInPath() == -1)   // Weight is halved if state does not belong to the path
            weight /= 2;

        if (weight == 0)
            horizon[i]->setStatus(HorizonState::Status::bad);

        if (weight > max_weight)
        {
            max_weight = weight;
            d_min = dist_to_goal[i];
            q_next = horizon[i];
        }

        horizon[i]->setWeight(weight);
    }

    // The best state nearest to the goal is chosen as the next state
    float hysteresis = 0.1;
    for (int i = 0; i < horizon.size(); i++)
    {
        if (std::abs(max_weight - horizon[i]->getWeight()) < hysteresis && dist_to_goal[i] < d_min)
        {
            d_min = dist_to_goal[i];
            q_next = horizon[i];
        }
    }

    // If weights of 'q_next_previous' and 'q_next' are close, 'q_next_previous' remains as the next state
    if (q_next != q_next_previous &&
        std::abs(q_next->getWeight() - q_next_previous->getWeight()) < hysteresis &&
        q_next->getStatus() != HorizonState::Status::goal)
            q_next = q_next_previous;

    q_next_previous = q_next;
}

bool planning::rbt::DRGBTConnect::whetherToReplan()
{
    float weight_max = 0;
    float weight_sum = 0;
    for (int i = 0; i < horizon.size(); i++)
    {
        weight_max = std::max(weight_max, horizon[i]->getWeight());
        weight_sum += horizon[i]->getWeight();
    }
    return (weight_max < DRGBTConnectConfig::WEIGHT_MIN && 
            weight_sum / horizon.size() < DRGBTConnectConfig::WEIGHT_MEAN_MIN) ? true : false;
}

void planning::rbt::DRGBTConnect::outputPlannerData(std::string filename, bool output_states_and_paths, bool append_output) const
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
		output_file << "Planner type:    " << "DRGBTConnect" << std::endl;
		output_file << "Planner info:\n";
		output_file << "\t Succesfull:           " << (planner_info->getSuccessState() ? "yes" : "no") << std::endl;
		output_file << "\t Number of iterations: " << planner_info->getNumIterations() << std::endl;
		output_file << "\t Number of states:     " << planner_info->getNumStates() << std::endl;
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
			output_file << *trees[0];
			output_file << *trees[1];
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