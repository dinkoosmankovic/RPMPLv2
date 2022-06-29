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
    auto time_iter_start = time_alg_start;                      // Start the iteration clock
    auto time_current = time_alg_start;
    q_current = start;
    q_next = std::make_shared<HorizonState>(q_current, 0);
    q_next_previous = q_next;
    path.emplace_back(start);                                   // State 'start' is added to the realized path
    bool replanning = false;                                    // Whether path replanning is required
    base::State::Status status;
    std::shared_ptr<base::State> q_new;
    num_lateral_spines = 2 * ss->getDimensions() - 2;
    int idx;                                                    // Designates from which state in predefined path new states are added to the horizon
    int num_states = DRGBTConnectConfig::INIT_HORIZON_SIZE + num_lateral_spines;     // Number of states that is required to be in the horizon
    planner_info->setNumIterations(0);
    planner_info->setNumStates(1);

    // Obtaining the inital path
    std::unique_ptr<planning::AbstractPlanner> RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
    RGBTConnect_planner->solve();
    std::vector<std::shared_ptr<base::State>> predefined_path = RGBTConnect_planner->getPath();
    LOG(INFO) << "Predefined path is" << (predefined_path.empty() ? " empty! " : ":");
    for (int i = 0; i < predefined_path.size(); i++)
        std::cout << predefined_path.at(i) << std::endl;
	
    while (true)
    {
        // Only states from predefined path that come after 'q_next' are remained in the horizon. Other states are deleted.
        LOG(INFO) << "Preparing horizon";
        idx = q_next->getIndex() + 1;
        if (status == base::State::Status::Reached)
        {
            for (int i = num_states - 1; i >= 0; i--)
            {
                if (horizon[i]->getIndex() < idx)
                {
                    horizon.erase(horizon.begin() + i);
                    LOG(INFO) << "Deleting state " << i;
                }
            }
            if (!horizon.empty())
                idx = horizon.back()->getIndex() + 1;
        }
        else
        {
            LOG(INFO) << "Deleting all states ";
            horizon.clear();
        }

        // Generating horizon
        LOG(INFO) << "Adding states from the predefined path starting from " << idx << ". state ";
        num_states -= (horizon.size() + num_lateral_spines);
        if (horizon.empty() && idx == 0)
            replanning = true;
        else if (idx + num_states <= predefined_path.size())
        {
            for (int i = idx; i < idx + num_states; i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path[i], i));
        }
        else if (idx < predefined_path.size())
        {
            for (int i = idx; i < predefined_path.size(); i++)
                horizon.emplace_back(std::make_shared<HorizonState>(predefined_path[i], i));
            horizon.back()->setStatus(HorizonState::Status::goal);
            replanning = false;
        }
        
        // Set the next state
        if (horizon.empty())
            addRandomStates(1);
        q_next = horizon.front();

        // Moving from 'q_current' towards 'q_next' for step 'DRGBTConnectConfig::STEP', where 'q_next' may change
        do
        {
            LOG(INFO) << "\n\nIteration num. " << planner_info->getNumIterations()
                      << " -----------------------------------------------------------------------------------------";

            // Update obstacles and check if the collision occurs
            LOG(INFO) << "Updating obstacles";
            ss->env->updateObstacles();
            if (!ss->isValid(q_current))
            {
                LOG(INFO) << "Collision has been occured! ";
                planner_info->setSuccessState(false);
                return false;
            }
            
            computeHorizon();

            // Update the robot current state
            // Check validity of the motion from 'q_current' to 'q_new'
            tie(status, q_new) = ss->interpolate(q_current, q_next->getState(), DRGBTConnectConfig::STEP);
            if (!q_next->getIsReached() && 
                (q_next->getStateReached()->getCoord() - q_current->getCoord()).norm() < DRGBTConnectConfig::STEP && 
                !ss->isValid(q_current, q_new))
            {
                status = base::State::Status::Trapped;
                replanning = true;
            }
            else
            {
                q_new->setParent(q_current);
                q_current = q_new;
            }
            path.emplace_back(q_current);
            LOG(INFO) << "Updating the robot current state to: " << q_current->getCoord().transpose();
            LOG(INFO) << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
                                    << (status == base::State::Status::Trapped ? "Trapped" : "")
                                    << (status == base::State::Status::Reached ? "Reached" : "");
            
            // Replanning procedure assessment
            if (replanning || whetherToReplan())
            {
                try
                {
                    time_current = std::chrono::steady_clock::now();
                    RRTConnectConfig::MAX_PLANNING_TIME = getElapsedTime(time_iter_start, time_current) - 1; // 1 [ms] is reserved for the following code lines
                    RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, q_current, goal);
                    if (RGBTConnect_planner->solve())   // New path is found, thus update path to the goal
                    {
                        LOG(INFO) << "The path has been replanned in " << RGBTConnect_planner->getPlannerInfo()->getPlanningTime() << " [ms]. ";
                        LOG(INFO) << "Predefined path is: ";
                        predefined_path = RGBTConnect_planner->getPath();
                        for (int i = 0; i < predefined_path.size(); i++)
                            std::cout << predefined_path.at(i) << std::endl;
                        replanning = false;
                        status = base::State::Status::Reached;
                        q_next = std::make_shared<HorizonState>(predefined_path.front(), 0);
                    }
                    else    // New path is not found
                        throw std::runtime_error("New path is not found! ");
                }
                catch (std::exception &e)
                {
                    LOG(INFO) << "Replanning is required. " << e.what();
                    replanning = true;
                }
            }
            else
                LOG(INFO) << "Replanning is not required! ";

            // Checking the real-time execution
            time_current = std::chrono::steady_clock::now();
            float time_iter_current = getElapsedTime(time_iter_start, time_current);
            float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - time_iter_current;
            LOG(INFO) << "Remaining iteration time is " << time_iter_remain << " [ms]. ";
            if (time_iter_remain < 0)
                LOG(INFO) << "Real-time is broken!!!";
            time_iter_start = time_current;

		    // Planner info and terminating condition
            planner_info->setNumIterations(planner_info->getNumIterations() + 1);
            planner_info->addIterationTime(getElapsedTime(time_alg_start, time_current));
            if (checkTerminatingCondition())
            {
                planner_info->setPlanningTime(planner_info->getIterationsTimes().back());
                return planner_info->getSuccessState();
            }
            
        } while (status == base::State::Status::Advanced);

        num_states = horizon.size();
        LOG(INFO) << "Next state is " << (status == base::State::Status::Reached ? "reached" : "not reached") 
            << "\n***************************************************************************";
    }
}

// Compute and update the horizon such that it contains possibly better states 'states'.
// Bad states will be replaced with "better" states.
// Also, next state is computed.
void planning::rbt::DRGBTConnect::computeHorizon()
{
    float d_c = computeDistance(q_current);
    int new_horizon_size = std::min((int) std::floor(DRGBTConnectConfig::INIT_HORIZON_SIZE * (1 + RBTConnectConfig::D_CRIT / d_c)),
                                    5 * ss->getDimensions() * DRGBTConnectConfig::INIT_HORIZON_SIZE);
    
    LOG(INFO) << "Modifying horizon size from " << horizon.size() << " to " << new_horizon_size;
    if (new_horizon_size < horizon.size())
        shortenHorizon(horizon.size() - new_horizon_size);
    else if (new_horizon_size > horizon.size())    // If 'new_horizon_size' has increased, or little states exist, then random states are added
        addRandomStates(new_horizon_size - horizon.size());

    // Lateral states are added
    LOG(INFO) << "Adding " << num_lateral_spines << " lateral states";
    addLateralStates(num_lateral_spines);

    LOG(INFO) << "Computing reached states ";
    for (int i = 0; i < horizon.size(); i++)
    {
        computeReachedState(q_current, horizon[i]);
        LOG(INFO) << i << ". state:\n" << horizon[i];

        // Bad and critical states are modified
        if (horizon[i]->getStatus() == HorizonState::Status::bad || 
            horizon[i]->getStatus() == HorizonState::Status::critical)
                modifyState(horizon[i]);
    }

    computeNextState();

    LOG(INFO) << "Horizon states are: ";
    for (int i = 0; i < horizon.size(); i++)
        LOG(INFO) << i << ". state:\n" << horizon[i]; 
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
            LOG(INFO) << "Deleting state " << i << " in stage 1";
        }
        if (num_deleted == num)
            return;
    }
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        if (horizon[i]->getIndex() == -1)
        {
            horizon.erase(horizon.begin() + i);
            num_deleted++;
            LOG(INFO) << "Deleting state " << i << " in stage 2";
        }
        if (num_deleted == num)
            return;
    }
    for (int i = horizon.size() - 1; i >= 0; i--)
    {
        horizon.erase(horizon.begin() + i);
        num_deleted++;
            LOG(INFO) << "Deleting state " << i << " in stage 3";
        if (num_deleted == num)
            return;
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
        LOG(INFO) << "Adding random state: " << horizon.back()->getCoord().transpose();
    }
}

void planning::rbt::DRGBTConnect::addLateralStates(int num)
{
    if (ss->getDimensions() == 2)   // In 2D C-space only two possible lateral spines exist
    {
        std::shared_ptr<base::State> q_new;
        Eigen::Vector2f new_vec;
        for (int coord = -1; coord <= 1; coord += 2)
        {
            new_vec(0) = -coord; 
            new_vec(1) = coord * (q_next->getCoord(0) - q_current->getCoord(0)) / (q_next->getCoord(1) - q_current->getCoord(1));
            q_new = ss->newState(q_current->getCoord() + new_vec);
            saturateSpine(q_current, q_new);
            pruneSpine(q_current, q_new);
            horizon.emplace_back(std::make_shared<HorizonState>(q_new, -1));
            // LOG(INFO) << "Adding lateral state: " << horizon.back()->getCoord().transpose();
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
                    LOG(INFO) << "Adding lateral state: " << horizon.back()->getCoord().transpose();
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

    std::shared_ptr<HorizonState> q_new_ = std::make_shared<HorizonState>(q_new, -1);
    computeReachedState(q_current, q_new_);
    if (q_new_->getDistance() > q->getDistance())
        q = q_new_;
    LOG(INFO) << "Modifying with state:\n" << q;
}

// Compute reached state when generating a generalized spine from 'q_current' towards 'q'.
void planning::rbt::DRGBTConnect::computeReachedState(std::shared_ptr<base::State> q_current, std::shared_ptr<HorizonState> q)
{
    base::State::Status status;
    std::shared_ptr<base::State> q_reached;
    tie(status, q_reached) = extendGenSpineV2(q_current, q->getState());
    q->setStateReached(q_reached);
    q->setIsReached(status == base::State::Status::Reached ? true : false);

    float d_c = computeDistanceUnderestimation(q_reached, q_current->getPlanes());
    if (q->getDistancePrevious() == -1)
        q->setDistancePrevious(d_c);
    else
        q->setDistancePrevious(q->getDistance());
    
    q->setDistance(d_c);
    
    // Set status of the reached state
    if (q->getIndex() > 0 && ss->isEqual(q_reached, goal))
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
    d_max_mean = (planner_info->getNumIterations() * d_max_mean + d_c_max) / (planner_info->getNumIterations() + 1);
    
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
        
        if (horizon[i]->getIndex() == -1)   // Weight is halved if state does not belong to the path
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
    for (int i = 0; i < horizon.size(); i++)
    {
        if (std::abs(max_weight - horizon[i]->getWeight()) < hysteresis && dist_to_goal[i] < d_min)
        {
            d_min = dist_to_goal[i];
            q_next = horizon[i];
        }
    }

    // If weights of 'q_next_previous' and 'q_next' are close, 'q_next_previous' remains the next state
    if (q_next != q_next_previous &&
        std::abs(q_next->getWeight() - q_next_previous->getWeight()) < hysteresis &&
        q_next->getStatus() != HorizonState::Status::goal &&
        getIndexInHorizon(q_next_previous) != -1)
            q_next = q_next_previous;

    LOG(INFO) << "Setting the robot next state to: " << q_next->getCoord().transpose();
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

// Return index in the horizon of state 'q'. If 'q' is not in the horizon, -1 is returned.
int planning::rbt::DRGBTConnect::getIndexInHorizon(std::shared_ptr<HorizonState> q)
{
    for (int idx = 0; idx < horizon.size(); idx++)
    {
        if (q == horizon[idx])
            return idx;
    }
    return -1;   
}

bool planning::rbt::DRGBTConnect::checkTerminatingCondition()
{
    if (ss->isEqual(q_current, goal))
    {
        LOG(INFO) << "Goal configuration has been successfully reached! ";
		planner_info->setSuccessState(true);
        return true;
    }
	else if (planner_info->getIterationsTimes().back() >= DRGBTConnectConfig::MAX_PLANNING_TIME)
	{
        LOG(INFO) << "Maximal planning time has been reached! ";
		planner_info->setSuccessState(false);
		return true;
	}
    else if (planner_info->getNumIterations() >= DRGBTConnectConfig::MAX_NUM_ITER)
	{
        LOG(INFO) << "Maximal number of iterations has been reached! ";
		planner_info->setSuccessState(false);
		return true;
	}

	return false;
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
		output_file << "\t Planning time [ms]:   " << planner_info->getPlanningTime() << std::endl;
		if (output_states_and_paths)
		{
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

planning::rbt::DRGBTConnect::HorizonState::HorizonState(std::shared_ptr<base::State> state_, int index_)
{
    state = state_;
    index = index_;
}

std::ostream &planning::rbt::operator<<(std::ostream &os, const planning::rbt::DRGBTConnect::HorizonState *q)
{
    os << "q:         (" << q->getCoord().transpose() << ")" << std::endl;
    if (q->getStateReached() == nullptr)
        os << "q_reached: NONE " << std::endl;
    else
        os << "q_reached: (" << q->getStateReached()->getCoord().transpose() << ")" << std::endl;
    os << "status:     " << q->getStatus() << std::endl;
    os << "idx path:   " << q->getIndex() << std::endl;
    os << "d_c:        " << q->getDistance() << std::endl;
    os << "weight:     " << q->getWeight() << std::endl;
    return os;
}
