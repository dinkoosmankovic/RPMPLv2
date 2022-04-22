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
    Horizon::size = DRGBTConnectConfig::INIT_HORIZON_SIZE;
    Horizon::q_current = start;
    Horizon::d_c_previous = std::vector<float>(Horizon::size, 0);
    Horizon::d_max_mean = 0;
    Horizon::idx_next = 0;
    bool replanning;                                                    // Whether path replanning is required
    path.emplace_back(start);                                           // State 'start' is added to the realized path
    base::StateSpace::Status status;
    planner_info->setNumIterations(0);
    planner_info->setNumStates(1);

    // Getting the inital path
    std::unique_ptr<planning::AbstractPlanner> RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
    RGBTConnect_planner->solve();
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> predefined_path = 
        std::make_shared<std::vector<std::shared_ptr<base::State>>>(RGBTConnect_planner->getPath());
    int idx = 0;    // Index of next state from 'predefined_path'
	
    while (true)
    {
        auto time_iter_start = std::chrono::steady_clock::now();     // Start the iteration clock
        replanning = false;
        idx += Horizon::idx_next;
        if (idx + Horizon::size <= predefined_path->size())
        {
            for (int i = idx; i < idx + Horizon::size; i++)
                Horizon::states.emplace_back(predefined_path->at(i));
        }
        else if (idx < predefined_path->size())
        {
            for (int i = idx; i < Horizon::size; i++)
                Horizon::states.emplace_back(predefined_path->at(i));
        }
        else
        {
            idx = predefined_path->size();
            if (predefined_path->empty())   // If the initial path was not found, random states are added
                addRandomSpines(Horizon::size);
            else                            // At least 'goal' remains in the horizon
                Horizon::states.emplace_back(predefined_path->back());
        }

        Horizon::states_good = Horizon::states;
        Horizon::states_bad = {};
        Horizon::q_next = Horizon::states[0];
        updateHorizon();
        Horizon::q_next = getNextState();

        if (idx == predefined_path->size() && !ss->isEqual(Horizon::q_next, goal))
            replanning = true;

        status = base::StateSpace::Status::Advanced;
        while (status == base::StateSpace::Status::Advanced)
        {
            ss->env->updateObstacles();
            planner_info->addIterationTime(getElapsedTime(time_iter_start));
            time_iter_start = std::chrono::steady_clock::now();

            tie(status, Horizon::q_current) = ss->interpolate(Horizon::q_current, Horizon::q_next, RRTConnectConfig::EPS_STEP);
            path.emplace_back(Horizon::q_current);
            if (!ss->isValid(Horizon::q_current))
            {
                LOG(INFO) << "Collision occured! ";
                return false;
            }

            if (ss->isEqual(Horizon::q_current, goal))
            {
                LOG(INFO) << "Goal configuration has been successfully reached! ";
                return true;
            }
            
            Horizon::d_c_previous = Horizon::d_c;
            updateHorizon();
            if (Horizon::d_c.size() > Horizon::d_c_previous.size())
            {
                for (int i = Horizon::d_c_previous.size(); i < Horizon::d_c.size(); i++)
                    Horizon::d_c_previous.emplace_back(Horizon::d_c[i]);                
            }
            else if (Horizon::d_c.size() < Horizon::d_c_previous.size())
                Horizon::d_c_previous.resize(Horizon::d_c.size());

            Horizon::q_next = getNextState();
            
            // Replanning procedure assessment
            if (whetherToReplan() || replanning)
            {
                Horizon::idx_next = 0;
                RGBTConnect_planner = std::make_unique<planning::rbt::RGBTConnect>(ss, Horizon::q_current, goal);
                RGBTConnect_planner->solve();

                if (!RGBTConnect_planner->getPath().empty())   // New path is found, thus update path to the goal
                {
                    LOG(INFO) << "The path has been replanned. ";
                    replanning = false;
                    status = base::StateSpace::Status::Reached;
                    predefined_path = std::make_shared<std::vector<std::shared_ptr<base::State>>>(RGBTConnect_planner->getPath());
                    idx = 0;
                }
                else    // New path is not found
                {
                    LOG(INFO) << "New path is not found. ";
                    replanning = true;
                    Horizon::states_good.clear();
                    Horizon::states_bad.clear();
                    for (int i = 0; i < Horizon::size; i++)
                    {
                        if (Horizon::weights[i] == 0)   // It is a bad state
                            Horizon::states_bad.emplace_back(Horizon::states[i]);
                        else
                            Horizon::states_good.emplace_back(Horizon::states[i]);
                    }
                }
            }

            // Real-time checking
            float time_iter_remain = DRGBTConnectConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start);
            LOG(INFO) << "Remaining iteration time [ms]: " << time_iter_remain;
            if (time_iter_remain < 0)
                LOG(INFO) << "Real-time is broken !!!";
        }
        

    }
    

}

// Update the horizon such that it contains possibly better states ('states')
// 'states_good' contains good states in the beginning, but later it contains all states from the updated horizon
// 'states_bad' are bad states that will be replaced with "better" states
// 'time_max' is the maximal runtime for this function
void planning::rbt::DRGBTConnect::updateHorizon()
{
    float d_c = getDistance(Horizon::q_current);
    Horizon::size = std::min((int) std::floor(DRGBTConnectConfig::INIT_HORIZON_SIZE * (1 + RBTConnectConfig::D_CRIT / d_c)),
                             5 * ss->getDimensions() * DRGBTConnectConfig::INIT_HORIZON_SIZE);
    
    Horizon::states_not_in_path.clear();
    Horizon::states = Horizon::states_good;
    if (Horizon::states_good.size() > Horizon::size)    // Surplus states are deleted. Best states holds priority.
        Horizon::states.resize(Horizon::size);
    else if (Horizon::states_good.size() + Horizon::states_bad.size() > Horizon::size)
        Horizon::states_bad.resize(Horizon::size - Horizon::states_good.size());
    else    // If 'Horizon::size' has increased, or little states exist, then random/lateral states are added
        addRandomSpines(Horizon::size - Horizon::states_good.size() - Horizon::states_bad.size());

    // Bad states are modified
    if (Horizon::states_bad.size() > 0)
        addWeightedSpines(Horizon::states_bad.size(), Horizon::states_bad, true);

    // Lateral spines are added
    float num_lateral_spines = 2 * ss->getDimensions() - 2;
    Horizon::size += num_lateral_spines;
    addLateralSpines(num_lateral_spines);

    int num = -1;    // Number of added spines
    while (++num < Horizon::size)
    {
        
    }


    Horizon::states_good.clear();
    Horizon::states_bad.clear();

    
}

void planning::rbt::DRGBTConnect::addRandomSpines(int num)
{
    std::shared_ptr<base::State> q_rand;
    for (int i = 0; i < num; i++)
    {
        q_rand = ss->randomState(Horizon::q_current);
        saturateSpine(Horizon::q_current, q_rand);
        pruneSpine(Horizon::q_current, q_rand);
        Horizon::states.emplace_back(q_rand);
        Horizon::states_not_in_path.emplace_back(q_rand);
    }
}

void planning::rbt::DRGBTConnect::addLateralSpines(int num)
{
    if (ss->getDimensions() == 2)   // In 2D C-space only two possible lateral spines exist
    {
        std::shared_ptr<base::State> q_new;
        Eigen::Vector2f new_vec;
        for (int coord = -1; coord <= 2; coord += 2)
        {
            new_vec << -coord, coord * (Horizon::q_next->getCoord(0) - Horizon::q_current->getCoord(0)) / 
                                       (Horizon::q_next->getCoord(1) - Horizon::q_current->getCoord(1));
            q_new = ss->newState(Horizon::q_current->getCoord() + new_vec);
            saturateSpine(Horizon::q_current, q_new);
            pruneSpine(Horizon::q_current, q_new);
            Horizon::states.emplace_back(q_new);
            Horizon::states_not_in_path.emplace_back(q_new);
        }
        if (num > 2)    // If more than two spines are required, add ('num' - 2) random spines
            addRandomSpines(num - 2); 
    }
    else
    {
        for (int idx = 0; idx < ss->getDimensions(); idx++)
        {
            if (Horizon::q_next->getCoord(idx) != Horizon::q_current->getCoord(idx))
            {
                std::shared_ptr<base::State> q_new;
                float coord;
                for (int i = 0; i < num; i++)
                {
                    q_new = ss->randomState(Horizon::q_current);
                    coord = Horizon::q_current->getCoord(idx) + q_new->getCoord(idx) -
                            (Horizon::q_next->getCoord() - Horizon::q_current->getCoord()).dot(q_new->getCoord()) /
                            (Horizon::q_next->getCoord(idx) - Horizon::q_current->getCoord(idx));
                    q_new->setCoord(coord, idx);
                    saturateSpine(Horizon::q_current, q_new);
                    pruneSpine(Horizon::q_current, q_new);
                    Horizon::states.emplace_back(q_new);
                    Horizon::states_not_in_path.emplace_back(q_new);
                }
                break;
            }                
        }
    }
    
}

// Add 'num' random states with oriented weight around states 'Q' ('orientation' = true) or around '-Q' ('orientation' = false)
void planning::rbt::DRGBTConnect::addWeightedSpines(int num, std::vector<std::shared_ptr<base::State>> &Q, bool orientation)
{
    float a_norm;
    Eigen::VectorXf vec;
    std::shared_ptr<base::State> q_new;
    for (int i = 0; i < num; i++)
    {
        a_norm = (Q[i]->getCoord() - Horizon::q_current->getCoord()).norm();
        vec = Eigen::VectorXf::Random(ss->getDimensions()) * a_norm / std::sqrt(ss->getDimensions() - 1);
        vec(0) = (vec(0) > 0) ? 1 : -1;
        vec(0) *= std::sqrt(a_norm * a_norm - vec.tail(ss->getDimensions() - 1).squaredNorm());
        if (orientation)
            q_new = ss->newState(Q[i]->getCoord() + vec);
        else
            q_new = ss->newState(2 * Horizon::q_current->getCoord() - Q[i]->getCoord() + vec);
        
        saturateSpine(Horizon::q_current, q_new);
        pruneSpine(Horizon::q_current, q_new);
        Horizon::states.emplace_back(q_new);
        Horizon::states_not_in_path.emplace_back(q_new);
    }
}

std::shared_ptr<base::State> planning::rbt::DRGBTConnect::getNextState(int idx_previous)
{

    Horizon::idx_previous = Horizon::idx_next;
}

bool planning::rbt::DRGBTConnect::whetherToReplan()
{
    float weight_max = 0;
    float weight_sum = 0;
    for (int i = 0; i < Horizon::size; i++)
    {
        weight_max = std::max(weight_max, Horizon::weights[i]);
        weight_sum += Horizon::weights[i];
    }
    return (weight_max < DRGBTConnectConfig::WEIGHT_MIN && 
            weight_sum / Horizon::size < DRGBTConnectConfig::WEIGHT_MEAN_MIN) ? true : false;
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