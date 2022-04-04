//
// Created by nermin on 19.02.22.
//

#include "RGBTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include "ConfigurationReader.h"
#include <chrono>
#include <fstream>

planning::rbt::RGBTConnect::RGBTConnect(std::shared_ptr<base::StateSpace> ss_) : RBTConnect(ss_){ }

planning::rbt::RGBTConnect::RGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                        std::shared_ptr<base::State> goal_) : RBTConnect(ss_, start_, goal_){ }

planning::rbt::RGBTConnect::~RGBTConnect()
{
    TREES[0].clearTree();
    TREES[1].clearTree();
    path.clear();
}

bool planning::rbt::RGBTConnect::solve()
{
    // start the clock
	auto time_start = std::chrono::steady_clock::now();
	// T_start and T_goal are initialized
	std::vector<std::shared_ptr<base::Tree>> trees = {std::make_shared<base::Tree>(TREES[0]),
													  std::make_shared<base::Tree>(TREES[1])};
	int treeIdx = 0;  // Determines the tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal
	std::shared_ptr<base::State> q_e, q_near, q_new;
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	planning::rrt::Status status;
	plannerInfo->setNumIterations(0);
    plannerInfo->setNumStates(2);

	while (true)
	{
		/* Generating generalized bur */
		// LOG(INFO) << "Iteration: " << plannerInfo->getNumIterations();
		LOG(INFO) << "Num states: " << plannerInfo->getNumStates();
		q_e = ss->randomState();
		// LOG(INFO) << q_rand->getCoord().transpose();
		q_near = trees[treeIdx]->getNearestState(kdtrees[treeIdx], q_e);
		// LOG(INFO) << "Tree: " << trees[treeNum]->getTreeName();
		if (getDistance(q_near) > RBTConnectConfig::D_CRIT)
		{
			for (int i = 0; i < RBTConnectConfig::NUM_SPINES; i++)
			{
				q_e = ss->randomState();
				q_e->setCoord(q_e->getCoord() + q_near->getCoord());
				saturateSpine(q_near, q_e);
				pruneSpine(q_near, q_e);
				tie(status, q_new_list) = extendGenSpine(q_near, q_e);
                trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new_list->front(), q_near);
                for (int j = 1; j < q_new_list->size(); j++)
                {
				    trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new_list->at(j), q_new_list->at(j-1));
                }
			}
            q_new = q_new_list->back();
		}
		else	// Distance-to-obstacles is less than d_crit
		{
			tie(status, q_new) = extend(q_near, q_e);
			if (status != planning::rrt::Status::Trapped)
			{
				trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new, q_near);
			}
		}
		treeIdx = 1 - treeIdx; 	// Swapping trees

		/* Bur-Connect */
		if (status != planning::rrt::Status::Trapped)
		{
			q_near = trees[treeIdx]->getNearestState(kdtrees[treeIdx], q_new);
			status = connectGenSpine(trees[treeIdx], kdtrees[treeIdx], q_near, q_new);
		}

	 	plannerInfo->setNumIterations(plannerInfo->getNumIterations() + 1);
		plannerInfo->addIterationTime(getElapsedTime(time_start));
		plannerInfo->setNumStates(trees[0]->getStates()->size() + trees[1]->getStates()->size());
		if (checkStoppingCondition(status, time_start))
		{
			plannerInfo->setPlanningTime(getElapsedTime(time_start));
			return status == planning::rrt::Status::Reached ? true : false;
		}
    }
}

// Generalized spine is generated from 'q' towards 'q_e'
// 'q_new_list' contains all states from the generalized spine
std::tuple<planning::rrt::Status, std::shared_ptr<std::vector<std::shared_ptr<base::State>>>> 
    planning::rbt::RGBTConnect::extendGenSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
    float d_c = getDistance(q);
	std::shared_ptr<base::State> q_new = q;
	std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
    planning::rrt::Status status;
    for (int i = 0; i < RGBTConnectConfig::NUM_LAYERS; i++)
    {
        std::shared_ptr<base::State> q_temp = ss->newState(q_new);
        tie(status, q_new) = extendSpine(q_temp, q_e, d_c);
		q_new_list->emplace_back(q_new);
        d_c = getDistanceUnderestimation(q_new, q->getPlanes());
		// d_c = getDistance(q_new); 	// If you want to use real distance
        if (d_c < RBTConnectConfig::D_CRIT || status == planning::rrt::Reached)
            break;
    }
    return {status, q_new_list};
}

planning::rrt::Status planning::rbt::RGBTConnect::connectGenSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
														 	      std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
    float d_c = getDistance(q);
	std::shared_ptr<base::State> q_new = q;
    std::shared_ptr<std::vector<std::shared_ptr<base::State>>> q_new_list;
	planning::rrt::Status status = planning::rrt::Advanced;
	int num_ext = 0;
	while (status == planning::rrt::Advanced && num_ext++ < RRTConnectConfig::MAX_EXTENSION_STEPS)
	{
		std::shared_ptr<base::State> q_temp = ss->newState(q_new);
		if (d_c > RBTConnectConfig::D_CRIT)
		{
			tie(status, q_new_list) = extendGenSpine(q_temp, q_e);
            tree->upgradeTree(kdtree, q_new_list->front(), q_temp);
            for (int i = 1; i < q_new_list->size(); i++)
            {
                tree->upgradeTree(kdtree, q_new_list->at(i), q_new_list->at(i-1));
            }
			q_new = q_new_list->back();
            d_c = getDistance(q_new);
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
            if (status != planning::rrt::Trapped)
            {
                tree->upgradeTree(kdtree, q_new, q_temp);
            }
		}	
	}
	return status;
}

// Get minimal distance from 'q' to obstacles
// Also set corresponding 'planes' (which are approximating the obstacles) for the configuation 'q'
float planning::rbt::RGBTConnect::getDistance(std::shared_ptr<base::State> q)
{
    float d_c;
	if (q->getDistance() > 0)
		d_c = q->getDistance();
	else
	{
    	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes;
		tie(d_c, planes) = ss->getDistanceAndPlanes(q);
		q->setDistance(d_c);
        q->setPlanes(planes);
	}
	return d_c;
}

// Returns the underestimation of distance-to-obstacles 'd_c', i.e. returns the distance-to-planes
float planning::rbt::RGBTConnect::getDistanceUnderestimation(std::shared_ptr<base::State> q, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes)
{
    float d_c = INFINITY;
    Eigen::Vector3f M, MN;    // planes << M, MN; where MN = N - M, where N is robot nearest point, and M is obstacle nearest point
	std::shared_ptr<Eigen::MatrixXf> XYZ = ss->robot->computeXYZ(q);
    
    for (int i = 0; i < ss->robot->getParts().size(); i++)
    {
        for (int j = 0; j < ss->env->getParts().size(); j++)
        {
            M << planes->at(j).col(i).head(3);
            MN << planes->at(j).col(i).tail(3);
            d_c = std::min(d_c, std::min(std::abs(MN.dot(XYZ->col(i) - M)) / MN.norm(), 
									 	 std::abs(MN.dot(XYZ->col(i+1) - M)) / MN.norm()) - ss->robot->getRadius(i));
			// std::cout << "(i, j) = (" <<i<<", "<<j<<") " << std::endl;
			// std::cout << "link NP = " << (MN + M).transpose() << std::endl;
			// std::cout << "obs NP  = " << M.transpose() << std::endl;
			// std::cout << "MN = " << MN.norm() - ss->robot->getRadius(i) << std::endl;
			// std::cout << ".................................." << std::endl;
        }
    }
    return d_c;
}

void planning::rbt::RGBTConnect::outputPlannerData(std::string filename) const
{
	std::ofstream outputFile;
	outputFile.open(filename);
	if (outputFile.is_open())
	{
		outputFile << "Space Type: " << ss->getStateSpaceType() << std::endl;
		outputFile << "Space dimension: " << ss->getDimensions() << std::endl;
		outputFile << "Planner type:\t" << "RGBTConnect" << std::endl;
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