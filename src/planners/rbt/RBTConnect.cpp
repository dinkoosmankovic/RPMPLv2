//
// Created by nermin on 15.02.22.
//

#include "RBTConnect.h"
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <nanoflann.hpp>
#include <chrono>
#include <fstream>

planning::rbt::RBTConnect::RBTConnect(std::shared_ptr<base::StateSpace> ss_) : RRTConnect(ss_){ }

planning::rbt::RBTConnect::RBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                      std::shared_ptr<base::State> goal_) : RRTConnect(ss_, start_, goal_){ }

planning::rbt::RBTConnect::~RBTConnect()
{
    TREES[0].emptyTree();
    TREES[1].emptyTree();
    path.empty();
}

bool planning::rbt::RBTConnect::solve()
{
	// start the clock
	auto time_start = std::chrono::steady_clock::now();
	// T_start and T_goal are initialized
	std::vector<std::shared_ptr<base::Tree>> trees = {std::make_shared<base::Tree>(TREES[0]),
													  std::make_shared<base::Tree>(TREES[1])};
	int treeIdx = 0;  // Determines the tree index, i.e., which tree is chosen, 0: from q_init; 1: from q_goal
	std::shared_ptr<base::State> q_e, q_near, q_new;
	size_t iter = 1;
	planning::rrt::Status status;

	while (true)
	{
		/* Generating bur */
		q_e = ss->randomState();
		//LOG(INFO) << q_rand->getCoord().transpose();
		q_near = trees[treeIdx]->get_q_near(kdtrees[treeIdx], q_e);
		// LOG(INFO) << "Iteration: " << iter;
		//LOG(INFO) << "Tree: " << trees[treeNum]->getTreeName();
		if (get_d_c(q_near) > d_crit)
		{
			for (int i = 0; i < numSpines; i++)
			{
				q_e = ss->randomState();
				q_e->setCoord(q_e->getCoord() + q_near->getCoord());
				saturateSpine(q_near, q_e);
				pruneSpine(q_near, q_e);
				tie(status, q_new) = extendSpine(q_near, q_e);
				trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new, q_near);
			}
		}
		else	// Distance-to-obstacles is less than d_crit
		{
			tie(status, q_new) = extend(q_near, q_e);
			if (status != planning::rrt::Status::Trapped)
			{
				trees[treeIdx]->upgradeTree(kdtrees[treeIdx], q_new, q_near);
			}
		}

		/* Bur-Connect */
		if (status != planning::rrt::Status::Trapped)
		{
            treeIdx = 1 - treeIdx;	// Swapping trees
			q_near = trees[treeIdx]->get_q_near(kdtrees[treeIdx], q_new);
			status = connectSpine(trees[treeIdx], kdtrees[treeIdx], q_near, q_new);
		}
		else 
		{
			treeIdx = 1 - treeIdx; 	// Swapping trees
		}

		iter++;
		plannerInfo->addIterationTime(getElapsedTime(time_start));
		plannerInfo->setNumNodes(trees[0]->getStates()->size() + trees[1]->getStates()->size());
		if (checkStoppingCondition(status, time_start))
		{
			plannerInfo->setPlanningTime(getElapsedTime(time_start));
			plannerInfo->setNumIterations(iter);
			return status == planning::rrt::Status::Reached ? true : false;
		}
    }
	return false;
}

// Get minimal distance from 'q' (determined with the pointer 'q_p' and 'tree') to obstacles
double planning::rbt::RBTConnect::get_d_c(std::shared_ptr<base::State> q)
{
	double d_c;
	if (q->get_d_c() > 0)
	{
		d_c = q->get_d_c();
	}
	else
	{
		d_c = ss->getDistance(q);
		q->set_d_c(d_c);
	}
	return d_c;
}

void planning::rbt::RBTConnect::saturateSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	double d = (q_e->getCoord() - q->getCoord()).norm();
	if (d > 0)
	{
		q_e->setCoord(q->getCoord() + (q_e->getCoord() - q->getCoord()) * delta / d);
	}
}

// Prune the spine from 'q' to 'q_e', if it comes out C-space domain
void planning::rbt::RBTConnect::pruneSpine(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	int dim = ss->getDimensions();
	Eigen::VectorXf q_temp;
	std::vector<float> bounds(dim);
	std::vector<int> indices;
	std::vector<robots::LinkLimits> limits = ss->robot->getLimits();
	double t;

	for (int k = 0; k < dim; k++)
	{
		if (q_e->getCoord(k) > limits[k][1])
		{
			bounds[k] = limits[k][1];
			indices.push_back(k);
		}
		else if (q_e->getCoord(k) < limits[k][0])
		{
			bounds[k] = limits[k][0];
			indices.push_back(k);
		}
	}
	if (indices.size() == 1)
	{
		t = (bounds[indices[0]] - q->getCoord(indices[0])) / (q_e->getCoord(indices[0]) - q->getCoord(indices[0]));
		q_e->setCoord(q->getCoord() + t * (q_e->getCoord() - q->getCoord()));
	}
	else if (indices.size() > 1)
	{
		for (int k : indices)
		{
			t = (bounds[k] - q->getCoord(k)) / (q_e->getCoord(k) - q->getCoord(k));
			q_temp = q->getCoord() + t * (q_e->getCoord() - q->getCoord());
			if ((q_temp.array() >= limits[k][0] && q_temp.array() <= limits[k][1]).prod())
			{
				q_e->setCoord(q_temp);
				break;
			}
		}
	}
}

// Spine is generated from 'q' towards 'q_e'
// 'q_new' is the new reached node
// 'reached' means whether 'q_e' is reached
std::tuple<planning::rrt::Status, std::shared_ptr<base::State>> planning::rbt::RBTConnect::extendSpine(std::shared_ptr<base::State> q, 
																									   std::shared_ptr<base::State> q_e)
{
	double rho = 0;             // The path length in W-space
	double step;
	int k = 1;
	int K_max = 5;              // The number of iterations for computing q*
	std::shared_ptr<base::State> q_new = ss->randomState(); q_new->makeCopy(q);
	std::vector<KDL::Frame> frames = ss->robot->computeForwardKinematics(q);
	std::vector<KDL::Frame> frames_temp = frames;
	while (true)
	{
		step = computeStep(q_new, q_e, q->get_d_c() - rho, frames_temp);     // 'd_c - rho' is the remaining path length in W-space
		if (step > 1)
		{
			q_new->setCoord(q_e->getCoord());
			return {planning::rrt::Reached, q_new};
		}
		else
		{
			q_new->setCoord(q_new->getCoord() + step * (q_e->getCoord() - q_new->getCoord()));
		}
		if (k == K_max)
		{
			return {planning::rrt::Advanced, q_new};
		}

		frames_temp = ss->robot->computeForwardKinematics(q_new);
		for (int i = 1; i < ss->robot->getParts().size()+1; i++)
		{
			rho = std::max(rho, (frames[i].p - frames_temp[i].p).Norm());
		}
		k += 1;
	}
}

planning::rrt::Status planning::rbt::RBTConnect::connectSpine(std::shared_ptr<base::Tree> tree, std::shared_ptr<KdTree> kdtree, 
														 	  std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e)
{
	std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q);
	planning::rrt::Status status = planning::rrt::Advanced;
	int num_ext = 0;  // TODO: should be read from configuration
	while (status == planning::rrt::Advanced && num_ext++ < 50)
	{
		std::shared_ptr<base::State> q_new;
		if (q_temp->get_d_c() > d_crit)
		{
			tie(status, q_new) = extendSpine(q_temp, q_e);
		}
		else
		{
			tie(status, q_new) = extend(q_temp, q_e);
		}

		if (status != planning::rrt::Trapped)
		{
			tree->upgradeTree(kdtree, q_new, q_temp);
			std::shared_ptr<base::State> q_temp = ss->randomState(); q_temp->makeCopy(q_new);
		}
	}
	// LOG(INFO) << "extended.";
	return status;
}

double planning::rbt::RBTConnect::computeStep(std::shared_ptr<base::State> q, std::shared_ptr<base::State> q_e, double fi, 
											  std::vector<KDL::Frame> &frames){
	double d = 0;
	double r, r_temp;
	// Maybe add robot name ??
	if (ss->getDimensions() == 2)	// Assumes that number of links = number of DOFs
	{
		for (int i = 0; i < ss->robot->getParts().size(); i++)
		{
			r = (frames[i+1].p - frames[i].p).Norm(); 	// i-th segment length. Any better way to get this?? 
			for (int k = i+1; k < ss->robot->getParts().size(); k++)
			{
				r_temp = (frames[k+1].p - frames[i].p).Norm();
				r = std::max(r, r_temp);
			}
			d += r * std::abs(q_e->getCoord(i) - q->getCoord(i));
		}
	}
	else if (ss->getDimensions() == 6) 	// TODO
	{

	}
	return fi / d;
}

void planning::rbt::RBTConnect::outputPlannerData(std::string filename) const
{
	std::ofstream outputFile;
	outputFile.open(filename);
	if (outputFile.is_open())
	{
		outputFile << "Space Type: " << ss->getStateSpaceType() << std::endl;
		outputFile << "Space dimension: " << ss->getDimensions() << std::endl;
		outputFile << "Planner type:\t" << "RBTConnect" << std::endl;
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

