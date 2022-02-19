//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include <time.h>

#include <fcl/distance.h>
#include <fcl/collision.h>

#include <glog/logging.h>

base::RealVectorSpaceFCL::~RealVectorSpaceFCL()
{
}

base::RealVectorSpaceFCL::RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
											 const std::shared_ptr<env::Environment> env_) : RealVectorSpace(dimensions_)
{
	srand((unsigned int) time(0));
	setStateSpaceType(StateSpaceType::RealVectorSpaceFCL);
	robot = robot_;
	env = env_;
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	//LOG(INFO) << "robot  parts: " << robot->getParts().size();
	//LOG(INFO) << "env  parts: "<< env->getParts().size() << "\n";
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			fcl::CollisionRequest request;
			fcl::CollisionResult result;
			fcl::collide(robot->getParts()[i].get(), env->getParts()[j].get(), request, result);
			if (result.isCollision() )
				return false;
		}
	}
	return true;
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	int numChecks = 10;
	double D = (q2->getCoord() - q1->getCoord()).norm();
	for (double t = 1./numChecks; t <= 1; t += 1./numChecks)
	{
		std::shared_ptr<base::State> q_t = interpolate(q1, q2, t, D);
		/* if (q_t != nullptr) 
		{
			LOG(INFO) << "checking " << t << " : " << q_t->getCoord().transpose();
			LOG(INFO) << "check: " << isValid(q_t);
		} */
		if (q_t == nullptr)
			return false;
	}
	return true;
}

std::shared_ptr<base::State> base::RealVectorSpaceFCL::randomState()
{
	std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(dimensions);
	Eigen::VectorXf rand = Eigen::VectorXf::Random(dimensions).normalized();
	for (size_t i = 0; i < dimensions; ++i)
	{
		float llimit = robot->getLimits()[i][0];
		float ulimit = robot->getLimits()[i][1];

		rand[i] = ((ulimit - llimit) * rand[i] + llimit + ulimit) / 2;
	}
	//LOG(INFO) << "random coord: " << rand.transpose();
	state->setCoord(rand);
	return state;
}

Eigen::VectorXf normalizedAngles(Eigen::VectorXf r)
{
	
}

double base::RealVectorSpaceFCL::getDistance(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	double min_dist = INFINITY;
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_;
		robot->getParts()[i]->computeAABB();
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			fcl::DistanceRequest request;
			fcl::DistanceResult result;
			fcl::distance(robot->getParts()[i].get(), env->getParts()[j].get(), request, result);
			//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_ << "\t" << result.min_distance;
			min_dist = std::min(min_dist, (double) result.min_distance);
		}
	}
	return min_dist;
}

std::tuple<double, std::shared_ptr<std::vector<Eigen::MatrixXd>>> base::RealVectorSpaceFCL::getDistanceAndPlanes
	(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	double min_dist = INFINITY;
	std::shared_ptr<std::vector<Eigen::MatrixXd>> planes = std::make_shared<std::vector<Eigen::MatrixXd>>
		(std::vector<Eigen::MatrixXd>(env->getParts().size(), Eigen::MatrixXd(6, robot->getParts().size())));
	
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_;
		robot->getParts()[i]->computeAABB();
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			fcl::DistanceRequest request(true);
			fcl::DistanceResult result;
			fcl::distance(robot->getParts()[i].get(), env->getParts()[j].get(), request, result);
			//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_ << "\t" << result.min_distance;
			min_dist = std::min(min_dist, (double) result.min_distance);

			planes->at(j).col(i) << result.nearest_points[1][0],
									result.nearest_points[1][1],
									result.nearest_points[1][2],
									result.nearest_points[0][0] - result.nearest_points[1][0],
									result.nearest_points[0][1] - result.nearest_points[1][1],
									result.nearest_points[0][2] - result.nearest_points[1][2];
			std::cout << "i = " << i << ", j = " << j << ". NN points: " << result.nearest_points[0] 
					<< "   " << result.nearest_points[1] << std::endl;
		}
	}
	return {min_dist, planes};
}