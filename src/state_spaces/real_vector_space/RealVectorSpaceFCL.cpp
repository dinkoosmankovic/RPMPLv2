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
	for (double t = 1./numChecks; t <= 1; t += 1./numChecks)
	{
		std::shared_ptr<base::State> q_t = interpolate(q1, q2, t);
		/*if (q_t != nullptr) 
		{
			LOG(INFO) << "checking " << t << " : " << q_t->getCoord().transpose();
			LOG(INFO) << "check: " << isValid(q_t);
		}*/
		if (q_t != nullptr && !isValid(q_t))
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

		rand[i] = llimit + rand[i] * (ulimit - llimit) / 2;
	}
	//LOG(INFO) << "random coord: " << rand.transpose();
	state->setCoord(rand);
	return state;
}

Eigen::VectorXf normalizedAngles(Eigen::VectorXf r)
{
	
	
}

float base::RealVectorSpaceFCL::getDistance(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	float min_dist = 1e12;
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
			min_dist = std::min( min_dist, (float)result.min_distance );
		}
	}
	return min_dist;
}