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

#include <fcl/distance.h>
#include <fcl/collision.h>

base::RealVectorSpaceFCL::~RealVectorSpaceFCL()
{
}

base::RealVectorSpaceFCL::RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
											 const std::shared_ptr<env::Environment> env_) : RealVectorSpace(dimensions_)
{
	robot = robot_;
	env_;
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
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

float base::RealVectorSpaceFCL::getDistance(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	float min_dist = 1e12;
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			fcl::DistanceRequest request;
			fcl::DistanceResult result;
			fcl::distance(robot->getParts()[i].get(), env->getParts()[j].get(), request, result);
			min_dist = std::min( min_dist, (float)result.min_distance );
		}
	}
	return min_dist;
}