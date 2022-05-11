//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpaceFCL.h"
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>
#include <time.h>
#include "RealVectorSpaceConfig.h"

#include <glog/log_severity.h>
#include <glog/logging.h>

base::RealVectorSpaceFCL::~RealVectorSpaceFCL() {}

base::RealVectorSpaceFCL::RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
											 const std::shared_ptr<env::Environment> env_) : RealVectorSpace(dimensions_, robot_, env_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpaceFCL);
	collision_manager_robot = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
	collision_manager_env = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
}

bool base::RealVectorSpaceFCL::isValid(const std::shared_ptr<base::State> q)
{
	prepareCollisionManager(q);
	fcl::DefaultCollisionData<float> collision_data;
	collision_manager_robot->collide(collision_manager_env.get(), &collision_data, fcl::DefaultCollisionFunction);

	return !collision_data.result.isCollision();
}

float base::RealVectorSpaceFCL::getDistance(const std::shared_ptr<base::State> q)
{
	prepareCollisionManager(q);
	fcl::DefaultDistanceData<float> distance_data;
	distance_data.result.clear();
	collision_manager_robot->distance(collision_manager_env.get(), &distance_data, fcl::DefaultDistanceFunction);

	return distance_data.result.min_distance;
}

std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> base::RealVectorSpaceFCL::getDistanceAndPlanes
	(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	float min_dist = INFINITY;
	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	fcl::DefaultDistanceData<float> distance_data;
	
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			collision_manager_robot->clear();
			collision_manager_env->clear();
			collision_manager_robot->registerObject(robot->getParts()[i].get());
			collision_manager_env->registerObject(env->getParts()[j].get());
			collision_manager_robot->setup();
			collision_manager_env->setup();

			distance_data.request.enable_nearest_points = true;
			distance_data.result.clear();
			collision_manager_robot->distance(collision_manager_env.get(), &distance_data, fcl::DefaultDistanceFunction);
			min_dist = std::min(min_dist, distance_data.result.min_distance);
			planes->at(j).col(i) << distance_data.result.nearest_points[1],
									distance_data.result.nearest_points[0] - distance_data.result.nearest_points[1];

			// std::cout << "(i, j) = (" << i << ", " << j << ")" << std::endl;
			// std::cout << "d_c = " << distance_data.result.min_distance << std::endl;
			// std::cout << "NP robot: " << distance_data.result.nearest_points[0].transpose() << std::endl;
			// std::cout << "NP env:   " << distance_data.result.nearest_points[1].transpose() << std::endl;
		}
	}
	return {min_dist, planes};
}

void base::RealVectorSpaceFCL::prepareCollisionManager(const std::shared_ptr<base::State> q)
{
	collision_manager_robot->clear();
	collision_manager_env->clear();
	robot->setState(q);

	for (size_t i = 0; i < robot->getParts().size(); i++)
		collision_manager_robot->registerObject(robot->getParts()[i].get());
	
	for (size_t j = 0; j < env->getParts().size(); j++)
		collision_manager_env->registerObject(env->getParts()[j].get());
	
	collision_manager_robot->setup();
	collision_manager_env->setup();
}