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
#include "RealVectorSpaceConfig.h"


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
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			// LOG(INFO) << "robot i: " << i;
			// LOG(INFO) << "env j: " << j;
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
	int numChecks = RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
	float D = (q2->getCoord() - q1->getCoord()).norm();
	for (float t = 1./numChecks; t <= 1; t += 1./numChecks)
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
	Eigen::VectorXf rand = Eigen::VectorXf::Random(dimensions);
	std::vector<std::vector<float>> limits = robot->getLimits();
	for (size_t i = 0; i < dimensions; ++i)
	{
		rand[i] = ((limits[i][1] - limits[i][0]) * rand[i] + limits[i][0] + limits[i][1]) / 2;
	}
	//LOG(INFO) << "random coord: " << rand.transpose();
	state->setCoord(rand);
	return state;
}

std::shared_ptr<base::State> base::RealVectorSpaceFCL::newState(std::shared_ptr<base::State> state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

std::shared_ptr<base::State> base::RealVectorSpaceFCL::newState(const Eigen::VectorXf &state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

Eigen::VectorXf normalizedAngles(Eigen::VectorXf r)
{
	
}

float base::RealVectorSpaceFCL::getDistance(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	float min_dist = INFINITY;
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
			min_dist = std::min(min_dist, (float) result.min_distance);
		}
	}
	return min_dist;
}

std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> base::RealVectorSpaceFCL::getDistanceAndPlanes
	(const std::shared_ptr<base::State> q)
{
	robot->setState(q);
	float min_dist = INFINITY;
	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	
	for (size_t i = 0; i < robot->getParts().size(); ++i)
	{	
		//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_;
		robot->getParts()[i]->computeAABB();
		for (size_t j = 0; j < env->getParts().size(); ++j)
		{
			// fcl::DistanceRequest request(true);
			fcl::DistanceRequest request(true, 0.01, 0.01, fcl::GST_INDEP);
			fcl::DistanceResult result;
			fcl::distance(robot->getParts()[i].get(), env->getParts()[j].get(), request, result);
			//LOG(INFO) << "part " << i <<"\t:" << robot->getParts()[i]->getAABB().min_ <<"\t;\t" << robot->getParts()[i]->getAABB().max_ << "\t" << result.min_distance;
			min_dist = std::min(min_dist, (float) result.min_distance);

			fcl::Vec3f link_point = result.nearest_points[0];
			link_point = robot->getParts().at(i)->getTransform().getRotation() * link_point;
			link_point += robot->getParts().at(i)->getTransform().getTranslation();

			fcl::Vec3f obs_point = result.nearest_points[1];
			obs_point = env->getParts().at(j)->getTransform().getRotation() * obs_point;
			obs_point += env->getParts().at(j)->getTransform().getTranslation();

			// std::cout << "link: " << i << std::endl;
			// std::cout << "link_point: " << link_point << std::endl;
			// std::cout << "obs_point: " << obs_point << std::endl;
			// std::cout << "----------------------------------" << std::endl;

			planes->at(j).col(i) << obs_point[0], 
									obs_point[1],
									obs_point[2],
									link_point[0] - obs_point[0],
									link_point[1] - obs_point[1],
									link_point[2] - obs_point[2];

		}
	}
	return {min_dist, planes};
}