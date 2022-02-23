//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>
#include <glog/log_severity.h>
#include <glog/logging.h>
#include "ConfigurationReader.h"

base::RealVectorSpace::RealVectorSpace(int dimensions) : dimensions(dimensions)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::~RealVectorSpace()
{
}

int base::RealVectorSpace::getDimensions()
{
	return dimensions;
}

std::ostream &base::operator<<(std::ostream &os, const base::RealVectorSpace &space)
{
	os << " dimensions: " << space.dimensions;
	return os;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q)
{
	return true;
}

float base::RealVectorSpace::getDistance(const std::shared_ptr<base::State> q)
{
	return 0;
}

std::shared_ptr<base::State> base::RealVectorSpace::randomState()
{
	std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(dimensions);
	state->setCoord(Eigen::VectorXf::Random(dimensions));
	return state;
}

std::shared_ptr<base::State> base::RealVectorSpace::interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step)
{
	std::shared_ptr<base::State> q_t = randomState();
	float distanceStates = (q2->getCoord() - q1->getCoord()).norm();
	Eigen::VectorXf eig; 
	if (step < distanceStates)
	{
		eig = (q2->getCoord() - q1->getCoord()) / distanceStates;
		q_t->setCoord(q1->getCoord() + step * eig);
	}
	else
	{
		q_t->setCoord(q2->getCoord());
	}
	// here we check the validity of the motion q1->q_t
	if (isValid(q_t))
		return q_t;
	else
		return nullptr;
}

bool base::RealVectorSpace::equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	double d = (q1->getCoord() - q2->getCoord()).norm();
	double stateEqualityThreshold = RealVectorSpaceConfig::EQUALITY_THRESHOLD;
	if (d < stateEqualityThreshold)
		return true;
	return false;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	int numChecks = RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
	float statesDistance = (q2->getCoord() - q1->getCoord()).norm();
	for (double t = 1./numChecks; t <= 1; t += 1./numChecks)
	{
		std::shared_ptr<base::State> q_t = interpolate(q1, q2, t);
		if (q_t != nullptr && !isValid(q_t))
			return false;
	}
	return true;
}
