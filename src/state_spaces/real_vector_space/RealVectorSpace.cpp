//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>

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

bool base::RealVectorSpace::isValid(const base::State *q)
{
	return true;
}

float base::RealVectorSpace::getDistance(const base::State *q)
{
	return 0;
}

base::RealVectorSpaceState *base::RealVectorSpace::randomState()
{
	base::RealVectorSpaceState* state = new base::RealVectorSpaceState(dimensions);
	return state;
}

base::Motion *base::RealVectorSpace::getMotion(base::State* s1, base::State* s2)
{
	base::Motion* motion = new base::Motion(s1, s2);
	return nullptr;
}

base::State *base::RealVectorSpace::interpolate(const base::State *q1, const base::State *q2, double t)
{
	base::State *q_t = randomState();
	Eigen::VectorXf eig = ( q2->getCoord() - q1->getCoord() ) / (q2->getCoord() - q1->getCoord()).norm();
	q_t->setCoord( q1->getCoord() + t * eig );
	if (isValid(q_t))
		return q_t;
	else
		return nullptr;
}

bool base::RealVectorSpace::equal(const base::State *q1, const base::State *q2)
{
	double d = (q1->getCoord() - q2->getCoord()).norm();
	// TODO: needs to be obtained from configuration file
	if (d < 5)
		return true;
	return false;
}
