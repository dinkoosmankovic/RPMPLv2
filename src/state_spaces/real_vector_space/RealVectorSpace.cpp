//
// Created by dinko on 7.3.21..
//
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>

base::RealVectorSpace::RealVectorSpace(int dimensions) : dimensions(dimensions)
{}

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
	return false;
}

float base::RealVectorSpace::getDistance(const base::State *q)
{
	return 0;
}

base::State *base::RealVectorSpace::randomState()
{
	base::State* state = new base::RealVectorSpaceState(dimensions);
	return state;
}

base::Motion *base::RealVectorSpace::getMotion(base::State* s1, base::State* s2)
{
	base::Motion* motion = new base::Motion(s1, s2);
	return nullptr;
}
