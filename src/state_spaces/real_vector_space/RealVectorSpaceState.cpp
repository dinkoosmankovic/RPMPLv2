//
// Created by dinko on 7.3.21..
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(int dimensions_, Eigen::VectorXf state_)
{
	dimensions = dimensions_;
	coord = state_;
}

base::RealVectorSpaceState::~RealVectorSpaceState()
{
}

base::RealVectorSpaceState::RealVectorSpaceState(int dimensions_)
{
	dimensions = dimensions_;
	coord = Eigen::VectorXf::Random(dimensions);
}

base::RealVectorSpaceState::RealVectorSpaceState(base::RealVectorSpaceState* state)
{
	dimensions = state->dimensions;
	coord = state->coord;
}

int base::RealVectorSpaceState::getDimensions() const
{
	return dimensions;
}

void base::RealVectorSpaceState::setDimensions(int dimensions)
{
	RealVectorSpaceState::dimensions = dimensions;
}

const Eigen::VectorXf &base::RealVectorSpaceState::getCoord() const
{
	return coord;
}

void base::RealVectorSpaceState::setCoord(const Eigen::VectorXf &coord)
{
	RealVectorSpaceState::coord = coord;
}
