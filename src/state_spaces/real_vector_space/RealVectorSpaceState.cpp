//
// Created by dinko on 7.3.21..
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(Eigen::VectorXf state_)
{
	dimensions = state_.size();
	coord = state_;
	stateSpaceType = StateSpaceType::RealVectorSpace;
}

base::RealVectorSpaceState::~RealVectorSpaceState()
{
}

base::RealVectorSpaceState::RealVectorSpaceState(int dimensions_)
{
	dimensions = dimensions_;
	coord = Eigen::VectorXf::Random(dimensions);
	stateSpaceType = StateSpaceType::RealVectorSpace;
}

base::RealVectorSpaceState::RealVectorSpaceState(base::RealVectorSpaceState* state)
{
	dimensions = state->dimensions;
	coord = state->coord;
	stateSpaceType = StateSpaceType::RealVectorSpace;
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

int base::RealVectorSpaceState::getDimension() const
{
	return dimensions;
}
