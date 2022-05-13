//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
//

#include "RealVectorSpaceState.h"

base::RealVectorSpaceState::RealVectorSpaceState(Eigen::VectorXf state_)
{
	dimensions = state_.size();
	coord = state_;
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

base::RealVectorSpaceState::RealVectorSpaceState(int dimensions_)
{
	dimensions = dimensions_;
	coord = Eigen::VectorXf::Random(dimensions);
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

// Make a copy of 'state'
base::RealVectorSpaceState::RealVectorSpaceState(std::shared_ptr<base::State> state)
{
	dimensions = state->getDimensions();
	coord = state->getCoord();
	tree_idx = state->getTreeIdx();
	idx = state->getIdx();
	d_c = state->getDistance();
	cost = state->getCost();
	planes = state->getPlanes();
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setParent(state->getParent());
	setChildren(state->getChildren());
}
