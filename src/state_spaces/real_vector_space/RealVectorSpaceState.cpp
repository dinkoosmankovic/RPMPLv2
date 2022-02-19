//
// Created by dinko on 7.3.21..
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

base::RealVectorSpaceState::RealVectorSpaceState(base::RealVectorSpaceState* state)
{
	dimensions = state->dimensions;
	coord = state->coord;
	planes = std::make_shared<std::vector<Eigen::MatrixXd>>();
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	setParent(nullptr);
	setChildren(std::make_shared<std::vector<std::shared_ptr<base::State>>>());
}

void base::RealVectorSpaceState::makeCopy(std::shared_ptr<base::State> q)
{
	dimensions = q->getDimension();
	coord = q->getCoord();
	treeIdx = q->getTreeIdx();
	idx = q->getIdx();
	d_c = q->getDistance();
	cost = q->getCost();
	planes = q->getPlanes();
	setStateSpaceType(q->getStateSpaceType());
	setParent(q->getParent());
	setChildren(q->getChildren());
}
