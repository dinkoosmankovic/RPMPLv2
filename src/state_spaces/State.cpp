//
// Created by dinko on 7.3.21..
//

#include "State.h"

base::State::~State() {}

void base::State::addChild(std::shared_ptr<base::State> child)
{
	children->emplace_back(child);
}

std::ostream &base::operator<<(std::ostream &os, const base::State* state)
{
	if (state->getParent() == nullptr)
		os << "q: (" << state->getCoord().transpose() << "); parent q: NONE";
	else
		os << "q: (" << state->getCoord().transpose() << "); parent q: (" <<
		   state->getParent()->getCoord().transpose() << ")";
	return os;
}

// -------------------------------------------------------------------------- //

base::Tree::Tree(std::string treeName_, uint treeNum_)
{
	treeName = treeName_;
	treeIdx = treeNum_;
}

void base::Tree::emptyTree()
{
	states->empty();
}

std::ostream& base::operator<<(std::ostream& os, const Tree& tree)
{
	os << "Tree: " << tree.getTreeName() << std::endl;
	for (int i = 0; i < tree.getStates()->size(); ++i)
	{
		os << tree.getStates()->at(i) << std::endl;
	}
	return os;
}

std::shared_ptr<base::State> base::Tree::getNearestState(std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q)
{
	const size_t num_results = 1;
	size_t q_near_idx;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&q_near_idx, &out_dist_sqr);
	std::vector<double> vec(q->getCoord().data(), 
							q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());
	double *vec_c = &vec[0];
	kdtree->findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	vec_c = nullptr;
	return states->at(q_near_idx);
}

// 'q_new' - new node added to tree
// 'q_parent' - parent of 'q_new'
void base::Tree::upgradeTree(std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
							 double d_c, std::shared_ptr<std::vector<Eigen::MatrixXd>> planes, double cost)
{
	size_t N = states->size();
	states->emplace_back(q_new);
	kdtree->addPoints(N, N);
	q_new->setTreeIdx(getTreeIdx());
	q_new->setIdx(N);
	q_new->setParent(q_parent);
	q_new->setDistance(d_c);
	q_new->setPlanes(planes);
	q_new->setCost(cost);
	if (q_parent != nullptr)
	{
		q_parent->addChild(q_new);
	}
}