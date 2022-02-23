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

void base::Tree::clearTree()
{
	states->clear();
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
	float out_dist_sqr;
	nanoflann::KNNResultSet<float> resultSet(num_results);
	resultSet.init(&q_near_idx, &out_dist_sqr);
	std::vector<float> vec(q->getCoord().data(), 
						    q->getCoord().data() + q->getCoord().rows() * q->getCoord().cols());
	float *vec_c = &vec[0];
	kdtree->findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	vec_c = nullptr;
	return states->at(q_near_idx);
}

std::shared_ptr<base::State> base::Tree::getNearestStateV2(std::shared_ptr<base::State> q)
{
	int q_nearIdx = 0;
	float d, d_min = INFINITY;
	bool isOut;
	Eigen::VectorXf q_temp;

	for (int i = 0; i < states->size(); i++)
	{
		q_temp = getState(i)->getCoord();
		isOut = false;
		for (int k = 0; k < q->getDimensions(); k++)
		{
			if (abs(q_temp(k) - q->getCoord(k)) > d_min)	// Is outside the box?
			{
				isOut = true;
				break;
			}
		}
		if (!isOut)	// Is inside the box?
		{    
			d = (q_temp - q->getCoord()).norm();
			if (d < d_min)
			{
				q_nearIdx = i;
				d_min = d;
			}
		}
	}
	return getState(q_nearIdx);
}

// 'q_new' - new node added to tree
// 'q_parent' - parent of 'q_new'
void base::Tree::upgradeTree(std::shared_ptr<KdTree> kdtree, std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
							 float d_c, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes, float cost)
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

// 'q_new' - new node added to tree
// 'q_parent' - parent of 'q_new'
void base::Tree::upgradeTree(std::shared_ptr<base::State> q_new, std::shared_ptr<base::State> q_parent,
							 float d_c, std::shared_ptr<std::vector<Eigen::MatrixXf>> planes, float cost)
{
	size_t N = states->size();
	states->emplace_back(q_new);
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