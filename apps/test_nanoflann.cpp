//
// Created by dinko on 24.5.21..
// Taken from nanoflann examples
//

#include <nanoflann.hpp>

#include <ctime>
#include <cstdlib>
#include <iostream>
#include "RealVectorSpaceState.h"
#include "RealVectorSpace.h"

//typedef std::vector<base::State*> Tree;

struct Tree
{
	std::vector<base::State*>  states;

	inline size_t kdtree_get_point_count() const { return states.size(); }

	inline double kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return states[idx]->getCoord()[dim];
	}
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

void generateRandomStates(base::StateSpace* ss, std::vector<base::State*>& states, int N = 20)
{
	for (int i = 0; i < N; i++)
	{
		states.emplace_back(ss->randomState());
	}
}

void buildKDTree(std::vector<base::State*>& states)
{
	base::State* test_state = new base::RealVectorSpaceState(Eigen::Vector2f({0,0}));

	base::State* new_state = new base::RealVectorSpaceState(Eigen::Vector2f({0.001,0.001}));

	Tree tree; tree.states = states;
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
			nanoflann::L2_Simple_Adaptor<double, Tree> ,
			Tree,
			2 /* dim */
			> KdTree;

	KdTree index(2, tree, nanoflann::KDTreeSingleIndexAdaptorParams(10) );
	const size_t num_results = 1;
	size_t ret_index;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr );
	std::vector<double> vec(test_state->getCoord().data(),
						 test_state->getCoord().data() + test_state->getCoord().rows() *
						 test_state->getCoord().cols());
	double* vec_c = &vec[0];
	index.findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
	std::cout << "point: " << tree.states[ret_index]->getCoord().transpose() << std::endl;
	std::cout << "No. points in tree: " << tree.states.size() << std::endl;
	std::cout << "*******************************************" << std::endl;
	int K = tree.states.size();
	tree.states.emplace_back(new_state);
	index.addPoints(K - 1, K);

	index.findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
	std::cout << "point: " << tree.states[ret_index]->getCoord().transpose() << std::endl;
	std::cout << "No. points in tree: " << tree.states.size() << std::endl;

}

int main()
{
	base::StateSpace *ss = new base::RealVectorSpace(2);
	std::vector<base::State*> states;
	generateRandomStates(ss, states);
	std::cout << states.size() << std::endl;
	buildKDTree(states);
	return 0;
}

