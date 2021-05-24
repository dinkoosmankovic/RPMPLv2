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
#include "State.h"


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

	base::Tree tree; tree.states = &states;
	const int dim = test_state->getDimension();
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
			nanoflann::L2_Simple_Adaptor<double, base::Tree> ,
			base::Tree /* dim */
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
	std::cout << "query point: " << test_state->getCoord().transpose() << std::endl;
	std::cout << "*******************************************" << std::endl;
	index.findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
	std::cout << "point: " << tree.states->at(ret_index)->getCoord().transpose() << std::endl;
	std::cout << "No. points in tree: " << tree.states->size() << std::endl;
	std::cout << "*******************************************" << std::endl;
	int K = tree.states->size();
	tree.states->emplace_back(new_state);
	index.addPoints(K - 1, K);

	index.findNeighbors(resultSet, vec_c, nanoflann::SearchParams(10));
	std::cout << "knnSearch(nn="<<num_results<<"): \n";
	std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << std::endl;
	std::cout << "point: " << tree.states->at(ret_index)->getCoord().transpose() << std::endl;
	std::cout << "No. points in tree: " << tree.states->size() << std::endl;

}

int main()
{
	base::StateSpace *ss = new base::RealVectorSpace(2);
	std::vector<base::State*> states;
	generateRandomStates(ss, states, 100);
	buildKDTree(states);
	return 0;
}

