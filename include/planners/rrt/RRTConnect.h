//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"
#include <vector>

namespace planning
{
	namespace rrt
	{
		enum Status {Advanced, Trapped, Reached};
		class RRTConnect : public AbstractPlanner
		{
		public:
			RRTConnect(base::StateSpace *ss_);
			RRTConnect(base::StateSpace *ss_, base::State *start_, base::State *goal_);
			~RRTConnect();
			bool solve() override;
			base::Tree getStartTree() const;
			base::Tree getGoalTree() const;
			Status extend(base::Tree* tree, KdTree* kdtree, base::State* q_rand);
			Status connect(base::Tree* tree, KdTree* kdtree, base::State* q_rand);
			base::State* get_q_near(base::Tree* tree, KdTree* kdtree, base::State* q);
			void addNode(base::Tree* tree, KdTree* kdtree, base::State* q);
			const std::vector<base::State *> &getPath() const;

		private:
			base::StateSpace* ss;
			base::State* start;
			base::State* goal;
			base::Tree startTree;
			base::Tree goalTree;
			void initPlanner();
			void prepareKdTrees();
			void computePath();
			KdTree* startKdTree;
			KdTree* goalKdTree;
			double step = 10;
			std::vector<base::State*> path;

		};
	}
}
#endif //RPMPL_RRTCONNECT_H
