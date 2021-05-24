//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"
#include <vector>

typedef std::vector<base::State*> Tree;

namespace planning
{
	namespace rrt
	{
		enum Status {Advance, Trapped, Reached};
		class RRTConnect : public AbstractPlanner
		{
		public:
			RRTConnect(base::StateSpace *ss_);
			RRTConnect(base::StateSpace *ss_, base::State *start_, base::State *goal_);
			~RRTConnect();
			bool solve() override;
			Tree getStartTree() const;
			Tree getGoalTree() const;
			Status extend(Tree tree, base::State* q_rand);
			base::State* get_q_near(base::State* q);

		private:
			base::StateSpace* ss;
			base::State* start;
			base::State* goal;
			Tree startTree;
			Tree goalTree;
			void initPlanner();
		};
	}
}
#endif //RPMPL_RRTCONNECT_H
