//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_RRTCONNECT_H
#define RPMPL_RRTCONNECT_H

#include "AbstractPlanner.h"

namespace planning
{
	namespace rrt
	{
		class RRTConnect : public AbstractPlanner
		{
		public:
			RRTConnect(base::StateSpace *ss_);
			RRTConnect(base::StateSpace *ss_, base::State *start_, base::State *goal_);
			~RRTConnect();
			bool solve() override;
		private:
			base::StateSpace* ss;
			base::State* start;
			base::State* goal;

		};
	}
}
#endif //RPMPL_RRTCONNECT_H
