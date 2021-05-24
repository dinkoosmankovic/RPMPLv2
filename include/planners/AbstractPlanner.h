//
// Created by dinko on 16.3.21..
//
#ifndef RPMPL_ABSTRACTPLANNER_H
#define RPMPL_ABSTRACTPLANNER_H

#include "StateSpace.h"
#include <nanoflann.hpp>

namespace planning
{
	typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
	nanoflann::L2_Simple_Adaptor<double, base::Tree> ,
	base::Tree /* dim */
	> KdTree;

	class AbstractPlanner
	{
	public:
		explicit AbstractPlanner(base::StateSpace* ss_) { ss = ss_; };
		virtual ~AbstractPlanner() = 0;
		virtual bool solve() = 0;
		base::StateSpace *getSs() const;
	protected:
		base::StateSpace* ss;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
