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
		explicit AbstractPlanner(std::shared_ptr<base::StateSpace> ss_) { ss = ss_; };
		virtual ~AbstractPlanner() = 0;
		virtual bool solve() = 0;
		std::shared_ptr<base::StateSpace> getSs() const;
	protected:
		std::shared_ptr<base::StateSpace> ss;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
