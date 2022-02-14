//
// Created by dinko on 14.02.22.
//

#ifndef RPMPL_ENVIRONMENT_H
#define RPMPL_ENVIRONMENT_H

#include <vector> 
#include <memory>

#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>

namespace env
{
	class Environment
	{
	public:
		Environment(const std::string& filename); // filename with description
        Environment(const fcl::Box& box, const fcl::Transform3f& tf);
		~Environment();
		//virtual void computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		const std::vector<std::shared_ptr<fcl::CollisionObject> >& getParts() const;
	private:
		std::vector<std::shared_ptr<fcl::CollisionObject> > parts_;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H
