//
// Created by dinko on 14.02.22.
//

#ifndef RPMPL_ENVIRONMENT_H
#define RPMPL_ENVIRONMENT_H

#include <vector> 
#include <memory>

#include "fcl/fcl.h"

namespace env
{
	typedef std::pair<fcl::Box<float>, fcl::Transform3<float>> Obstacle;
	class Environment
	{
	public:
		Environment(const std::string& filename); // filename with description
        Environment(const fcl::Box<float> &box, const fcl::Transform3<float> &tf);
		Environment(std::vector<Obstacle> obs);
		~Environment();
		//virtual void computeForwardKinematics(std::shared_ptr<base::State> q) = 0;
		const std::vector<std::shared_ptr<fcl::CollisionObject<float>>> &getParts() const;
	private:
		std::vector<std::shared_ptr<fcl::CollisionObject<float>>> parts_;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H