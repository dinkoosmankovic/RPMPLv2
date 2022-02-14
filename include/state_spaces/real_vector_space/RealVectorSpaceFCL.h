//
// Created by dinko on 14.2.22.
//

#ifndef RPMPL_REALVECTORSPACEFCL_H
#define RPMPL_REALVECTORSPACEFCL_H

#include <ostream>
#include <memory>
#include "StateSpace.h"
#include "RealVectorSpaceState.h"
#include "RealVectorSpace.h"
#include "AbstractRobot.h"
#include "Environment.h"

namespace base
{
	class RealVectorSpaceFCL : public base::RealVectorSpace
	{
	public:
		RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, const std::shared_ptr<env::Environment> env_);
		~RealVectorSpaceFCL();
		bool isValid(const std::shared_ptr<base::State> q) override;
		float getDistance(const std::shared_ptr<base::State>q) override;

	private:
		std::shared_ptr<robots::AbstractRobot> robot;
		std::shared_ptr<env::Environment> env;
		
	};
}
#endif //RPMPL_REALVECTORSPACE_H
