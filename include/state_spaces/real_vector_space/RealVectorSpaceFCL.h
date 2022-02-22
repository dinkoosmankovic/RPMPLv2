//
// Created by dinko on 14.2.22.
//

#ifndef RPMPL_REALVECTORSPACEFCL_H
#define RPMPL_REALVECTORSPACEFCL_H

#include <ostream>
#include <memory>
#include <vector>
#include "StateSpace.h"
#include "RealVectorSpaceState.h"
#include "RealVectorSpace.h"

namespace base
{
	class RealVectorSpaceFCL : public base::RealVectorSpace
	{
	public:
		RealVectorSpaceFCL(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, const std::shared_ptr<env::Environment> env_);
		~RealVectorSpaceFCL();
		bool isValid(const std::shared_ptr<base::State> q) override;
		bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		float getDistance(const std::shared_ptr<base::State> q) override;
		std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> getDistanceAndPlanes(const std::shared_ptr<base::State> q) override;
		std::shared_ptr<base::State> randomState() override;
		std::shared_ptr<base::State> newState(std::shared_ptr<base::State> q) override;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
