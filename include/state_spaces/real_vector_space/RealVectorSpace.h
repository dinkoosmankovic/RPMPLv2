//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_REALVECTORSPACE_H
#define RPMPL_REALVECTORSPACE_H

#include <ostream>
#include "StateSpace.h"
#include "RealVectorSpaceState.h"

namespace base
{
	class RealVectorSpace : public base::StateSpace
	{
	public:
		RealVectorSpace(int dimensions);
		virtual ~RealVectorSpace();
		int getDimensions() override;
		friend std::ostream &operator<<(std::ostream &os, const RealVectorSpace &space);

		// these four function needs to be reimplemented in the context
		// of collision checking (some other app with FCL)
		virtual bool isValid(const std::shared_ptr<base::State> q) override;
		// this one if cont. collision vs. discrete collision check is used
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		virtual float getDistance(const std::shared_ptr<base::State>q) override;
		virtual std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> getDistanceAndPlanes(const std::shared_ptr<base::State> q) override;

		std::shared_ptr<base::State> interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float D) override;
		bool equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		virtual std::shared_ptr<base::State> randomState() override;
		virtual std::shared_ptr<base::State> newState(std::shared_ptr<base::State> q) override;

	protected:
		int dimensions;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
