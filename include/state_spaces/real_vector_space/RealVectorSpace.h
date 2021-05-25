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
		~RealVectorSpace();
		int getDimensions() override;
		friend std::ostream &operator<<(std::ostream &os, const RealVectorSpace &space);

		// these two function needs to be reimplemented in the context
		// of collision checking (some other app with FCL)
		virtual bool isValid(const std::shared_ptr<base::State> q) override;
		virtual float getDistance(const std::shared_ptr<base::State>q) override;

		std::shared_ptr<base::State> interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, double t) override;
		bool equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		std::shared_ptr<base::State> randomState() override;
		std::shared_ptr<Motion> getMotion(std::shared_ptr<base::State> s1, std::shared_ptr<base::State> s2) override;
private:
		int dimensions;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
