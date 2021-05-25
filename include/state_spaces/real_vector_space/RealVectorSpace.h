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
		virtual bool isValid(const base::State *q) override;
		virtual float getDistance(const base::State *q) override;

		base::State* interpolate(const base::State* q1, const base::State* q2, double t) override;
		RealVectorSpaceState *randomState() override;
		base::Motion *getMotion(base::State* s1, base::State* s2) override;
	bool equal(const base::State *q1, const base::State *q2) override;
private:
		int dimensions;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
