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
		bool isValid(const base::State *q) override;
		float getDistance(const base::State *q) override;
		RealVectorSpaceState *randomState() override;
		base::Motion *getMotion(base::State* s1, base::State* s2) override;
	private:
		int dimensions;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
