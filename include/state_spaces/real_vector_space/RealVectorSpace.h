//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_REALVECTORSPACE_H
#define RPMPL_REALVECTORSPACE_H

#include <ostream>
#include "StateSpace.h"

namespace base
{
	class RealVectorSpace : public StateSpace
	{
	public:
		RealVectorSpace(int dimensions);
		~RealVectorSpace();
		int getDimensions() override;
		friend std::ostream &operator<<(std::ostream &os, const RealVectorSpace &space);
		bool isValid(const State *q) override;
		float getDistance(const State *q) override;
		State *randomState() override;
		Motion *getMotion(State* s1, State* s2) override;
	private:
		int dimensions;
	};
}
#endif //RPMPL_REALVECTORSPACE_H
