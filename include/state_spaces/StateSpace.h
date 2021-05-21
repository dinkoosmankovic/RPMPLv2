#pragma once
#include "State.h"
#include "Motion.h"
#include "StateSpaceType.h"

namespace base
{
	class StateSpace
	{
	public:
		StateSpace(){};
		virtual ~StateSpace() = 0;
		virtual int getDimensions() = 0;
		virtual bool isValid(const State* q) = 0;
		virtual float getDistance(const State* q) = 0;
		virtual State* randomState() = 0;
		virtual Motion* getMotion(State* s1, State* s2) = 0;
		virtual StateSpaceType getStateSpaceType() const;
		virtual void setStateSpaceType(StateSpaceType stateSpaceType);
	protected:
		StateSpaceType stateSpaceType;
	};
}

