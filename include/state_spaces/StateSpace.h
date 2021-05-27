#pragma once
#include "State.h"
#include "StateSpaceType.h"

namespace base
{
	class StateSpace
	{
	public:
		StateSpace(){};
		virtual ~StateSpace() = 0;
		virtual int getDimensions() = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q) = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual float getDistance(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> randomState() = 0;
		virtual StateSpaceType getStateSpaceType() const;
		virtual void setStateSpaceType(StateSpaceType stateSpaceType);
		virtual std::shared_ptr<base::State> interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, double t) = 0;
		virtual bool equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
	protected:
		StateSpaceType stateSpaceType;
	};
}

