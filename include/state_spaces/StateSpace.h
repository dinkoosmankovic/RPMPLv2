#pragma once
#include "State.h"
#include "StateSpaceType.h"
#include "AbstractRobot.h"
#include "Environment.h"
#include <vector>

namespace base
{
	class StateSpace
	{
	public:
		StateSpaceType stateSpaceType;
		std::shared_ptr<robots::AbstractRobot> robot;
		std::shared_ptr<env::Environment> env;

		StateSpace(){};
		virtual ~StateSpace() = 0;
		virtual int getDimensions() = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q) = 0;
		virtual bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		virtual double getDistance(const std::shared_ptr<base::State> q) = 0;
		virtual std::tuple<double, std::shared_ptr<std::vector<Eigen::MatrixXd>>> getDistanceAndPlanes(const std::shared_ptr<base::State> q) = 0;
		virtual std::shared_ptr<base::State> randomState() = 0;
		virtual StateSpaceType getStateSpaceType() const;
		virtual void setStateSpaceType(StateSpaceType stateSpaceType);
		virtual std::shared_ptr<base::State> interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, double step, double D = -1) = 0;
		virtual bool equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) = 0;
		
	};
}

