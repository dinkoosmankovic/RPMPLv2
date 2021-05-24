//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"
#include <Eigen/Dense>

namespace base
{
	class State
	{
	private:
		State* parent;
	public:
		State *getParent() const;
		void setParent(State *parent_);
		StateSpaceType getStateSpaceType() const;
		void setStateSpaceType(StateSpaceType stateSpaceType);
		virtual const Eigen::VectorXf &getCoord() const = 0;
		virtual void setCoord(const Eigen::VectorXf &coord) = 0;
	protected:
		State(){};
		virtual ~State() = 0;
		StateSpaceType stateSpaceType;
	};
}
#endif //RPMPL_STATE_H
