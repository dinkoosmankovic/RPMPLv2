//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"

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
	protected:
		State(){};
		virtual ~State() = 0;
		StateSpaceType stateSpaceType;
	};
}
#endif //RPMPL_STATE_H
