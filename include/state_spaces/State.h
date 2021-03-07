//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

namespace base
{
	class State
	{
	private:
		State* parent;
	public:
		State *getParent() const;
		void setParent(State *parent_);
	protected:
		State(){};
		virtual ~State() = 0;
	};
}
#endif //RPMPL_STATE_H
