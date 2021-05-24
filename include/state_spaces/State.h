//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"
#include <Eigen/Dense>
#include <vector>

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
		virtual int getDimension() const = 0;
	protected:
		State(){};
		virtual ~State() = 0;
		StateSpaceType stateSpaceType;
	};

	class Tree
	{
		std::vector<base::State*>  *states;
	public:
		Tree(std::vector<State *> *states_) : states(states_) {}
		Tree()
		{
			states = new std::vector<State*>();
		}

		std::vector<base::State *> *getStates() const
		{
			return states;
		}

		void setStates(std::vector<State *> *states)
		{
			Tree::states = states;
		}

		void emptyTree()
		{
			states->empty();
		}

		inline size_t kdtree_get_point_count() const { return states->size(); }

		inline double kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			return states->at(idx)->getCoord()[dim];
		}
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
	};

}
#endif //RPMPL_STATE_H
