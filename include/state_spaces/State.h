//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_STATE_H
#define RPMPL_STATE_H

#include "StateSpaceType.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace base
{
	class State
	{
	private:
		std::shared_ptr<State> parent;
	public:
		std::shared_ptr<State> getParent() const;
		void setParent(std::shared_ptr<State> parent_);
		StateSpaceType getStateSpaceType() const;
		void setStateSpaceType(StateSpaceType stateSpaceType);
		virtual const Eigen::VectorXf &getCoord() const = 0;
		virtual void setCoord(const Eigen::VectorXf &coord) = 0;
		virtual int getDimension() const = 0;
		friend std::ostream& operator<<(std::ostream& os, const State* state);
		virtual ~State() = 0;
	protected:
		State(){};
		StateSpaceType stateSpaceType;
	};

	class Tree
	{
		std::shared_ptr<std::vector<std::shared_ptr<base::State>>> states;
	public:
		Tree(std::shared_ptr<std::vector<std::shared_ptr<base::State> > > states_) : states(states_) {}
		~Tree(){}
		Tree()
		{
			states = std::make_shared<std::vector<std::shared_ptr<State>>>();
		}

		std::shared_ptr<std::vector<std::shared_ptr<base::State>>> getStates() const
		{
			return states;
		}

		void setStates(std::shared_ptr<std::vector<std::shared_ptr<base::State>>>states)
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

		friend std::ostream& operator<<(std::ostream& os, const Tree& tree)
		{
			os << "Tree: " << std::endl;
			for (int i = 0; i < tree.getStates()->size(); ++i)
			{
				os << tree.getStates()->at(i);

			}
			return os;
		}
	};

}
#endif //RPMPL_STATE_H
