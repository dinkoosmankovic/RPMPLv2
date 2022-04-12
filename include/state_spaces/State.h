//
// Created by dinko on 7.3.21.
// Modified by nermin on 18.02.22.
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
		StateSpaceType state_space_type;
		std::shared_ptr<State> parent = nullptr;
		std::shared_ptr<std::vector<std::shared_ptr<State>>> children = 
			std::make_shared<std::vector<std::shared_ptr<base::State>>>();
		
	public:
		State(){};
		virtual ~State() = 0;

		virtual uint getTreeIdx() const = 0;
		virtual size_t getIdx() const = 0;
		virtual float getDistance() const = 0;
		virtual float getCost() const = 0;
		virtual std::shared_ptr<std::vector<Eigen::MatrixXf>> getPlanes() const = 0;
		virtual int getDimensions() const = 0;
		virtual Eigen::VectorXf getCoord() const = 0;
		virtual float getCoord(int idx) const = 0;
		inline std::shared_ptr<State> getParent() const { return parent; }
		inline std::shared_ptr<std::vector<std::shared_ptr<State>>> getChildren() const { return children; };
		inline StateSpaceType getStateSpaceType() const { return state_space_type; }

		virtual void setTreeIdx(uint treeIdx_) = 0;
		virtual void setIdx(size_t idx_) = 0;
		virtual void setDistance(float d_c_) = 0;
		virtual void setCost(float cost_) = 0;
		virtual void setPlanes(std::shared_ptr<std::vector<Eigen::MatrixXf>> planes_) = 0;
		virtual void setCoord(Eigen::VectorXf coord) = 0;
		virtual void setCoord(const float coord_, int idx) = 0;
		inline void setParent(std::shared_ptr<State> parent_) { parent = parent_; }
		inline void setChildren(std::shared_ptr<std::vector<std::shared_ptr<State>>> children_) { children = children_; }
		inline void setStateSpaceType(StateSpaceType state_space_type_) { state_space_type = state_space_type_; }

		void addChild(std::shared_ptr<State> child);
		friend std::ostream &operator<<(std::ostream &os, const State *state);
	};
}
#endif //RPMPL_STATE_H
