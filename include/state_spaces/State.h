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
	public:
		enum Status {Advanced, Trapped, Reached};
		
	private:
		StateSpaceType state_space_type;
		int dimensions;													// Dimensionality in C-space
		Eigen::VectorXf coord;											// Coordinates in C-space
		uint tree_idx = -1;												// Tree index in which the state is stored
		size_t idx = -1; 												// Index of the state in the tree
		float d_c = -1;													// Distance-to-obstacles
		float cost = -1;                  								// Cost-to-come
		std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = nullptr;	// Lines/planes dividing space into two subspaces (free and occupied)
		std::shared_ptr<State> parent = nullptr;
		std::shared_ptr<std::vector<std::shared_ptr<State>>> children = std::make_shared<std::vector<std::shared_ptr<base::State>>>();
		
	public:
		State() {}
		virtual ~State() = 0;

		inline StateSpaceType getStateSpaceType() const { return state_space_type; }
		inline int getDimensions() const { return dimensions; }
		inline Eigen::VectorXf getCoord() const { return coord; }		// 'coord' is not returned by reference intentionally
		inline float getCoord(int idx) const { return coord(idx); }
		inline uint getTreeIdx() const { return tree_idx; }
		inline size_t getIdx() const { return idx; }
		inline float getDistance() const { return d_c; }
		inline float getCost() const { return cost; }
		inline std::shared_ptr<std::vector<Eigen::MatrixXf>> getPlanes() const { return planes; }
		inline std::shared_ptr<State> getParent() const { return parent; }
		inline std::shared_ptr<std::vector<std::shared_ptr<State>>> getChildren() const { return children; };

		inline void setStateSpaceType(StateSpaceType state_space_type_) { state_space_type = state_space_type_; }
		inline void setDimensions(int dimensions_) { dimensions = dimensions_; }
		inline void setCoord(Eigen::VectorXf coord_) { coord = coord_; } 	// 'coord_' is not sent by reference intentionally
		inline void setCoord(const float coord_, int idx) { coord(idx) = coord_; }
		inline void setTreeIdx(uint treeIdx_) { tree_idx = treeIdx_; }
		inline void setIdx(size_t idx_) { idx = idx_; }
		inline void setDistance(float d_c_) { d_c = d_c_; }
		inline void setCost(float cost_) { cost = cost_; }
		inline void setPlanes(std::shared_ptr<std::vector<Eigen::MatrixXf>> planes_) { planes = planes_; }
		inline void setParent(std::shared_ptr<State> parent_) { parent = parent_; }
		inline void setChildren(std::shared_ptr<std::vector<std::shared_ptr<State>>> children_) { children = children_; }

		void addChild(std::shared_ptr<State> child);
		friend std::ostream &operator<<(std::ostream &os, const State *state);
	};
}
#endif //RPMPL_STATE_H
