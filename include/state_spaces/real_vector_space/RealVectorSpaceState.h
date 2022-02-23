//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_REALVECTORSPACESTATE_H
#define RPMPL_REALVECTORSPACESTATE_H

#include "State.h"
#include <Eigen/Dense>

namespace base
{
	class RealVectorSpaceState : public State
	{
	private:
		int dimensions;
		Eigen::VectorXf coord;
		uint treeIdx = -1;							// Tree index in which the state is stored
		size_t idx = -1; 							// Index of the state in the tree
		float d_c = -1;								// Distance-to-obstacles
		float cost = -1;                  			// Cost-to-come
		std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = nullptr;	// Lines/planes dividing space into two subspaces (free and occupied)
	
	public:
		RealVectorSpaceState(Eigen::VectorXf state_);
		RealVectorSpaceState(int dimensions_);
		RealVectorSpaceState(std::shared_ptr<base::State> state);
		~RealVectorSpaceState() {}

		inline int getDimensions() const override { return dimensions; }
		inline Eigen::VectorXf getCoord() const override { return coord; }
		inline float getCoord(int idx) const override { return coord(idx); }
		inline uint getTreeIdx() const override { return treeIdx; }
		inline size_t getIdx() const override { return idx; }
		inline float getDistance() const override { return d_c; }
		inline float getCost() const override { return cost; }
		inline std::shared_ptr<std::vector<Eigen::MatrixXf>> getPlanes() const override { return planes; }

		inline void setDimensions(int dimensions_) { dimensions = dimensions_; }
		inline void setCoord(Eigen::VectorXf coord_) override { coord = coord_; }
		inline void setCoord(const float coord_, int idx) override { coord(idx) = coord_; }
		inline void setTreeIdx(uint treeIdx_) override { treeIdx = treeIdx_; }
		inline void setIdx(size_t idx_) override { idx = idx_; }
		inline void setDistance(float d_c_) override { d_c = d_c_; }
		inline void setCost(float cost_) override { cost = cost_; }
		inline void setPlanes(std::shared_ptr<std::vector<Eigen::MatrixXf>> planes_) override { planes = planes_; }
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H
