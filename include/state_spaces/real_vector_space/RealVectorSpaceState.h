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
		uint treeIdx;								// Tree index in which the state is stored
		size_t idx; 								// Index of the state in the tree
		double d_c = -1;							// Distance-to-obstacles
		double cost = -1;                  			// Cost-to-come
		std::shared_ptr<std::vector<Eigen::MatrixXd>> planes = nullptr;	// Lines/planes dividing space into two subspaces (free and occupied)
	
	public:
		RealVectorSpaceState(Eigen::VectorXf state_);
		RealVectorSpaceState(int dimensions_);
		RealVectorSpaceState(base::RealVectorSpaceState* state);
		~RealVectorSpaceState() {}

		inline int getDimension() const override { return dimensions; }
		inline const Eigen::VectorXf &getCoord() const override { return coord; }
		inline const float getCoord(int idx) const override { return coord(idx); }
		inline uint getTreeIdx() const override { return treeIdx; }
		inline size_t getIdx() const override { return idx; }
		inline double getDistance() const override { return d_c; }
		inline double getCost() const override { return cost; }
		inline std::shared_ptr<std::vector<Eigen::MatrixXd>> getPlanes() const override { return planes; }

		inline void setDimensions(int dimensions_) { dimensions = dimensions_; }
		inline void setCoord(const Eigen::VectorXf &coord_) { coord = coord_; }
		inline void setTreeIdx(uint treeIdx_) override { treeIdx = treeIdx_; }
		inline void setIdx(size_t idx_) override { idx = idx_; }
		inline void setDistance(double d_c_) override { d_c = d_c_; }
		inline void setCost(double cost_) override { cost = cost_; }
		inline void setPlanes(std::shared_ptr<std::vector<Eigen::MatrixXd>> planes_) override { planes = planes_; }

		void makeCopy(std::shared_ptr<State> q) override;
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H
