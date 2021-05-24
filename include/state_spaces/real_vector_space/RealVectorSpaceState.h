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
	public:
		RealVectorSpaceState(base::RealVectorSpaceState* state);
		RealVectorSpaceState(int dimensions_);
		RealVectorSpaceState(Eigen::VectorXf state_);
		~RealVectorSpaceState();
		void setDimensions(int dimensions);
		const Eigen::VectorXf &getCoord() const override;
		void setCoord(const Eigen::VectorXf &coord);
		int getDimension() const override;
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H
