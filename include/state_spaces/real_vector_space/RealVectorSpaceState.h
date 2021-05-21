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
		int getDimensions() const;
		void setDimensions(int dimensions);
		const Eigen::VectorXf &getCoord() const;
		void setCoord(const Eigen::VectorXf &coord);
	};
}

#endif //RPMPL_REALVECTORSPACESTATE_H
