//
// Created by dinko on 7.3.21..
//

#ifndef RPMPL_REALVECTORSPACE_H
#define RPMPL_REALVECTORSPACE_H

#include <ostream>
#include "StateSpace.h"
#include "RealVectorSpaceState.h"

namespace base
{
	class RealVectorSpace : public base::StateSpace
	{
	public:
		int dimensions;
		RealVectorSpace(int dimensions_);
		RealVectorSpace(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, const std::shared_ptr<env::Environment> env_);
		virtual ~RealVectorSpace();
		int getDimensions() override;
		friend std::ostream &operator<<(std::ostream &os, const RealVectorSpace &space);

		virtual std::shared_ptr<base::State> randomState() override;
		virtual std::shared_ptr<base::State> newState(std::shared_ptr<base::State> q) override;
		virtual std::shared_ptr<base::State> newState(const Eigen::VectorXf &state) override;
		bool equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;

		std::shared_ptr<base::State> interpolate(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2, float step, float D) override;
		bool isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2) override;
		virtual bool isValid(const std::shared_ptr<base::State> q) override;
		virtual float getDistance(const std::shared_ptr<base::State> q) override;
		virtual std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> getDistanceAndPlanes(const std::shared_ptr<base::State> q) override;
		
	private:
		bool collisionCapsuleToCuboid(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		bool collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs, int coord);
		bool collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D);
		
		std::tuple<float, std::shared_ptr<Eigen::VectorXf>> distanceLineSegToCuboid
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::VectorXf &obs);
		std::tuple<float, std::shared_ptr<Eigen::VectorXf>> distanceLineSegToLineSeg
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D);
		std::tuple<float, std::shared_ptr<Eigen::VectorXf>> distanceLineSegToPoint
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C);
		
		float checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, Eigen::Vector2f &point, float obs_coord, int coord);
		const Eigen::Vector3f get3DPoint(const Eigen::Vector2f &point, float obs_coord, int coord);
	};
}
#endif //RPMPL_REALVECTORSPACE_H
