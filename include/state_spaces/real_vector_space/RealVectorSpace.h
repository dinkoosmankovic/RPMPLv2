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
		static bool collisionCapsuleToCuboid(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		static bool collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs, int coord);
		static bool collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D);
		static float checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, Eigen::Vector2f &point, float obs_coord, int coord);
		static const Eigen::Vector3f get3DPoint(const Eigen::Vector2f &point, float coord_value, int coord);
		static bool collisionCapsuleToSphere(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceCapsuleToCuboid
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceLineSegToLineSeg
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C, const Eigen::Vector3f &D);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceLineSegToPoint
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C);
		static std::tuple<float, std::shared_ptr<Eigen::MatrixXf>> distanceCapsuleToSphere
			(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs);
		class Capsule_Cuboid
		{
			private:
			float d_c = INFINITY;
    		std::shared_ptr<Eigen::MatrixXf> nearest_pts = std::make_shared<Eigen::MatrixXf>(3, 2);
			Eigen::Vector3f A, B, P1, P2;
			Eigen::MatrixXi projections = Eigen::MatrixXi::Zero(6, 2);			// Determines whether projections on obs exist. First column is for point 'A', and second is for point 'B'
			Eigen::Vector2f dist_AB_obs = Eigen::Vector2f(INFINITY, INFINITY);	// Distances of 'A' and 'B' to 'obs' (if projections exist)
			Eigen::MatrixXf AB = Eigen::MatrixXf(3, 2);							// Contains points 'A' and 'B'
			Eigen::VectorXf obs;
			float radius;
			void projectionLineSegOnSide(int min1, int min2, int min3, int max1, int max2, int max3);
			void checkEdges(Eigen::Vector3f &point, int k);
			std::shared_ptr<Eigen::MatrixXf> getLineSegments(const Eigen::Vector2f &point, float min1, float min2, float max1, float max2, 
															 float coord_value, int coord);
			void distanceToMoreLineSegments(const Eigen::MatrixXf &line_segments);
			void checkOtherCases();

			public:
			Capsule_Cuboid(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, float radius_, Eigen::VectorXf &obs_);
			void compute();
			float getDistance() { return d_c; }
			std::shared_ptr<Eigen::MatrixXf> getNearestPoints() { return nearest_pts; }
		};
		
	};
}
#endif //RPMPL_REALVECTORSPACE_H
