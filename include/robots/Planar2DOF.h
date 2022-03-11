//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"
#include <memory>
#include <vector>
#include <string>


namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF(std::string robot_desc);
		~Planar2DOF();
		std::shared_ptr<std::vector<KDL::Frame>> computeForwardKinematics(std::shared_ptr<base::State> q) override;
		std::shared_ptr<Eigen::MatrixXf> computeXYZ(std::shared_ptr<base::State> q) override;
		float computeStep(std::shared_ptr<base::State> q1, std::shared_ptr<base::State> q2, float fi, 
						  std::shared_ptr<Eigen::MatrixXf> XYZ) override;
		const KDL::Tree& getRobotTree() const;
		const std::vector<std::unique_ptr<fcl::CollisionObject>> &getParts() const override;
		void setState(std::shared_ptr<base::State> q_) override;
		const std::vector<std::vector<float>> &getLimits() const override;
		const int getDimensions() override;
		const float getRadius(int dim) override;
		void test();

	private:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
	
	private:
		std::vector<std::unique_ptr<fcl::CollisionObject>> parts_;
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		std::vector<std::vector<float>> limits_;
		std::vector<float> radii;		// Radii of all enclosing cylinders
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
