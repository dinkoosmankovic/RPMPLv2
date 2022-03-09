//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"
#include "Environment.h"
#include <memory>
#include <vector>
#include <string>

#include <fcl/broadphase/broadphase.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF(std::string robot_desc);
		~Planar2DOF();
		std::vector<KDL::Frame> computeForwardKinematics(std::shared_ptr<base::State> q);
		const KDL::Tree& getRobotTree() const;
		const std::vector<std::unique_ptr<fcl::CollisionObject> >& getParts() const override;
		void setState(std::shared_ptr<base::State> q_) override;
		void test(std::shared_ptr<env::Environment> env, std::shared_ptr<base::State> q);

		std::vector<robots::LinkLimits> getLimits() const override;

	private:
		fcl::Transform3f KDL2fcl(const KDL::Frame &in);
		KDL::Frame fcl2KDL(const fcl::Transform3f &in);
		fcl::Vec3f transformPoint(fcl::Vec3f& v, fcl::Transform3f t);
	
	private:
		std::vector<std::unique_ptr<fcl::CollisionObject> > parts_;
		std::vector<KDL::Frame> init_poses;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
		std::vector<robots::LinkLimits> limits_;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
