//
// Created by dinko on 07.02.22.
//

#ifndef RPMPL_PLANAR2DOF_H
#define RPMPL_PLANAR2DOF_H

#include "AbstractRobot.h"
#include <memory>
#include <vector>
#include <string>

#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <kdl_parser/kdl_parser.hpp>

namespace robots
{
	class Planar2DOF : public AbstractRobot
	{
	public:
		Planar2DOF(std::string robot_desc);
		~Planar2DOF();
		void computeForwardKinematics(std::shared_ptr<base::State> q) override;
		const KDL::Tree& getRobotTree() const; 
	
	private:

		std::vector<std::shared_ptr<fcl::CollisionGeometry> > part_meshes_;
  		std::vector<std::unique_ptr<fcl::CollisionObject> > parts_;
		KDL::Tree robot_tree;
		KDL::Chain robot_chain;
	};

}
#endif //RPMPL_ABSTRACTPLANNER_H
