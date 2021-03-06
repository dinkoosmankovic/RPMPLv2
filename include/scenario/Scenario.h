//
// Created by dinko on 1.02.22.
//

#ifndef RPMPL_SCENARIO_H
#define RPMPL_SCENARIO_H

#include <vector> 
#include <memory>
#include "AbstractRobot.h"
#include "Environment.h"
#include "State.h"
#include <string>
#include <StateSpace.h>

#include "fcl/fcl.h"

namespace scenario
{
	class Scenario
	{
	public:
		Scenario(std::string configuration_file);
        const std::shared_ptr<robots::AbstractRobot> &getRobot() const { return robot; }		
        const std::shared_ptr<env::Environment> &getEnvironment() const { return env; }
        const std::shared_ptr<base::StateSpace> &getStateSpace() const { return ss; }
        const std::shared_ptr<base::State> &getStart() const { return start; }
        const std::shared_ptr<base::State> &getGoal() const { return goal; }
        const std::string &getSpaceType() const { return spaceType; }
        const int &getDimensions() const { return dimensions; }

	private:
		std::shared_ptr<robots::AbstractRobot> robot;
        std::shared_ptr<env::Environment> env;
        std::shared_ptr<base::State> start;
        std::shared_ptr<base::State> goal;
        std::string spaceType;
        int dimensions;
        std::shared_ptr<base::StateSpace> ss;
	};
}
#endif //RPMPL_ABSTRACTPLANNER_H