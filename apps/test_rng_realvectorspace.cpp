#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include "Scenario.h"
#include <iostream>

int main()
{
	scenario::Scenario scenario("data/planar_2dof/scenario_easy.yaml");

	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	for (size_t i = 0; i < 100; ++i)
	{
		std::shared_ptr<base::State> q_rand = ss->randomState();
		std::cout << q_rand->getCoord().transpose() << std::endl;
	}
	return 0;
}
