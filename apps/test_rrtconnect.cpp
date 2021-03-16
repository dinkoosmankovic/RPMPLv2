#include <RRTConnect.h>
#include <iostream>
#include <RealVectorSpace.h>

int main()
{
	//base::RealVectorSpace *space = new base::RealVectorSpace(2);
	base::StateSpace *ss = new base::RealVectorSpace(2);
	planning::rrt::RRTConnect* planner = new planning::rrt::RRTConnect(ss);
	planner->solve();
	return 0;
}
