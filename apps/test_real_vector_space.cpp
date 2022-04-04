#include <iostream>
#include <RealVectorSpaceFCL.h>
#include <Environment.h>
#include <Planar2DOF.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <RGBTConnect.h>

#include <glog/logging.h>


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";
	// std::shared_ptr<robots::Planar2DOF> robot = std::make_shared<robots::Planar2DOF>("data/planar_2dof/planar_2dof.urdf");
	// std::shared_ptr<env::Environment> env = std::make_shared<env::Environment>("data/planar_2dof/obstacles_easy.yaml" );
	// std::shared_ptr<base::StateSpace> ss = std::make_shared<base::RealVectorSpaceFCL>(2, robot, env);
	// std::shared_ptr<base::State> start = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({-M_PI/2, 0}));
	// std::shared_ptr<base::State> goal = std::make_shared<base::RealVectorSpaceState>(Eigen::Vector2f({M_PI/2, 0}));

	ConfigurationReader::initConfiguration();
	// scenario::Scenario scenario("data/planar_2dof/scenario_test.yaml");
	// scenario::Scenario scenarioFCL("data/planar_2dof/scenario1.yaml");
	scenario::Scenario scenario("data/xarm6/scenario2.yaml");
	// scenario::Scenario scenarioFCL("data/xarm6/scenario1.yaml");

	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
	// std::shared_ptr<base::StateSpace> ssFCL = scenarioFCL.getStateSpace();
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();
	try
	{
		std::unique_ptr<planning::rbt::RGBTConnect> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, scenario.getStart(), scenario.getGoal());
		float d_c;
		std::shared_ptr<std::vector<Eigen::MatrixXf>> planes;
		int num = 0;
		while (num++ < 1)
		{
			// std::shared_ptr<base::State> q = ss->randomState();
			Eigen::VectorXf Q(6); Q << 1.49469,   1.81164,  -3.35258,  0.626211, -0.117099, -0.466912;
			std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(Q);
			std::cout << "Num: " << num << " Configuration: " << q << std::endl;
			
			// if (ss->isValid(q))
			// {
			// 	tie(d_c, planes) = ss->getDistanceAndPlanes(q);
			// 	float d_c_under = planner->getDistanceUnderestimation(q, planes);
			// 	if (abs(d_c - d_c_under) > 1e-3)
			// 	{
			// 		std::cout << "************************ different ************************" << std::endl;
			// 		std::cout << "d_c = " << d_c << std::endl;
			// 		std::cout << "d_c_under = " << d_c_under << std::endl;
			// 		throw;
			// 	}
			// }
			// else
			// 	std::cout << "invalid " << std::endl;
			

			// std::cout << "-------------------- WITHOUT FCL --------------------" << std::endl;
			// bool valid = ss->isValid(q);
			// std::cout << "Is valid: " << (valid ? "true" : "false") << std::endl;
			// float d_c = ss->getDistance(q); if (d_c == 0) d_c = -1;
			// std::cout << "Distance: " << d_c << std::endl;

			// std::cout << "-------------------- WITH FCL -----------------------" << std::endl;
			// bool validFCL = ssFCL->isValid(q);
			// std::cout << "Is valid: " << (validFCL ? "true" : "false") << std::endl;
			// float d_cFCL = ssFCL->getDistance(q);
			// std::cout << "Distance: " << d_cFCL << std::endl;
			
			// if (valid != validFCL)
			// {
			// 	std::cout << "************ DIFFERENT ISVALID *********** " << std::endl;
			// 	throw;
			// }
			// if (std::abs(d_c - d_cFCL) > 1e-2)
			// {
			// 	std::cout << "************ DIFFERENT DISTANCE *********** " << std::endl;
			// 	throw;
			// }
			std::cout << std::endl;
		}
		std::cout << "Test completed successfully! " << std::endl;
	}
	catch (std::domain_error& e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
