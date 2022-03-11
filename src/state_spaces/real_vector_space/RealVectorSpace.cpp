//
// Created by dinko on 07.03.21.
// Modified by nermin on 07.03.22.
//
#include "RealVectorSpace.h"
#include "RealVectorSpaceState.h"
#include <ostream>
#include <Eigen/Dense>
#include "ConfigurationReader.h"
#include <glog/log_severity.h>
#include <glog/logging.h>

base::RealVectorSpace::RealVectorSpace(int dimensions_) : dimensions(dimensions_)
{
	setStateSpaceType(StateSpaceType::RealVectorSpace);
}

base::RealVectorSpace::RealVectorSpace(int dimensions_, const std::shared_ptr<robots::AbstractRobot> robot_, 
									   const std::shared_ptr<env::Environment> env_) : dimensions(dimensions_)
{
	srand((unsigned int) time(0));
	setStateSpaceType(StateSpaceType::RealVectorSpace);
	robot = robot_;
	env = env_;
}

base::RealVectorSpace::~RealVectorSpace()
{
}

int base::RealVectorSpace::getDimensions()
{
	return dimensions;
}

std::ostream &base::operator<<(std::ostream &os, const base::RealVectorSpace &space)
{
	os << " dimensions: " << space.dimensions;
	return os;
}

std::shared_ptr<base::State> base::RealVectorSpace::randomState()
{
	std::shared_ptr<base::State> state = std::make_shared<base::RealVectorSpaceState>(dimensions);
	state->setCoord(Eigen::VectorXf::Random(dimensions));
	return state;
}

std::shared_ptr<base::State> base::RealVectorSpace::newState(std::shared_ptr<base::State> state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

std::shared_ptr<base::State> base::RealVectorSpace::newState(const Eigen::VectorXf &state)
{
	std::shared_ptr<base::State> q = std::make_shared<base::RealVectorSpaceState>(state);
	return q;
}

bool base::RealVectorSpace::equal(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	float d = (q1->getCoord() - q2->getCoord()).norm();
	if (d < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
		return true;
	return false;
}

// D (optional parameter) is the distance between q1 and q2
std::shared_ptr<base::State> base::RealVectorSpace::interpolate(const std::shared_ptr<base::State> q1, 
																const std::shared_ptr<base::State> q2, float step, float D)
{
	std::shared_ptr<base::State> q_t = std::make_shared<base::RealVectorSpaceState>(dimensions);
	Eigen::VectorXf eig;
	if (D < 0) 	// D = -1 is the default obs_coord
	{
		D = (q2->getCoord() - q1->getCoord()).norm();
	}
	
	if (step < D)
	{
		eig = (q2->getCoord() - q1->getCoord()) / D;
		q_t->setCoord(q1->getCoord() + step * eig);
	}
	else
	{
		q_t->setCoord(q2->getCoord());
	}
	// here we check the validity of the motion q1->q_t
	if (isValid(q_t))
		return q_t;
	else
		return nullptr;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q1, const std::shared_ptr<base::State> q2)
{
	int numChecks = RealVectorSpaceConfig::NUM_INTERPOLATION_VALIDITY_CHECKS;
	float D = (q2->getCoord() - q1->getCoord()).norm();
	for (float t = 1./numChecks; t <= 1; t += 1./numChecks)
	{
		std::shared_ptr<base::State> q_t = interpolate(q1, q2, t, D);
		if (q_t == nullptr)
			return false;
	}
	return true;
}

bool base::RealVectorSpace::isValid(const std::shared_ptr<base::State> q)
{
	bool collision;
	std::shared_ptr<Eigen::MatrixXf> XYZ = robot->computeXYZ(q);
	for (int i = 0; i < robot->getParts().size(); i++)
	{
    	for (int j = 0; j < env->getParts().size(); j++)
		{
            if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
				collision = collisionCapsuleToCuboid((Eigen::Vector3f) XYZ->col(i), (Eigen::Vector3f) XYZ->col(i+1), robot->getRadius(i), obs);
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
                // collision = collisionRectangleToSphere((Eigen::Vector3f) xyz.col(i), (Eigen::Vector3f) xyz.col(i+1), 
                //             obstacles.obsation.col(j), robot.radii(i));
            }
            if (collision)	// The collision occurs
			{    
                return true;
            }
        }
    }
    return false;
}

bool base::RealVectorSpace::collisionCapsuleToCuboid(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
    bool collision = false;
    float r_new = radius * sqrt(2) / 2;

	if (A(0) > obs(0) - r_new && A(1) > obs(1) - r_new && A(2) > obs(2) - r_new &&
		A(0) < obs(3) + r_new && A(1) < obs(3) + r_new && A(2) < obs(4) + r_new ||
		B(0) > obs(0) - r_new && B(1) > obs(1) - r_new && B(2) > obs(2) - r_new &&
		B(0) < obs(3) + r_new && B(1) < obs(3) + r_new && B(2) < obs(4) + r_new)
		return true;
    else if (A(0) < obs(0) - radius && B(0) < obs(0) - radius || A(0) > obs(3) + radius && B(0) > obs(3) + radius ||
           	 A(1) < obs(1) - radius && B(1) < obs(1) - radius || A(1) > obs(4) + radius && B(1) > obs(4) + radius ||
           	 A(2) < obs(2) - radius && B(2) < obs(2) - radius || A(2) > obs(5) + radius && B(2) > obs(5) + radius)
		return false;
    
    if (A(0) < obs(0))      				// < x_min
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 0);
    else if (A(0) > obs(3))  				// > x_max
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 3);
    
    if (!collision && A(1) < obs(1))     	// < y_min
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 1);
    else if (!collision && A(1) > obs(4)) 	// > y_max
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 4);
    
    if (!collision && A(2) < obs(2))     	// < z_min
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 2);
    else if (!collision && A(2) > obs(5)) 	// > z_max
        collision = collisionCapsuleToRectangle(A, B, radius, obs, 5);
	
    return collision;
}

bool base::RealVectorSpace::collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, 
														float radius, Eigen::VectorXf &obs, int coord)
{
	// 'coord' determines which coordinate is constant: {0,1,2,3,4,5} = {x_min, y_min, z_min, x_max, y_max, z_max}
	float obs_coord = obs(coord);
    float r_new = radius * sqrt(2) / 2;
    if (coord > 2)
        coord -= 3;
    
	Eigen::Vector4f rec; rec << obs.head(coord), obs.segment(coord+1, 2-coord), obs.segment(3, coord), obs.tail(2-coord);
	Eigen::Vector2f A_rec; A_rec << A.head(coord), A.tail(2-coord);
	Eigen::Vector2f B_rec; B_rec << B.head(coord), B.tail(2-coord);
    float t = (obs_coord - A(coord)) / (B(coord) - A(coord));
	Eigen::Vector2f M = A_rec + t * (B_rec - A_rec);
	Eigen::Vector3f A_proj = get3DPoint(A_rec, obs_coord, coord);
	Eigen::Vector3f B_proj = get3DPoint(B_rec, obs_coord, coord);
    if (t > 0 && t < 1)
	{
		if (M(0) > rec(0) - radius && M(0) < rec(2) + radius && 
			M(1) > rec(1) - radius && M(1) < rec(3) + radius) 	// Whether point lies on the oversized rectangle
		{
			if (M(0) > rec(0) - radius && M(0) < rec(2) + radius && M(1) > rec(1) && M(1) < rec(3) || 
				M(1) > rec(1) - radius && M(1) < rec(3) + radius && M(0) > rec(0) && M(0) < rec(2) ||
				M(0) < rec(0) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(0), rec(2))).norm() < radius ||
				M(0) < rec(0) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(0), rec(3))).norm() < radius ||
				M(0) > rec(1) && M(1) < rec(2) && (M - Eigen::Vector2f(rec(1), rec(2))).norm() < radius ||
				M(0) > rec(1) && M(1) > rec(3) && (M - Eigen::Vector2f(rec(1), rec(3))).norm() < radius)
				return true;
		}
	}
	else if (std::min((A - A_proj).norm(), (B - B_proj).norm()) > radius)
		return false;

    // Considering collision between capsule and rectangle
	if (A_rec(0) > rec(0) && A_rec(0) < rec(2) && A_rec(1) > rec(1) && A_rec(1) < rec(3))
	{
		if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3)) 	// Both projections
		{
			if (std::min((A - A_proj).norm(), (B - B_proj).norm()) < radius)
				return true;
		}
		else
		{
			if (std::min(checkCases(A, B, rec, B_rec, obs_coord, coord), (A - A_proj).norm()) < radius)
				return true;
		}
	}
	else if (B_rec(0) > rec(0) && B_rec(0) < rec(2) && B_rec(1) > rec(1) && B_rec(1) < rec(3))
	{
		if (std::min(checkCases(A, B, rec, A_rec, obs_coord, coord), (B- B_proj).norm()) < radius)
			return true;
	}
	else
	{
		if (checkCases(A, B, rec, A_rec, obs_coord, coord) < radius)
			return true;
			
		if (checkCases(A, B, rec, B_rec, obs_coord, coord) < radius)
			return true;
	}
	return false;
}

float base::RealVectorSpace::checkCases(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector4f &rec, 
										Eigen::Vector2f &point, float obs_coord, int coord)
{
	float d_c1 = 0;
	float d_c2 = 0;
	if (point(0) < rec(0))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (point(0) > rec(2))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c1 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	
	if (d_c1 > 0 && point(1) < rec(1))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(1)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(1)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	else if (d_c1 > 0 && point(1) > rec(3))
	{
		Eigen::Vector3f C = get3DPoint(Eigen::Vector2f(rec(0), rec(3)), obs_coord, coord);
		Eigen::Vector3f D = get3DPoint(Eigen::Vector2f(rec(2), rec(3)), obs_coord, coord);
		d_c2 = std::get<0>(distanceLineSegToLineSeg(A, B, C, D));
	}
	
	return std::min(d_c1, d_c2);
}

const Eigen::Vector3f base::RealVectorSpace::get3DPoint(const Eigen::Vector2f &point, float obs_coord, int coord)
{
	Eigen::Vector3f point_new;
	point_new << point.head(coord), obs_coord, point.tail(2-coord);
	return point_new;
}

bool base::RealVectorSpace::collisionLineSegToLineSeg(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D)
{
    Eigen::Vector3f P1, P2;
    double alpha1 = (B - A).squaredNorm();
    double alpha2 = (B - A).dot(D - C);
    double beta1 = (C - D).dot(B - A);
    double beta2 = (C - D).dot(D - C);
    double gamma1 = (A - C).dot(A - B);
    double gamma2 = (A - C).dot(C - D);
    double s = (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1);
    double t = (gamma1 - beta1 * s) / alpha1;

    if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        P1 = C + s * (D - C);
        P2 = A + t * (B - A);
        if ((P1 - P2).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return true;
    }
    return false;
}




float base::RealVectorSpace::getDistance(const std::shared_ptr<base::State> q)
{
	return std::get<0>(getDistanceAndPlanes(q));
}

std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> base::RealVectorSpace::getDistanceAndPlanes(const std::shared_ptr<base::State> q)
{
    Eigen::MatrixXf distances(robot->getParts().size(), env->getParts().size());
	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	for (int i = 0; i < robot->getParts().size(); i++)
	{
    	for (int j = 0; j < env->getParts().size(); j++)
		{
            if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_BOX)
			{
				fcl::AABB AABB = env->getParts()[j]->getAABB();
				Eigen::VectorXf obs(6);
				obs << AABB.min_[0], AABB.min_[1], AABB.min_[2], AABB.max_[0], AABB.max_[1], AABB.max_[2];
                
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
                // distances(i,j) = distanceRectangleToSphere((Eigen::Vector3f) xyz.col(i), (Eigen::Vector3f) xyz.col(i+1), 
                //             obstacles.obsation.col(j), robot.radii(i));
            }
            distances(i,j) -=  robot->getRadius(i);
            if (distances(i,j) <= 0)		// The collision occurs
                return {0, nullptr};
        }
    }
	return {distances.minCoeff(), planes};
}

std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceLineSegToCuboid
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::VectorXf &obs)
{
	class LineSeg_Cuboid
	{
		public:
		float d_c = INFINITY;  
		std::shared_ptr<Eigen::VectorXf> plane = std::make_shared<Eigen::VectorXf>(6);
		Eigen::Vector3f A, B, P1, P2;
		Eigen::MatrixXi projections = Eigen::MatrixXi::Zero(6, 2);			// Determines whether projections on obs exist. First column is for point 'A', and second is for point 'B'
		Eigen::Vector2f dist_AB_obs = Eigen::Vector2f(INFINITY, INFINITY);	// Distances of 'A' and 'B' to obs (if projections exist)
		Eigen::MatrixXf AB = Eigen::MatrixXf(3, 2);
		Eigen::VectorXf obs;
		
		LineSeg_Cuboid(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, Eigen::VectorXf &obs_)
		{
			A = A_;
			B = B_;
			AB << A_, B_;
			obs = obs_;
		}

		std::tuple<float, std::shared_ptr<Eigen::VectorXf>> getDistance()
		{
			projectionLineSegOnSide(1, 2, 0, 4, 5, 3);   // Projection on x_min or x_max
			projectionLineSegOnSide(0, 2, 1, 3, 5, 4);   // Projection on y_min or y_max
			projectionLineSegOnSide(0, 1, 2, 3, 4, 5);   // Projection on z_min or z_max     
			if (d_c == 0)
				return {0, nullptr};

			int M = (projections.col(0) + projections.col(1)).maxCoeff();
			if (M > 0) 		// Projection of one or two points exists
			{
				int idx = (dist_AB_obs(0) < dist_AB_obs(1)) ? 0 : 1;
				d_c = dist_AB_obs.minCoeff();
				P2 = AB.col(idx);
				Eigen::Index I;
				projections.col(idx).maxCoeff(&I);
				if (I == 0 || I == 3)
					P1 << obs(I), AB.col(idx).segment(1, 2);
				else if (I == 1 || I == 4)
					P1 << AB(0, idx), obs(I), AB(2, idx);
				else if (I == 2 || I == 5)
					P1 = AB.col(idx).head(2), obs(I);
				
				if (M == 1 && idx == 0)   // Projection of 'A' exists, but projection of 'B' does not exist
					checkEdges(B, idx);
				else if (M == 1)           // Projection of B exists, but projection of A does not exist
					checkEdges(A, idx);
			}
			else			// There is no projection of any point
				checkOtherCases();
			
			if (d_c > 0)
				*plane << P1, P2 - P1;
			
			return {d_c, plane};
		}

		void projectionLineSegOnSide(int min1, int min2, int min3, int max1, int max2, int max3)
		{
			// min3 and max3 corresponds to coordinate which is constant
			for (int i = 0; i < 2; i++)
			{
				if (AB(min1, i) > obs(min1) && AB(min1, i) < obs(max1) && AB(min2, i) > obs(min2) && AB(min2, i) < obs(max2))
				{
					if (AB(min3,i) > obs(min3) && AB(min3,i) < obs(max3))
					{
						d_c = 0;
						return;
					}
					else if (AB(min3, i) < obs(min3))
					{
						projections(min3, i) = 1;
						dist_AB_obs(i) = obs(min3) - AB(min3, i);
					}
					else if (AB(min3, i) > obs(max3))
					{
						projections(max3, i) = 1;
						dist_AB_obs(i) = AB(min3, i) - obs(max3);
					}
				}
			}
		}

		void checkEdges(Eigen::Vector3f &point, int k)
		{

		}

		void checkOtherCases()
		{

		}
	};
}

// Get distance (and plane) between two line segments, AB and CD
std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceLineSegToLineSeg
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C, Eigen::Vector3f &D)
{
    float d_c = INFINITY;
	std::shared_ptr<Eigen::VectorXf> plane = std::make_shared<Eigen::VectorXf>(6);
    Eigen::Vector3f P1, P2;
    double alpha1 = (B - A).squaredNorm();
    double alpha2 = (B - A).dot(D - C);
    double beta1 = (C - D).dot(B - A);
    double beta2 = (C - D).dot(D - C);
    double gamma1 = (A - C).dot(A - B);
    double gamma2 = (A - C).dot(C - D);
    double s = (alpha1 * gamma2 - alpha2 * gamma1) / (alpha1 * beta2 - alpha2 * beta1);
    double t = (gamma1 - beta1 * s) / alpha1;
	
	if (t > 0 && t < 1 && s > 0 && s < 1)
	{
        P1 = C + s * (D - C);
        P2 = A + t * (B - A);
		d_c = (P1 - P2).norm();
        if (d_c < RealVectorSpaceConfig::EQUALITY_THRESHOLD) 	// The collision occurs
            return {0, nullptr};
		*plane << P1, P2 - P1;
    }
    else
	{
		float d_c_temp;
		float var1 = 1 / (B-A).squaredNorm();
        float var2 = 1 / (C-D).squaredNorm();
        Eigen::Vector4f opt((A - C).dot(A - B) * var1,	// s = 0
							(A - D).dot(A - B) * var1,	// s = 1
							(A - C).dot(D - C) * var2,	// t = 0
							(B - C).dot(D - C) * var2);	// t = 1
		
        for (int i = 0; i < 3; i++)
		{
            if (opt(i) < 0)
                if (i == 0 || i == 2)     	// s = 0, t = 0
				{
					P1 = C; 
					P2 = A;
				}
                else if (i == 1)      		// s = 1, t = 0
				{
                    P1 = D; 
					P2 = A;
				}
                else                      	// t = 1, s = 0
				{
                    P1 = C; 
					P2 = B;
				}
            else if (opt(i) > 0)
			{
				if (i == 1 || i == 3)    	// s = 1, t = 1
				{
					P1 = D; 
					P2 = B;
				}
                else if (i == 0)        	// s = 0, t = 1
				{
					P1 = C; 
					P2 = B;
				}                    
                else                    	// t = 0, s = 1
				{
					P1 = D; 
					P2 = A;
				}
			}
            else
			{
				if (i == 0)                	// s = 0, t € [0,1]
				{
					P1 = C; 
					P2 = A + opt(i) * (B - A);
				}                    
                else if (i == 1)       		// s = 1, t € [0,1]
				{
                    P1 = D; 
					P2 = A + opt(i) * (B - A);
				}
                else if (i == 2)           	// t = 0, s € [0,1]
				{
                    P1 = C + opt(i) * (D - C); 
					P2 = A;
				}
                else                       	// t = 1, s € [0,1]
				{
                    P1 = C + opt(i) * (D - C); 
					P2 = B;
				}
			}
            
            d_c_temp = (P1 - P2).norm();
            if (d_c_temp < d_c)
			{
                d_c = d_c_temp;
                *plane << P1, P2 - P1;
			}
        }
    }
	return {d_c, plane};
}

// Get distance (and plane) between line segment AB and point C
std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceLineSegToPoint
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, Eigen::Vector3f &C)
{
	std::shared_ptr<Eigen::VectorXf> plane = std::make_shared<Eigen::VectorXf>(6);
	float d_c;
    Eigen::Vector3f P1 = C;
    Eigen::Vector3f P2;
    float t_opt = (C - A).dot(B - A) / (B - A).squaredNorm();
    if (t_opt < 0)
	{	
		d_c = (C - A).norm();
		P2 = A;
	}
    else if (t_opt > 1)
	{
		d_c = (C - B).norm();
        P2 = B;
	}
    else
	{
		P2 = A + t_opt * (B - A);
        if ((C - P2).norm() < RealVectorSpaceConfig::EQUALITY_THRESHOLD)
            return {0, nullptr};
	}
    *plane << P1, P2 - P1;
	return {d_c, plane};
}

