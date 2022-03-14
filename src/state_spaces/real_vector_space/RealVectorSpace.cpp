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
				collision = collisionCapsuleToCuboid(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
                // collision = collisionCapsuleToSphere(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
            }
            if (collision)	// The collision occurs
			{    
                return true;
            }
        }
    }
    return false;
}

// Check collision between capsule (determined with line segment AB and 'radius') and cuboid (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
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

// Check collision between capsule (determined with line segment AB and 'radius') and rectangle (determined with 'obs',
// where 'coord' determines which coordinate is constant: {0,1,2,3,4,5} = {x_min, y_min, z_min, x_max, y_max, z_max}
bool base::RealVectorSpace::collisionCapsuleToRectangle(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, 
														Eigen::VectorXf &obs, int coord)
{
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
	if (radius > 0)
	{
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

const Eigen::Vector3f base::RealVectorSpace::get3DPoint(const Eigen::Vector2f &point, float coord_value, int coord)
{
	Eigen::Vector3f point_new;
	point_new << point.head(coord), coord_value, point.tail(2-coord);
	return point_new;
}

// Check collision between two line segments, AB and CD
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

bool base::RealVectorSpace::collisionCapsuleToSphere(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{

}

// ------------------------------------------------------------------------------------------------------------------------------- //

float base::RealVectorSpace::getDistance(const std::shared_ptr<base::State> q)
{
	return std::get<0>(getDistanceAndPlanes(q));
}

std::tuple<float, std::shared_ptr<std::vector<Eigen::MatrixXf>>> base::RealVectorSpace::getDistanceAndPlanes(const std::shared_ptr<base::State> q)
{
    Eigen::MatrixXf distances(robot->getParts().size(), env->getParts().size());
	std::shared_ptr<std::vector<Eigen::MatrixXf>> planes = std::make_shared<std::vector<Eigen::MatrixXf>>
		(std::vector<Eigen::MatrixXf>(env->getParts().size(), Eigen::MatrixXf(6, robot->getParts().size())));
	std::shared_ptr<Eigen::VectorXf> plane;
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
                tie(distances(i,j), plane) = distanceCapsuleToCuboid(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
            }
			else if (env->getParts()[j]->getNodeType() == fcl::NODE_TYPE::GEOM_SPHERE)
			{
                // tie(distances(i,j), plane) = distanceCapsuleToSphere(XYZ->col(i), XYZ->col(i+1), robot->getRadius(i), obs);
            }
			planes->at(j).col(i) = *plane;
            if (distances(i,j) <= 0)		// The collision occurs
                return {0, nullptr};
        }
    }
	return {distances.minCoeff(), planes};
}

// Get distance (and plane) between line segment AB and cuboid (determined with 'obs = (x_min, y_min, z_min, x_max, y_max, z_max)')
std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceCapsuleToCuboid
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{
	Capsule_Cuboid capsule_cuboid(A, B, radius, obs);
	return capsule_cuboid.getDistance();
}

// Get distance (and plane) between two line segments, AB and CD
std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceLineSegToLineSeg
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C, const Eigen::Vector3f &D)
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
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C)
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

std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::distanceCapsuleToSphere
	(const Eigen::Vector3f &A, const Eigen::Vector3f &B, float radius, Eigen::VectorXf &obs)
{

}

// ------------------------------------------------ Class LineSeg_Cuboid -------------------------------------------------------//
base::RealVectorSpace::Capsule_Cuboid::Capsule_Cuboid(const Eigen::Vector3f &A_, const Eigen::Vector3f &B_, float radius_, Eigen::VectorXf &obs_)
{
	A = A_;
	B = B_;
	AB << A_, B_;
	radius = radius_;
	obs = obs_;
}

std::tuple<float, std::shared_ptr<Eigen::VectorXf>> base::RealVectorSpace::Capsule_Cuboid::getDistance()
{
	projectionLineSegOnSide(1, 2, 0, 4, 5, 3);   // Projection on x_min or x_max
	projectionLineSegOnSide(0, 2, 1, 3, 5, 4);   // Projection on y_min or y_max
	projectionLineSegOnSide(0, 1, 2, 3, 4, 5);   // Projection on z_min or z_max     
	if (d_c == 0)
		return {d_c, plane};

	int M = (projections.col(0) + projections.col(1)).maxCoeff();
	if (M > 0) 						// Projection of one or two points exists
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
		
		plane = std::make_shared<Eigen::VectorXf>(6);
		*plane << P1, P2 - P1;
		
		if (M == 1 && idx == 0)   	// Projection of 'A' exists, but projection of 'B' does not exist
			checkEdges(B, idx);
		else if (M == 1)           	// Projection of 'B' exists, but projection of 'A' does not exist
			checkEdges(A, idx);
	}
	else							// There is no projection of any point
		checkOtherCases();
				
	return {d_c - radius, plane};
}

void base::RealVectorSpace::Capsule_Cuboid::projectionLineSegOnSide(int min1, int min2, int min3, int max1, int max2, int max3)
{
	// 'min3' and 'max3' corresponds to the coordinate which is constant
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

void base::RealVectorSpace::Capsule_Cuboid::checkEdges(Eigen::Vector3f &point, int k)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments;
	if (projections(0, k))  		// Projection on x_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 0))
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(0), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(3, k))  	// Projection on x_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 3))           
			line_segments = getLineSegments(Eigen::Vector2f(point(1), point(2)), obs(1), obs(2), obs(4), obs(5), obs(3), 0);
		else
		{
			d_c = 0;
			return;
		}
	}				
	else if (projections(1, k))  	// Projection on y_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 1))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(1), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(4, k))  	// Projection on y_max
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 4))           
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(2)), obs(0), obs(2), obs(3), obs(5), obs(4), 1);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(2, k))  	// Projection on z_min
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 2))
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(2), 2);
		else
		{
			d_c = 0;
			return;
		}
	}
	else if (projections(5, k))  	// Projection on z_max 
	{
		if (!collisionCapsuleToRectangle(A, B, 0, obs, 5))            
			line_segments = getLineSegments(Eigen::Vector2f(point(0), point(1)), obs(0), obs(1), obs(3), obs(4), obs(5), 2);
		else
		{
			d_c = 0;
			return;
		}
	}

	distanceToMoreLineSegments(*line_segments);
}

std::shared_ptr<Eigen::MatrixXf> base::RealVectorSpace::Capsule_Cuboid::getLineSegments
	(const Eigen::Vector2f &point, float min1, float min2, float max1, float max2, float coord_value, int coord)
{
	std::shared_ptr<Eigen::MatrixXf> line_segments = std::make_shared<Eigen::MatrixXf>(3, 2);
	if (point(0) < min1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
	}
	else if (point(0) > max1)
	{
		line_segments->col(0) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
		line_segments->col(1) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
	}
				
	if (point(1) < min2)
	{
		line_segments->conservativeResize(3, 4);
		line_segments->col(2) = get3DPoint(Eigen::Vector2f(min1, min2), coord_value, coord);
		line_segments->col(3) = get3DPoint(Eigen::Vector2f(max1, min2), coord_value, coord);
	}
	else if (point(1) > max2)
	{
		line_segments->conservativeResize(3, 4);
		line_segments->col(2) = get3DPoint(Eigen::Vector2f(min1, max2), coord_value, coord);
		line_segments->col(3) = get3DPoint(Eigen::Vector2f(max1, max2), coord_value, coord);
	}
	return line_segments;	
}
	
void base::RealVectorSpace::Capsule_Cuboid::distanceToMoreLineSegments(const Eigen::MatrixXf &line_segments)
{
	float d_c_temp;
	std::shared_ptr<Eigen::VectorXf> plane_temp;
	for (int k = 0; k < line_segments.cols(); k += 2)
	{
		tie(d_c_temp, plane_temp) = distanceLineSegToLineSeg(A, B, line_segments.col(k), line_segments.col(k+1));
		if (d_c_temp == 0)
		{
			d_c = 0;
			return;
		}
		else if (d_c_temp < d_c)
		{
			d_c = d_c_temp;
			plane = plane_temp;
		}
	}
}

void base::RealVectorSpace::Capsule_Cuboid::checkOtherCases()
{
	if (A(0) < obs(0) && B(0) < obs(0))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < y_min, < z_min
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, < y_min, > z_max
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)));
			else    									// < x_min, < y_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 Eigen::Vector3f(obs(0), obs(1), obs(5)));
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, > y_max, < z_min
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > y_max, > z_max
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min, > y_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 Eigen::Vector3f(obs(0), obs(4), obs(5)));
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < x_min, < z_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 Eigen::Vector3f(obs(0), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < x_min, > z_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 Eigen::Vector3f(obs(0), obs(4), obs(5)));
			else    									// < x_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), obs(0), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	////////////////////////////////////////
	else if (A(0) > obs(3) && B(0) > obs(3))
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < y_min, < z_min
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, < y_min, > z_max
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)));
			else    									// > x_max, < y_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 Eigen::Vector3f(obs(3), obs(1), obs(5)));                    
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, > y_max, < z_min
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > y_max, > z_max
				tie(d_c, plane) = distanceLineSegToPoint(A, B, Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max, > y_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(4), obs(2)), 
																 Eigen::Vector3f(obs(3), obs(4), obs(5)));                     
		}
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > x_max, < z_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(2)),
																 Eigen::Vector3f(obs(3), obs(4), obs(2)));
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > x_max, > z_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(3), obs(1), obs(5)), 
																 Eigen::Vector3f(obs(3), obs(4), obs(5)));
			else    									// > x_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), obs(3), 
								 obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), 
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
		}
	}
	///////////////////////////////////////
	else
	{
		if (A(1) < obs(1) && B(1) < obs(1))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < y_min, < z_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(2)), 
																 Eigen::Vector3f(obs(3), obs(1), obs(2))); 
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// < y_min, > z_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(1), obs(5)), 
																 Eigen::Vector3f(obs(3), obs(1), obs(5))); 
			else    									// < y_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1), obs(1),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);                        
			}
		}
		else if (A(1) > obs(4) && B(1) > obs(4))
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// > y_max, < z_min
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(2)), 
																 Eigen::Vector3f(obs(3), obs(4), obs(2)));                        
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > y_max, > z_max
				tie(d_c, plane) = distanceLineSegToLineSeg(A, B, Eigen::Vector3f(obs(0), obs(4), obs(5)),
																 Eigen::Vector3f(obs(3), obs(4), obs(5)));                             
			else    									// > y_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0),
								 obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(2);
				distanceToMoreLineSegments(line_segments);  
			}
		}					                    
		else
		{
			if (A(2) < obs(2) && B(2) < obs(2)) 		// < z_min
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2);
				distanceToMoreLineSegments(line_segments);
			}
											
			else if (A(2) > obs(5) && B(2) > obs(5)) 	// > z_max
			{
				Eigen::MatrixXf line_segments(3, 8);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), 
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), 
								 obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5);
				distanceToMoreLineSegments(line_segments);  
			}
					
			else
			{
				for (int kk = 0; kk < 6; kk++)    		// Check collision with all sides
				{
					if (collisionCapsuleToRectangle(A, B, 0, obs, kk))
					{
						d_c = 0;
						return;
					}
				}
				Eigen::MatrixXf line_segments(3, 24);
				line_segments << obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0), obs(0), obs(0), obs(0), obs(3), obs(3), obs(3), obs(3), obs(0), obs(0),
								 obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4), obs(1), obs(1), obs(1), obs(1), obs(1), obs(4), obs(4), obs(4), obs(4),
								 obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(2), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5), obs(2), obs(5);
				distanceToMoreLineSegments(line_segments);
			}      
		}
	}
}
// -----------------------------------------------------------------------------------------------------------------------------//
