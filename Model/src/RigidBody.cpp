#include <Joint.h>
#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const float           &mass,
                     const Eigen::Matrix3f &inertia,
                     const Eigen::Vector3f &centreOfMass)
                     :
                     _mass(mass),
                     _centreOfMass(centreOfMass)
{	
	if(mass < 0.0)
	{
		throw std::invalid_argument("[ERROR] [RIGID BODY] Constructor: Mass argument was " + std::to_string(mass) + " but cannot be negative.");
	}
	else if((inertia - inertia.transpose()).norm() > 1e-04)
	{
		throw std::invalid_argument("[ERROR] [RIGID BODY] Constructor: Moment of inertia does not appear to be symmetric.");
	}
}
