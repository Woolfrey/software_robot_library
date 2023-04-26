#include <Joint.h>
#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const float           &mass,
                     const Eigen::Matrix3f &inertia,
                     const Pose            &centreOfMass)
                     :
                     _mass(mass),
                     _com(centreOfMass.pos())
{	
	std::string message = "[ERROR] [RIGID BODY] Constructor: ";
	if(mass < 0.0)
	{
		message += "Mass was " + std::to_string(mass) + " but cannot be negative.";
		throw std::invalid_argument(message);
	}
	else if((inertia - inertia.transpose()).norm() > 1e-04)
	{
		throw std::invalid_argument(message + "Moment of inertia does not appear to be symmetric.");
	}
	else
	{
		// Transform the inertia properties to the origin specified
		// by the Pose object
		
		Eigen::Matrix3f R = centreOfMass.quat().toRotationMatrix();
		
		Eigen::Matrix3f S; S <<            0 , -this->_com(2),  this->_com(1),
		                        this->_com(2),             0 , -this->_com(0),
		                       -this->_com(1),  this->_com(0),             0 ;
		                       
		this->_inertia = R*inertia*R.transpose() - mass*S*S;                                // From the parallel axis theorem
	}
}

/*
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Set new kinematic properties and update the dynamics                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
void RigidBody::update_state(const Pose &origin,
                             const Eigen::Vector3f &linearVel,
                             const Eigen::Vector3f &angularVel)
{

	// Set new state values
	this->_pose = origin;
	this->_linearVelocity = linearVel;
	this->_angularVelocity = angularVel;
	
	// Rotate inertia to new global reference frame
	Eigen::Matrix3f R = this->_pose.quat().toRotationMatrix();
	this->_globalInertia = R*this->_localInertia*R.transpose();
	
	// Compute time-derivative of the inertia
	Eigen::Matrix3f skew;
	skew <<                      0.0, -this->_angularVelocity(2),  this->_angularVelocity(1),
	        this->_angularVelocity(2),                       0.0, -this->_angularVelocity(0),
	       -this->_angularVelocity(1),  this->_angularVelocity(0),                       0.0;
	
	this->_inertiaDerivative = skew*this->_globalInertia;                                         // d/dt(I) = s(w)*I

}
*/
