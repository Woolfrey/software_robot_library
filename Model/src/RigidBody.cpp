#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const float &mass,
                     const Eigen::Matrix3f &momentOfInertia,
                     const Pose &centreOfMass)
                     :
                     _mass(mass),
                     _momentOfInertia(momentOfInertia),
                     _centreOfMass(centreOfMass)
{	
	std::string message = "[ERROR] [RIGID BODY] Constructor: ";
	if(mass <= 0.0)
	{
		message += "Mass must be greater than 0, but your input was " + std::to_string(mass) + "kg.";
		
		throw std::invalid_argument(message);
	}
	else if((momentOfInertia - momentOfInertia).norm() < 1e-04)
	{
		throw std::invalid_argument(message + "Moment of inertia was not symmetric.");
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
