#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const Pose            &origin,
                     const Eigen::Vector3f &centreOfMass,
                     const float           &weight,
                     const Eigen::Matrix3f &momentOfInertia):
                     _pose(origin),
                     _com(centreOfMass),
                     _mass(weight),
                     localInertia(momentOfInertia)
{		
	Eigen::Matrix3f R = this->_pose.quat().toRotationMatrix();                                  // Convert to SO(3)
	
	this->globalInertia = R*this->localInertia*R.transpose();                                   // Rotate to global frame

	if(this->_mass < 0)
	{
		std::cerr << "[WARNING] [RIGID BODY] Constructor: "
                          << "Mass of object was " << this->_mass << ", "
                          << "but value cannot be negative." << std::endl;
			  
		this->_mass *= -1;                                                                  // Ensure positive
	}
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Set new kinematic properties and update the dynamics                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
void RigidBody::update_state(const Pose            &origin,
                             const Eigen::Vector3f &linearVel,
                             const Eigen::Vector3f &angularVel)
{
	// Set new state values
	this->_pose = origin;
	this->linearVelocity = linearVel;
	this->angularVelocity = angularVel;
	
	// Rotate inertia to new global reference frame
	Eigen::Matrix3f R = this->_pose.quat().toRotationMatrix();
	this->globalInertia = R*this->localInertia*R.transpose();
	
	// Compute time-derivative of the inertia
	Eigen::Matrix3f skew;
	skew <<                      0.0, -this->angularVelocity(2),  this->angularVelocity(1),
	        this->angularVelocity(2),                       0.0, -this->angularVelocity(0),
	       -this->angularVelocity(1),  this->angularVelocity(0),                       0.0;
	
	this->inertiaDerivative = skew*this->globalInertia;                                         // d/dt(I) = s(w)*I
}
