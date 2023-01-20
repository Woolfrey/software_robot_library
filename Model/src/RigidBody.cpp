#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const Pose            &origin,
                     const Eigen::Vector3f &centreOfMass,
                     const float           &_mass,
                     const Eigen::Matrix3f &momentOfInertia):
                     P(origin),
                     c(centreOfMass),
                     m(_mass),
                     H(momentOfInertia)
{		
	Eigen::Matrix3f R = this->P.quat.toRotationMatrix();                                        // Get rotation matrix
	this->I = R*this->H*R.transpose();                                                          // Rotate to global frame
	
	if(this->m < 0)
	{
		std::cerr << "[WARNING] [RIGID BODY] Constructor: "
                          << "Mass argument of " << _mass << " cannot be negative." << std::endl;
			  
		this->m *= -1;
	}
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Set new kinematic properties and update the dynamics                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
void RigidBody::update_state(const Pose            &pose,
                             const Eigen::Vector3f &linearVelocity,
                             const Eigen::Vector3f &angularVelocity)
{
	// Set new values
	this->P = pose;
	this->v = linearVelocity;
	this->w = angularVelocity;
	
	// Rotate inertia to new global reference frame
	Eigen::Matrix3f R = this->P.quat.toRotationMatrix();                                 
	this->I = R*this->H*R.transpose();
	
	// Compute time-derivative of the inertia
	Eigen::Matrix3f skew;
	skew <<          0.0, -this->w(2),  this->w(1),
	          this->w(2),         0.0, -this->w(0),
	         -this->w(1),  this->w(0),         0.0;
	
	this->Idot = skew*this->I;                                                                  // d/dt(I) = s(w)*I
}
