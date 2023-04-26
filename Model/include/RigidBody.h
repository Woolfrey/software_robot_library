    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Pose.h>                                                                                   // Custom class

class Joint;                                                                                        // Forward declaration since RigidBody and Joint are mutually referential

class RigidBody
{
	public:   
		// Constructor
		RigidBody(const float           &mass,
		          const Eigen::Matrix3f &inertia,
		          const Pose            &centreOfMass);
		      
		// Functions    
		
		float mass()              const { return this->_mass; }
		
		Eigen::Matrix3f inertia() const { return this->_inertia; }
		
		Eigen::Vector3f com()     const { return this->_com; }                              // Get the center of mass
		
	protected:
	          
		Eigen::Matrix3f _inertia = (Eigen::MatrixXf(3,3) << 0, 0, 0,
		                                                    0, 0, 0,
		                                                    0, 0, 0).finished();
		
		Eigen::Vector3f _com = {0,0,0};
		
		float _mass = 0.0;
};                                                                                                  // Semicolon needed after a class declaration

#endif
