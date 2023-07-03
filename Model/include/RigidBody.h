    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Frame.h>

class Joint;                                                                                        // Forward declaration since RigidBody and Joint are mutually referential

class RigidBody
{
	public:   
		// Constructor
		RigidBody(const float           &mass,
		          const Eigen::Matrix3f &inertia,
		          const Eigen::Vector3f &centerOfMass);
		      
		// Methods  
		
		float mass()              const { return this->_mass; }
		
		Eigen::Matrix3f inertia() const { return this->_inertia; }
		
		Eigen::Vector3f com()     const { return this->_centerOfMass; }                     // Get the center of mass
		
	
	protected:
	          
		float _mass = 0.0;                                               
		
		Eigen::Matrix3f _inertia = (Eigen::MatrixXf(3,3) << 0, 0, 0,
		                                                     0, 0, 0,
		                                                     0, 0, 0).finished();
		 
		Eigen::Vector3f _centerOfMass = {0,0,0};
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
