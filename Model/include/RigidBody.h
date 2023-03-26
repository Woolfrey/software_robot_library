    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Pose.h>                                                                                   // Custom class

class RigidBody
{
	public:   
		// Empty constructor, delegate default values
		RigidBody()
		: 
		RigidBody(0.0,
		          Eigen::Matrix3f::Zero(),
		          Pose(Eigen::Vector3f::Zero(),Eigen::Quaternionf(1,0,0,0)),
		          "unnamed") {}
		
		// Full constructor
		RigidBody(const float           &mass,
		          const Eigen::Matrix3f &inertia,
		          const Pose            &centreOfMass,
		          const std::string     &name);
		      
		// Functions    
		
		float mass()              const { return this->_mass; }
		
		Eigen::Matrix3f inertia() const { return this->_inertia; }
		
		Eigen::Vector3f com()     const { return this->_com; }                              // Get the center of mass
	
		std::string name()        const { return this->_name; }
		
	protected:
	          
		Eigen::Matrix3f _inertia;
		
		Eigen::Vector3f _com;
		
		float _mass;
		
		std::string _name;
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
