    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Pose.h>                                                                                   // Position & orientation
#include <iostream>

class RigidBody
{
	public:
		// Constructor(s)
		RigidBody() {}                                                                      // Empty constructory

		RigidBody(const Pose            &origin,                                            // Origin relative to some other reference frame
		          const Eigen::Vector3f &centreOfMass,                                      // Center of mass relative to own origin frame
		          const float           &weight,                                             // kilograms
		          const Eigen::Matrix3f &momentOfInertia,                                  // N*m^2
                  const std::string     &name = "" );
			
		// Functions
		        
		Eigen::Vector3f com() const { return this->_com; }
		
		Eigen::Matrix3f inertia() const { return this->_globalInertia; }
		
		Eigen::Matrix3f inertiaDerivative() const { return this->_inertiaDerivative; }
		
		float mass() const { return this->_mass; }
		
		Pose pose() const { return this->_pose; }

		void update_state(const Pose            &origin,
		                  const Eigen::Vector3f &linearVel,
		                  const Eigen::Vector3f &angularVel);
		
	protected:

        std::string _name;
	
		// Kinematic properties
		Pose _pose;                                                                         // Pose of object relative to some local frame
		Eigen::Vector3f _linearVelocity  = {0.0 , 0.0 , 0.0};
		Eigen::Vector3f _angularVelocity = {0.0 , 0.0 , 0.0};
				
		// Dynamic Properties
		Eigen::Matrix3f _localInertia;                                                       // Inertia in local frame
		Eigen::Matrix3f _globalInertia;                                                      // Inertia in global frame
		Eigen::Matrix3f _inertiaDerivative;                                                  // Time-derivative of inertia
		Eigen::Vector3f _com = {0.5, 0.0, 0.0};                                             // Location of center of mass in local frame
		float _mass = 1.0;                                                                  // Mass of the rigid body
};                                                                                                  // Semicolon needed after a class declaration

#endif
