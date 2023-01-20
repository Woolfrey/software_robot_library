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
		          const float           &_mass,                                             // kilograms
		          const Eigen::Matrix3f &momentOfInertia);                                  // N*m^2
			
		// Functions
		        
		Eigen::Vector3f com() const { return this->c; }
		
		Eigen::Matrix3f inertia() const { return this->I; }
		
		Eigen::Matrix3f inertia_derivative() const { return this->Idot; }
		
		float mass() const { return this->m; }
		
		Pose pose() const { return this->P; }

		void update_state(const Pose            &_pose,
		                  const Eigen::Vector3f &linearVelocity,
		                  const Eigen::Vector3f &angularVelocity);
		
	private:
	
		// Kinematic properties
		Pose P;                                                                             // Pose of object relative to some local frame
		Eigen::Vector3f v = {0.0 , 0.0 , 0.0};
		Eigen::Vector3f w = {0.0 , 0.0 , 0.0};
				
		// Dynamic Properties
		Eigen::Matrix3f H;                                                                  // Inertia in local frame
		Eigen::Matrix3f I;                                                                  // Inertia in global frame
		Eigen::Matrix3f Idot;                                                               // Time-derivative of inertia
		Eigen::Vector3f c = {0.5, 0.0, 0.0};                                                // Location of center of mass in local frame
		float m = 1.0;                                                                      // Mass of the rigid body
};                                                                                                  // Semicolon needed after a class declaration

#endif
