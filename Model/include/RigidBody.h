    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Affine3f

class RigidBody
{
	public:
		RigidBody() {}                                                                      // Empty constructory
		
		RigidBody(const Eigen::Isometry3f &origin,                                          // Origin relative to some other frame
                         const Eigen::Vector3f &centreOfMass,                                       // Center of mass rleative to origin
                         const float &_mass,                                                        // Mass (kg)
                         const Eigen::VectorXf &momentOfInertia);                                   // Inertia (Nm^2)
			
		// Set Functions
		bool set_state(const Eigen::Isometry3f &origin,
                              const Eigen::Vector3f &linearVel,
                              const Eigen::Vector3f &angularVel);
		
		// Get Functions
		Eigen::Vector3f get_com() const {return this->com;}                                 // Get the position of the c.o.m. in local frame
		
		Eigen::Isometry3f get_pose() const {return this->pose;}                             // Get the pose relative to some local frame
		
		Eigen::Matrix3f get_inertia() const {return this->inertia;}
		
		float get_mass() const {return this->mass;}
		
	private:
		// Kinematic Properties
		Eigen::Isometry3f pose;                                                             // Pose of object relative to some local frame
		Eigen::Vector3f linearVel = {0.0 , 0.0 , 0.0};
		Eigen::Vector3f angularVel = {0.0 , 0.0 , 0.0};
				
		// Dynamic Properties
		Eigen::Matrix3f inertia;                                                            // Moment of inertia at the center of mass
		Eigen::Matrix3f Idot;                                                               // Time-derivative of inertia
		Eigen::Vector3f com = {0.5, 0.0, 0.0};                                              // Location of center of mass in local frame
		float mass = 1.0;                                                                   // Mass of the rigid body
		
};                                                                                                  // Semicolon needed after a class declaration

#endif