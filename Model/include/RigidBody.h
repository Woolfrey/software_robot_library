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
		// Empty constructor, delegate defaults to full constructor
		RigidBody()
		:
		RigidBody(0.1,
		         (Eigen::MatrixXf(3,3) << 1e-04,   0.0,   0.0,
		                                    0.0, 1e-04,   0.0,
		                                    0.0,   0.0, 1e-04).finished(),
		          Pose()) {}
		           
		// Full constructor                                       
		RigidBody(const float &mass,
		          const Eigen::Matrix3f &momentOfInertia,
		          const Pose &centreOfMass);
		          
	private:
		float _mass;
		
		Eigen::Matrix3f _momentOfInertia;
		
		Pose _centreOfMass;
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
