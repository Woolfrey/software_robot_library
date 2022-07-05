    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <Eigen/Geometry>                                                                          // Eigen::Isometry3f, Eigen::Vector3f

class Joint
{
	public:
		Joint() {}                                                                         // Empty constructor
		
		Joint(const Eigen::Isometry3f &origin,
                     const Eigen::Vector3f &axisOfActuation,
                     const float positionLimits[2],
                     const float &velocityLimit,
                     const float &torqueLimit,
                     const bool &revolute);
		
		// Get Functions
		bool is_revolute() const {return this->isRevolute;}                                // Returns true if revolute
		
		bool is_prismatic() const {return not this->isRevolute;}                           // Returns true if NOT revolute
		
		Eigen::Isometry3f get_pose(const float &pos);                                      // Get the pose from displacement of this joint
		
		Eigen::Vector3f get_axis() const {return this->axis;}
		
		float get_velocity_limit() const {return this->vLim;}                              // Get the speed limit
		
		void get_position_limits(float &lower, float &upper);                              // Get the position limits
		
	protected:
		bool isRevolute = true;
		Eigen::Isometry3f pose;                                                            // Pose relative to some origin
		Eigen::Vector3f axis = {0,0,1};                                                    // Axis of actuation
		float damping = 1.0;                                                               // Viscous friction for the joint
		float pLim[2] = {-M_PI, M_PI};                                                     // Position limits
		float vLim = 10;                                                                   // Speed limits
		float tLim = 10;                                                                   // Torque / force limits

};                                                                                                 // Semicolon needed after a class declaration

#endif