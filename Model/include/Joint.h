    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <iostream>
#include <vector>

#include "Pose.h"

class Joint
{
	public:
		Joint() {}                                                                         // Empty constructor
		
		Joint(const Pose &origin,
		      const Eigen::Vector3f &axisOfActuation,
		      const float positionLimits[2],
		      const float &velocityLimit,
		      const float &torqueLimit,
		      const bool  &revolute,
              const std::string &parentLink,
              const std::string &childLink,
              const std::string &name = "");
		
		// Get Functions
		bool is_revolute() const {return this->isRevolute;}                                // Returns true if revolute
		
		bool is_prismatic() const {return not this->isRevolute;}                           // Returns true if NOT revolute
		
		Pose pose(const float &jointPosition);                                             // Get the pose associated with the given joint position
		
		Eigen::Vector3f axis() const {return this->_axis;}
		
		float velocity_limit() const {return this->vLim;}                                 // Get the speed limit
		
		void get_position_limits(float &lower, float &upper);                             // Get the position limits

        std::string get_name() const{return this->_name;}
        std::string get_child_link_name() const { return this->_childLink;}
        std::string get_parent_link_name() const { return this->_parentLink;}

	protected:

        std::string _name;
        std::string _parentLink;
        std::string _childLink;

		bool isRevolute = true; // NOTE: NEED TO MAKE A SET {REVOLUTE, PRISMATIC, FIXED}
	
		Pose _pose;
		Eigen::Vector3f _axis = {0,0,1};
		float damping = 1.0;                                                               // Viscous friction for the joint
		float pLim[2] = {-M_PI, M_PI};                                                     // Position limits
		float vLim = 10;                                                                   // Speed limits
		float tLim = 10;                                                                   // Torque / force limits

};                                                                                                 // Semicolon needed after a class declaration

#endif
