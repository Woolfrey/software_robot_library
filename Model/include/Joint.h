    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <Frame.h>                                                                                  // Custom class for a coordinate frame
#include <vector>

class Link;                                                                                         // Forward declaration since Joint and RigidBody are mutually referential

class Joint
{
	public:
		// Minimum constructor, delegate to full constructor
		Joint(const std::string     &name,
		      const std::string     &type,
		      const Eigen::Vector3f &axis,
		      const float            positionLimit[2])
		:
		Joint(name, type, axis, Pose(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}

		// Full constructor
		Joint(const std::string     &name,
		      const std::string     &type,
		      const Eigen::Vector3f &axis,
		      const Pose            &offset,
		      const float            positionLimit[2],                                      // Can't use pointers & for arrays for some reason
		      const float           &speedLimit,
		      const float           &effortLimit,
		      const float           &damping,
		      const float           &frictionS);
		
		// Methods
	
		bool is_prismatic() const { return not this->isRevolute; }                          // Because I'm lazy
		
		bool is_revolute() const { return this->isRevolute; }
		
		bool set_child_link(Link &link);                                                    // Set pointer to child link
		
		bool set_parent_link(Link &link);                                                   // Set pointer to parent link
	
		bool update_state(const Pose &previousPose, const float &position);
		
		Eigen::Vector3f axis() const { return this->_axis; }		
		
		Link* parent_link() const { return this->_parentLink; }                             // Get the pointer to parent link
		
		Link* child_link() const { return this->_childLink; }                               // Get pointer to child link
		
		Pose offset() const { return this->_offset; }                                       // Get the pose of the joint relative to pose of joint on parent link
		
		Pose pose() const { return this->_pose; }                                           // Get the pose of the joint in the parent frame
		
		std::string type() const { return this->_type; }
		
		std::string name() const { return this->_name; }                                    // Get the name of this joint
		
		unsigned int number() const { return this->_number; }                               // Get the index for this joint in the joint vector
		
		void extend_offset(Pose &other) { this->_offset = other*this->_offset; }            // Extend the offset for fixed joints when merging links
		
		void set_number(const unsigned int &number) { this->_number = number; }             // Set the index in the joint vector list
		
	private:
	
		bool isRevolute = true;
		
		Eigen::Vector3f _localAxis;                                                         // Axis of actuation in local frame
		
		Eigen::Vector3f _axis;                                                              // Axis of actuation in global frame

		float _positionLimit[2];
		
		float _speedLimit;
		
		float _effortLimit;
		
		float _damping;
		
		float _friction;
		
		Link* _childLink = nullptr;
		
		Link* _parentLink = nullptr;
		
		Pose _offset;                                                                       // Pose with respect to joint of parent link

		Pose _pose;                                                                         // Pose of the joint in global frame
		
		std::string _type = "unknown";
		
		std::string _name = "unnamed";                                                      // Unique identifier
		
		unsigned int _number = 0;                                                           // The number in the joint list
};                                                                                                  // Semicolon needed after a class declaration

#endif
