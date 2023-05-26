    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <Pose.h>
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
		      const Pose            &origin,
		      const float            positionLimit[2],
		      const float           &velocityLimit,
		      const float           &forceLimit,
		      const float           &damping,
		      const float           &friction);
		
		// Functions
	
		bool set_child_link(Link* link);                                                    // Set pointer to child link
		
		bool set_parent_link(Link* link);                                                   // Set pointer to parent link
	
		Link* parent_link() const { return this->parentLink; }                              // Get the pointer to parent link
		
		Link* child_link() const { return this->childLink; }                                // Get pointer to child link
		
		Pose origin() const { return this->_origin; }                                       // Joint pose relative to its parent link
		
		std::string type() const { return this->_type; }
		
		std::string name() const { return this->_name; }
		
		void offset_origin(Pose &offset) { this->_origin = offset*this->_origin; }
		
	private:
	
		Eigen::Vector3f _axis;                                                              // Axis of actuation for this joint

		float _position = 0;
		
		float _velocity = 0;

		float _positionLimit[2];
		
		float _velocityLimit;
		
		float _forceLimit;
		
		float _damping;
		
		float _friction;
		
		Pose _origin;                                                                       // Origin relative to preceding link
		
		std::string _type;
		
		std::string _name;
		
		Link* childLink = nullptr;
		
		Link* parentLink = nullptr;
};                                                                                                  // Semicolon needed after a class declaration

#endif
