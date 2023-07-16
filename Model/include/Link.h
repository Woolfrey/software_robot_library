    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LINK_H_
#define LINK_H_

#include <RigidBody.h>                                                                              // Custom class

class Joint;                                                                                        // Forward declaration
 
class Link : public RigidBody
{
	public:
		Link(const std::string &name,
		     const RigidBody   &rigidBody)
		:
		_name(name),
		RigidBody(rigidBody) {}
		
		// Methods
		
		bool merge(Link *otherLink);                                                        // Merge the properties of the other link with this one

		bool add_child_joint(Joint *joint);
		
		bool set_parent_joint(Joint *joint);
		
		Eigen::Vector3f angular_velocity() const { return this->angularVelocity; }
		
		Joint* parent_joint() const { return this->parentJoint; }
		
		std::vector<Joint*> child_joints() const { return this->childJoint; }
		
		std::string name() const { return this->_name; }                                    // Return the name of this link
		
		void set_angular_velocity(const Eigen::Vector3f &velocity) { this->angularVelocity = velocity; }
		
	private:

		Eigen::Vector3f angularVelocity = {0, 0, 0};
		
		Joint* parentJoint = nullptr;
		
		std::vector<Joint*> childJoint = {};	
		
		std::string _name = "unnamed";
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
