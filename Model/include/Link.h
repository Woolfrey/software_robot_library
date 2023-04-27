    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LINK_H_
#define LINK_H_

#include <memory>                                                                                   // std::shared_ptr
#include <RigidBody.h>                                                                              // Custom class

class Joint;                                                                                        // Forward declaration
 
class Link : public RigidBody
{
	public:
		Link(const std::string &name,
		     const RigidBody &rigidBody)
		:
		_name(name),
		RigidBody(rigidBody) {}
		
		bool merge(const Link *other);                                                      // Merge the properties of the other link with this one
		
		bool attach_joint(Joint *joint);                                                    // Specify a pointer to the preceding joint
		
		Joint* attached_joint() const { return this->attachedJoint; }                       // Get the pointer to the joint
		
		std::string name() const { return this->_name; }                                    // Return the name of this link
		     
	private:
		
		Joint *attachedJoint = nullptr;                                                     // Pointer to the attached joint
		
		std::string _name;
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
