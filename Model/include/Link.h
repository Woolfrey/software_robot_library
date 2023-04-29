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
		
		bool merge(Link *other);                                                            // Merge the properties of the other link with this one
		
		bool attach_joint(Joint *joint);                                                    // Specify a pointer to the preceding joint
		
		bool set_previous_link(Link *link);
		
		bool add_next_link(Link *link);
		
		Joint* attached_joint() const { return this->attachedJoint; }                       // Get the pointer to the joint
		
		Link* previous_link() const { return this->previousLink; }
		
		std::vector<Link*> next_links() const { return this->nextLinks; }
		
		std::string name() const { return this->_name; }                                    // Return the name of this link
		     
	private:
		
		Joint *attachedJoint = nullptr;                                                     // Pointer to the attached joint
		
		Link *previousLink = nullptr;                                                       // Pointer to previous link in the kinematic chain
		
		std::vector<Link*> nextLinks {};                                                    // Pointer to next links in the kinematic chain
		
		std::string _name;
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
