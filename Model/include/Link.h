    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LINK_H_
#define LINK_H_

#include <Joint.h>                                                                                  // Custom class
#include <memory>                                                                                   // std::shared_ptr
#include <RigidBody.h>                                                                              // Custom class

class Link : public RigidBody
{
	public:
		Link(const Joint     &_joint,
		     const RigidBody &rigidBody);
		     
		Joint joint;                                                                        // The joint attached to this link
		
		bool set_preceding_link(Link &link);
		
		bool add_proceeding_link(Link &link);
	
	private:
		std::shared_ptr<Link> _precedingLink = nullptr;
		
		std::vector<std::shared_ptr<Link>> _proceedingLink = {};

};                                                                                                  // Semicolon needed at the end of class declaration

#endif
