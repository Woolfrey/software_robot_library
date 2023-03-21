    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef LINK_H_
#define LINK_H_

#include <Joint.h>
#include <RigidBody.h>

class Link : public RigidBody
{
	public:
		Link(const Joint     &_joint,
		     const RigidBody &rigidBody);
		     
		Joint joint;
		
	private:
		
		
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
