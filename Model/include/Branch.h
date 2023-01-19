    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRANCH_H_
#define BRANCH_H_

#include <Joint.h>
#include <RigidBody.h>

class Branch : public RigidBody
{
	public:	       
		const unsigned int &root;
	
		Joint joint;                                                                        // Obvious
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
