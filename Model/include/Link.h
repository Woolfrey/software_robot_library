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
		
		Eigen::Matrix3f local_inertia() const {return this->_inertia;}
		
		void combine_link(const Link &other);
		
		Link* preceding_link() const { return this->_precedingLink; }                       // Get the preceding link in the chain
		
	private:
	
		Link* _precedingLink;

};                                                                                                  // Semicolon needed at the end of class declaration

#endif
