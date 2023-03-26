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
		
		const Link* preceding_link() const { return this->precedingLink; }                  // Get the preceding link in the chain
		
		const Link* proceeding_link(const unsigned int &i) const { return this->proceedingLink[i]; } // Get the proceeding link in the chain

		bool add_proceeding_link(const Link* link);
		
		void set_preceding_link(const Link* link) { this->precedingLink = link; }
		
	private:
	
		const Link* precedingLink = nullptr;
		
		std::vector<const Link*> proceedingLink;

};                                                                                                  // Semicolon needed at the end of class declaration

#endif
