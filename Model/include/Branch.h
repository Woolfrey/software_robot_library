    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRANCH_H_
#define BRANCH_H_

#include <Joint.h>
#include <RigidBody.h>
#include <vector>                                                                                   // std::vector

class Branch : public RigidBody
{
	public:	
		// Constructor(s)
		Branch() {}                                                                         // Empty constructor
		
		Branch(const RigidBody &rigidBody,
		       const Joint &_joint);
		       
		// Functions
		bool set_root(const unsigned int &number);                                          // Assign reference to parent branch
		
		bool set_stem(const std::vector<unsigned int> &number);                             // Assign reference(s) to child branch(es)
	
		std::string name() const {return this->_name;}                                      // Get the name
		
		// Properties
		Joint joint;                                                                        // Obvious
		
		unsigned int root = 0;                                                              // Reference to a parent Branch object
		
		std::vector<unsigned int> stem;                                                     // Reference to child Branch objects
	
	private:
		std::string _name = "branch";                                                       // Unique identifier
		
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
