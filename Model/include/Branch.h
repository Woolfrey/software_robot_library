    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef BRANCH_H_
#define BRANCH_H_

<<<<<<< HEAD
#include <memory>

#include "Joint.h"
#include "RigidBody.h"
=======
#include <Joint.h>
#include <RigidBody.h>
#include <vector>                                                                                   // std::vector
>>>>>>> 8721f15d767f1973bffe3f7d19b95628b3bdf62a

class Branch : public RigidBody
{
	public:
		// Constructor(s)
		Branch() {}                                                                         // Empty constructor

		Branch(const RigidBody &rigidBody,
		       const Joint &joint);

		// Functions
		bool setRoot(const unsigned int &number);                                          // Assign reference to parent branch
        bool setRoot(std::shared_ptr<Branch> root);                                          // Assign reference to parent branch

		bool setStem(const std::vector<unsigned int> &number);                             // Assign reference(s) to child branch(es)
        bool setStem(Branch &stem);                             // Assign reference(s) to child branch(es)

		std::string name() const {return this->_name;}                                      // Get the name

        Joint getJoint() const {return this->_joint;}


        std::vector<std::shared_ptr<Branch>> get_stem() {return this->_stem;}                                                     // Reference to child Branch objects
	private:
        // Properties
//		std::string _name = "branch";                                                       // Unique identifier

        Joint _joint;                                                                        // Obvious

//        unsigned int root = 0;                                                              // Reference to a parent Branch object
//        std::unique_ptr<Branch> _root;                                                              // Reference to a parent Branch object
        std::shared_ptr<Branch> _root = nullptr;                                                              // Reference to a parent Branch object

//        std::vector<unsigned int> stem;                                                     // Reference to child Branch objects
//        std::vector<std::unique_ptr<Branch>> stem;                                                     // Reference to child Branch objects
        std::vector<std::shared_ptr<Branch>> _stem = {};                                                     // Reference to child Branch objects

};                                                                                                  // Semicolon needed at the end of class declaration

#endif
