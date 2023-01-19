    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <Branch.h>

class KinematicTree
{
	public:
		// Constructor(s)
		KinematicTree() {}                                                                  // Empty constructor
		
		KinematicTree(const std::Vector<Branch> &branches);
		
	private:
};                                                                                                  // Semicolon needed after class declarations

#endif
