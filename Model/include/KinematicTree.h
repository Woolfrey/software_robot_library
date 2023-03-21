    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Link.h>                                                                                   // Custom class
#include <tinyxml2.h>

class KinematicTree
{
	public:
		KinematicTree(const std::string &pathToURDF);                                       // Constructor from URDF
		
	private:
		std::vector<Link> link;
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
};                                                                                                  // Semicolon needed after class declarations

#endif
