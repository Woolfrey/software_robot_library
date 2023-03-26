    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Link.h>                                                                                   // Custom class
#include <map>                                                                                      // std::map
#include <tinyxml2.h>                                                                               // For parsing urdf files

class KinematicTree
{
	public:
		KinematicTree(const std::string &pathToURDF);                                       // Constructor from URDF
		
	private:
		RigidBody base;                                                                     // Base "link"
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
};                                                                                                  // Semicolon needed after class declarations

#endif
