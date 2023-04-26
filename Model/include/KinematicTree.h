    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>
#include <Link.h>
#include <map>                                                                                      // std::map
#include <tinyxml2.h>                                                                               // For parsing urdf files

class KinematicTree
{
	public:
		KinematicTree(const std::string &pathToURDF);                                       // Constructor from URDF

	private:

		std::map<std::string, Joint> jointList;
		
		std::map<std::string, Link> linkList;
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
};                                                                                                  // Semicolon needed after class declarations

#endif
