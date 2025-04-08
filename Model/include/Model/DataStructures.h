/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   Structs used in the Model classes.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef MODEL_DATA_STRUCTS_H
#define MODEL_DATA_STRUCTS_H

#include <Model/Pose.h>

namespace RobotLibrary { namespace Model {

class Link;

/**
 * @brief A data structure for joint limits.
 *        It could represent position, velocity, acceleration, or torque depending on the context.
 */
struct Limits
{
     double lower;                                                                                  ///< Upper limit
     double upper;                                                                                  ///< Lower limit
};                                                                                                  // Semicolon needed after declaring a data structure

/**
 * @brief A structure containing necessary information for defining a reference frame on a kinematic tree.
 */
struct ReferenceFrame
{
     Link *link = nullptr;                                                                          ///< The link it is attached to
     Pose relativePose;                                                                             ///< Pose with respect to local link frame
};

} } // namespace

#endif
