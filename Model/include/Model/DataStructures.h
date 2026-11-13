/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Structs used in the Model classes.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef MODEL_DATA_STRUCTS_H
#define MODEL_DATA_STRUCTS_H

namespace RobotLibrary { namespace Model {

/**
 * @brief A data structure for joint limits.
 *        It could represent position, velocity, acceleration, or torque depending on the context.
 */
struct Limits
{
     double lower;                                                                                  ///< Upper limit
     double upper;                                                                                  ///< Lower limit
};                                                                                                  // Semicolon needed after declaring a data structure

} } // namespace

#endif
