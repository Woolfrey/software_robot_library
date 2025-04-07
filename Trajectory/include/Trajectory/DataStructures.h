/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   This files contains data structures specific to Trajectory classes.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef TRAJECTORY_STRUCTS_H
#define TRAJECTORY_STRUCTS_H
 
namespace RobotLibrary { namespace Trajectory {
 
/**
 * @brief A data structure for the state of a system.
 */
struct State
{
     Eigen::VectorXd position;
     Eigen::VectorXd velocity;
     Eigen::VectorXd acceleration;
};                                                                                                  // Semicolon needed after struct declaration

} } // namespace

#endif
