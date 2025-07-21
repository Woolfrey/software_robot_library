/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   This files contains data structures specific to Trajectory classes.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef TRAJECTORY_DATA_STRUCTS_H
#define TRAJECTORY_DATA_STRUCTS_H

#include <Model/Pose.h>

#include <Eigen/Core>                                                                               // Eigen::VectorXd
 
namespace RobotLibrary { namespace Trajectory {
 
/**
 * @brief A data structure for returning state information from functions.
 */
struct CartesianState
{
     RobotLibrary::Model::Pose pose;                                                                ///< Does this really need an explanation?
     Eigen::Vector<double,6>   twist;                                                               ///< Linear and angular velocity
     Eigen::Vector<double,6>   acceleration;                                                        ///<  Linear and angular acceleration
};                                                                                                  // Semicolon needed after declaration


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
