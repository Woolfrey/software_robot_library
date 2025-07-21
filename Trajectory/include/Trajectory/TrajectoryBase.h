/**
 * @file    TrajectoryBase.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   A class to standardise the structure of all other trajectory classes.
 * 
 * @details This class is used to standardise the structure and interfaces for all trajectory classes.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 *
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef TRAJECTORY_BASE_H
#define TRAJECTORY_BASE_H

#include <Trajectory/DataStructures.h>
#include <Eigen/Core>                                                                               // Eigen::Vector
#include <iostream>                                                                                 // std::cout

namespace RobotLibrary { namespace Trajectory {

/**
 * @brief A base class for providing common structure to all other trajectory classes.
 */
class TrajectoryBase
{
    public:
    
        /**
         * @brief Empty constructor.
         */
        TrajectoryBase() {}

        /**
         * @brief Full constructor.
         * @param startTime The time that the trajectory begins.
         * @param endTime The time that the trajectory finishes.
         * @param dimensions The spatial dimensions that the trajectory moves through.
         */
        TrajectoryBase(const RobotLibrary::Trajectory::State &startPoint,
                       const RobotLibrary::Trajectory::State &endPoint,
                       const double &startTime,
                       const double &endTime);

        /**
         * @brief Query the position of the trajectory for the given time.
         * @param time The point at which to evaluate the position.
         * @return The position as an Eigen::Vector object
         */
        Eigen::VectorXd
        query_position(const double &time)
        {
            return query_state(time).position;                                                      // Too easy lol (☞⌐▀͡ ͜ʖ͡▀ )☞
        }

        /**
         * @brief Query the current trajectory state for the given input time.
         *        This is a virtual function and must be defined in any derived class.
         * @param pos The current position.
         * @param vel The current velocity.
         * @param acc The current acceleration.
         * @param time The time at which to query the state.
         * @return Returns true if there were no problems.
         */
        virtual
        RobotLibrary::Trajectory::State
        query_state(const double &time) = 0;

        /**
         * @brief The time at which this trajectory commences.
         */
        double
        start_time() const { return this->_startTime; }
        
        /**
         * @brief The time at which this trajectory finishes.
         */
        double
        end_time() const { return this->_endTime; }

    protected:

    double _startTime;                                                                              ///< The start time for the trajectory

    double _endTime;                                                                                ///< The end time for the trajectory

    RobotLibrary::Trajectory::State _startPoint;                                                    ///< The starting position, velocity, and acceleration

    RobotLibrary::Trajectory::State _endPoint;                                                      ///< The final position, velocity, and acceleration
    
    unsigned int _dimensions;                                                                       ///< The number of dimensions this trajectory spans
    
};                                                                                                  // Semicolon needed after class declaration

} } // namespace

#endif
