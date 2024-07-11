/**
 * @file   TrajectoryBase.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A base class for trajectory objects.
 */

#ifndef TRAJECTORYBASE_H_
#define TRAJECTORYBASE_H_

#include <Eigen/Core>                                                                               // Eigen::Vector
#include <iostream>                                                                                 // std::cout

/**
 * A data structure for the state of a system.
 */
struct State
{
     Eigen::VectorXd position;
     Eigen::VectorXd velocity;
     Eigen::VectorXd acceleration;
};                                                                                                  // Semicolon needed after struct declaration

/**
 * A base class for providing common structure to all other trajectory classes.
 */
class TrajectoryBase
{
    public:
    
        /**
         * Empty constructor.
         */
        TrajectoryBase() {}

        /**
         * Full constructor.
         * @param startTime The time that the trajectory begins.
         * @param endTime The time that the trajectory finishes.
         * @param dimensions The spatial dimensions that the trajectory moves through.
         */
        TrajectoryBase(const State  &startPoint,
                       const State  &endPoint,
                       const double &startTime,
                       const double &endTime);

        /**
         * Query the position of the trajectory for the given time.
         * @param time The point at which to evaluate the position.
         * @return The position as an Eigen::Vector object
         */
        inline
        Eigen::VectorXd
        query_position(const double &time)
        {
            return query_state(time).position;                                                      // Too easy lol (☞⌐▀͡ ͜ʖ͡▀ )☞
        }

        /**
         * Query the current trajectory state for the given input time.
         * This is a virtual function and must be defined in any derived class.
         * @param pos The current position.
         * @param vel The current velocity.
         * @param acc The current acceleration.
         * @param time The time at which to query the state.
         * @return Returns true if there were no problems.
         */
        virtual
        inline
        State
        query_state(const double &time) = 0;

        /**
         * @return The time at which this trajectory commences.
         */
        inline
        double
        start_time() const { return this->_startTime; }
        
        /**
         * @return The time at which this trajectory finishes.
         */
        inline
        double
        end_time() const { return this->_endTime; }

    protected:

    double _startTime;                                                                              ///< The start time for the trajectory

    double _endTime;                                                                                ///< The end time for the trajectory

    State _startPoint;                                                                              ///< The starting position, velocity, and acceleration

    State _endPoint;                                                                                ///< The final position, velocity, and acceleration
    
    unsigned int _dimensions;                                                                       ///< The number of dimensions this trajectory spans
    
};                                                                                                  // Semicolon needed after class declaration

#endif
