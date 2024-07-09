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
 * A data structure for the state of an object.
 */
template <typename DataType>
struct State
{
     Eigen::Vector<DataType,Eigen::Dynamic> position;
     Eigen::Vector<DataType,Eigen::Dynamic> velocity;
     Eigen::Vector<DataType,Eigen::Dynamic> acceleration;
};                                                                                                  // Semicolon needed after struct declaration

template <class DataType>
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
        TrajectoryBase(const State<DataType> &startPoint,
                       const State<DataType> &endPoint,
                       const DataType        &startTime,
                       const DataType        &endTime);

        /**
         * Query the position of the trajectory for the given time.
         * @param time The point at which to evaluate the position.
         * @return The position as an Eigen::Vector object
         */
        inline
        Eigen::Vector<DataType,Eigen::Dynamic>
        query_position(const DataType &time)
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
        State<DataType>
        query_state(const DataType &time) = 0;

        /**
         * @return The time at which this trajectory commences.
         */
        inline
        DataType
        start_time() const { return this->_startTime; }
        
        /**
         * @return The time at which this trajectory finishes.
         */
        inline
        DataType
        end_time() const { return this->_endTime; }

    protected:

    DataType _startTime;                                                                            ///< The start time for the trajectory

    DataType _endTime;                                                                              ///< The end time for the trajectory

    State<DataType> _startPoint;                                                                    ///< The starting position, velocity, and acceleration

    State<DataType> _endPoint;                                                                      ///< The final position, velocity, and acceleration
    
    unsigned int _dimensions;                                                                       ///< The number of dimensions this trajectory spans
    
};                                                                                                  // Semicolon needed after class declaration

using TrajectoryBase_f = TrajectoryBase<float>;
using TrajectoryBase_d = TrajectoryBase<double>;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrajectoryBase<DataType>::TrajectoryBase(const State<DataType> &startPoint,
                                         const State<DataType> &endPoint,
                                         const DataType        &startTime,
                                         const DataType        &endTime)
                                         :
                                         _startPoint(startPoint),
                                         _endPoint(endPoint),
                                         _startTime(startTime),
                                         _endTime(endTime),
                                         _dimensions(startPoint.position.size())
{
    if(startTime == endTime)
    {
        throw std::logic_error("[ERROR] [TRAJECTORY BASE] Constructor: "
                               "Start time is equal to the end time for the trajectory "
                               "(" + std::to_string(startTime) + " = " + std::to_string(endTime) + "). "
                               "You cannot move faster than light.");
    }
    else if(startTime > endTime)
    {
        throw std::logic_error("[ERROR] [TRAJECTORY BASE] Constructor: "
                               "Start time is greater than end time for the trajectory "
                               "(" + std::to_string(startTime) + " > " + std::to_string(endTime) + "). "
                               "You cannot go back in time.");
    }
    else if(startPoint.position.size()     != startPoint.velocity.size()
         or startPoint.acceleration.size() != endPoint.position.size()
         or endPoint.position.size()       != endPoint.velocity.size()
         or endPoint.velocity.size()       != endPoint.acceleration.size())
    {
        throw std::invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
                                    "Dimensions of arguments do not match. "
                                    "The start position had " + std::to_string(startPoint.position.size()) + " elements, "
                                    "the start velocity had " + std::to_string(startPoint.velocity.size()) + " elements, "
                                    "the start acceleration had " + std::to_string(startPoint.acceleration.size()) + " elements, "
                                    "the end position had " + std::to_string(endPoint.position.size()) + " elements, "
                                    "the end velocity had " + std::to_string(endPoint.velocity.size()) + " elements, and "
                                    "the end acceleration had " + std::to_string(endPoint.acceleration.size()) + " elements.");
    }
}

#endif
