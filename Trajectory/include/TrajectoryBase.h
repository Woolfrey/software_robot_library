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
		TrajectoryBase(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
		               const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
		               const DataType &startTime,
		               const DataType &endTime,
		               const unsigned int &dimensions);
		    
		/**
		 * Query the position of the trajectory for the given time.
		 * @param time The point at which to evaluate the position.
		 * @return The position as an Eigen::Vector object
		 */
		Eigen::Vector<DataType,Eigen::Dynamic> query_position(const DataType &time)
		{
			return query_state(time).position;                                          // Too easy lol (☞⌐▀͡ ͜ʖ͡▀ )☞
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
		virtual State<DataType> query_state(const DataType &time) = 0;
		
		/**
		 * @return The computed end time of this trajectory.
		 */
		DataType end_time() const { return this->_endTime; }
		              
	protected:
	
		DataType _startTime;                                                                ///< The start time for the trajectory
		
		DataType _endTime;                                                                  ///< The end time for the trajectory
		
		unsigned int _dimensions;                                                           ///< The number of spatial dimensions the trajectory moves through

		Eigen::Vector<DataType,Eigen::Dynamic> _startPoint;
		
		Eigen::Vector<DataType,Eigen::Dynamic> _endPoint;
	
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrajectoryBase<DataType>::TrajectoryBase(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
                                         const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
                                         const DataType &startTime,
                                         const DataType &endTime,
                                         const unsigned int &dimensions)
                                         :
                                         _startPoint(startPoint),
                                         _endPoint(endPoint),
                                         _startTime(startTime),
                                         _endTime(endTime),
                                         _dimensions(dimensions)
{
	if(startTime == endTime)
	{
		throw std::invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
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
      	else if(dimensions == 0)
      	{
      		throw std::invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
      		                            "Number of dimensions was zero.");
	}
}

#endif
