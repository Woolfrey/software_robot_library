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

using namespace Eigen;                                                                              // Eigen::Vector
using namespace std;                                                                                // std::invalid_argument, std::logic_error

template <class DataType>
class TrajectoryBase
{
	public:

		/**
		 * Empty constructor.
		 */
		TrajectoryBase() {}                                                                 // Empty constructor
		
		/**
		 * Full constructor.
		 * @param startTime The time that the trajectory begins.
		 * @param endTime The time that the trajectory finishes.
		 * @param dimensions The spatial dimensions that the trajectory moves through.
		 */
		TrajectoryBase(const DataType     &startTime,
		               const DataType     &endTime,
		               const unsigned int &dimensions);
		           
		/**
		 * Query the current trajectory state for the given input time.
		 * This is a virtual function and must be defined in any derived class.
		 * @param pos The current position.
		 * @param vel The current velocity.
		 * @param acc The current acceleration.
		 * @param time The time at which to query the state.
		 * @return Returns true if there were no problems.
		 */
		virtual bool get_state(Vector<DataType,Dynamic> &pos,
		                       Vector<DataType,Dynamic> &vel,
		                       Vector<DataType,Dynamic> &acc,
		                       const float              &time) = 0;
		
		/**
		 * @return Returns the starting time for this trajectory.
		 */
                DataType start_time() const { return this->_startTime; }
                
                /**
                 * @return Returns the end time for this trajectory.
                 */
                DataType end_time()   const { return this->_endTime;   }
		              
	protected:
	
		DataType _startTime;                                                                ///< The start time for the trajectory
		
		DataType _endTime;                                                                  ///< The end time for the trajectory
		
		unsigned int _dimensions;                                                           ///< The number of spatial dimensions the trajectory moves through
		
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrajectoryBase<DataType>::TrajectoryBase(const DataType     &startTime,
                                         const DataType     &endTime,
                                         const unsigned int &dimensions)
                                         :
                                          _startTime(startTime),
                                          _endTime(endTime),
                                          _dimensions(dimensions)
{
	if(startTime == endTime)
	{
		throw invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
		                       "Start time is equal to the end time for the trajectory "
		                       "(" + to_string(startTime) + " = " + to_string(endTime) + "). "
		                       "You cannot move faster than light.");
	}
	else if(startTime > endTime)
	{
		throw logic_error("[ERROR] [TRAJECTORY BASE] Constructor: "
                                  "Start time is greater than end time for the trajectory "
                                  "(" + to_string(startTime) + " > " + to_string(endTime) + "). "
                                  "You cannot go back in time.");
      	}
      	else if(dimensions == 0)
      	{
      		throw invalid_argument("[ERROR] [TRAJECTORY BASE] Constructor: "
      		                       "Number of dimensions was zero.");
	}
}

#endif
