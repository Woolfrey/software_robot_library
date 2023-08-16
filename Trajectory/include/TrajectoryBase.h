    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                               A base class for trajectories                                   //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

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
		TrajectoryBase() {}                                                                 // Empty constructor
		
		TrajectoryBase(const DataType     &startTime,
		               const DataType     &endTime,
		               const unsigned int &dimensions);
			
		// Get the desired state for the given time
		virtual bool get_state(Vector<DataType,Dynamic> &pos,
		                       Vector<DataType,Dynamic> &vel,
		                       Vector<DataType,Dynamic> &acc,
		                       const float              &time) = 0;
                                       
                // Other functions
                DataType start_time() const { return this->_startTime; }
                DataType end_time()   const { return this->_endTime;   }
		              
	protected:
		
		DataType _startTime, _endTime;
		
		unsigned int _dimensions;
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrajectoryBase<DataType>::TrajectoryBase(const float        &startTime,
                                         const float        &endTime,
                                         const unsigned int &dim)
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
