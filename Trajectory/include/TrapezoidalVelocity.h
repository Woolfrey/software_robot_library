/**
 * @file   TrapezoidalVelocity.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A trajectory with constant velocity.
 */

#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Vector, Eigen::Matrix, Eigen::Quaternion
#include <TrajectoryBase.h>

using namespace Eigen;                                                                              // Eigen::Vector

template <class DataType>
class TrapezoidalVelocity : public TrajectoryBase<DataType>
{
	public:

		/**
		 * Emtpy constructor.
		 */
		TrapezoidalVelocity() {}
		
		/**
		 * Full constructor.
		 * @param startPoint A vector of positions for the beginning of the trajectory.
		 * @param endPoint A vector of positions for the end of the trajectory.
		 * @param maxVel A scalar for the maximum speed.
		 * @param maxAcc A scalar for the maximum acceleration and deceleration.
		 * @param startTime The time that the trajectory begins.
		 */
		TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                    const Vector<DataType,Dynamic> &endPoint,
                                    const DataType                 &maxVel,
                                    const DataType                 &maxAcc,
                                    const DataType                 &startTime;
		
		/**
		 * Query the state for the given time.
		 * @param pos A storage location for the position.
		 * @param vel A storage location for the velocity.
		 * @param acc A storage location for the acceleration.
		 * @param time The time at which to compute the state.
		 * @return Returns false if there are any issues.
		 */
		bool get_state(Vector<DataType,Dynamic> &pos,
		               Vector<DataType,Dynamic> &vel,
		               Vector<DataType,Dynamic> &acc,
		               const DataType           &time);			
				
	private:
		DataType _dt;                                                                       ///< Ratio of speed over acceleration
		DataType _t1;                                                                       ///< Time at first corner of trapezoid
		DataType _t2;                                                                       ///< Time at second corner of trapezoid
		DataType _normalizedVel;
		DataType _normalizedAcc;
		
		Vector<DataType,Dynamic> _p1;                                                       ///< Start point
		Vector<DataType,Dynamic> _p2;                                                       ///< End point
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                                   const Vector<DataType,Dynamic> &endPoint,
                                                   const DataType                 &maxVel,
                                                   const DataType                 &maxAcc,
                                                   const DataType                 &startTime)
                                                   :
                                                   _maxVel(maxVel),
                                                   _maxAccel(maxAccel),
                                                   _startTime(startTime);              
{
	if(startPoint.size() != endPoint.size())
	{
		throw invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Dimensions of input arguments do not match. "
		                       "The start point had " + to_string(startPoint.size()) + " elements, "
		                       "and the end point had " + to_string(endPoint.size()) + " elements.");
	}
	else if(maxVel <= 0 or maxAccel <= 0)
	{
		throw invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Velocity and acceleration arguments were "
		                       + to_string(maxVel) + " and " + to_string(maxAcc) +
		                       " but must be positive.");
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the state for the given time                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool TrapezoidalVelocity<DataType>::get_state(Vector<DataType,Dynamic> &pos,
                                              Vector<DataType,Dynamic> &vel,
                                              Vector<DataType,Dynamic> &acc,
                                              const DataType           &time)
{
	// Make sure the inputs are sound
	if(pos.size() != this->dim)
	{
		cerr << "[ERROR] [TRAPEZOIDAL VELOCITY] get_state(): "
		     << "This trajectory object has " << this->dim << " dimensions, "
		     << "but your input argument had " << pos.size() << " elements.\n";
		     
		return false;
	}
	else if(pose.size() != vel.size() or vel.size() != acc.size())
	{
		cerr << "[ERROR] [TRAPEZOIDAL VELOCITY] get_state(): "
		     << "Dimensions of arguments do not match. "
		     << "Position argument had " << pos.size() << " elements, "
		     << "velocity argument had " << vel.size() << " elements, and "
		     << "acceleration argument had + " << acc.size() << " elements.\n";
		     
		return false;
	}
	
	return true;
}


#endif
