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
		TrapezoidalVelocity(const Eigen::Vector<DataType, Eigen::Dynamic> &startPoint,
                                    const Eigen::Vector<DataType, Eigen::Dynamic> &endPoint,
                                    const DataType                                &maxVel,
                                    const DataType                                &maxAcc,
                                    const DataType                                &startTime);
		
		/**
		 * Query the state for the given time. Override from TrajectoryBase class.
		 * @param pos A storage location for the position.
		 * @param vel A storage location for the velocity.
		 * @param acc A storage location for the acceleration.
		 * @param time The time at which to compute the state.
		 * @return Returns false if there are any issues.
		 */
		State<DataType> query_state(const DataType &time);	
				
	private:
	
		DataType _rampTime;
		DataType _firstCornerDist;
		DataType _secondCornerDist;
	
	
		DataType _dt;                                                                       ///< Ratio of speed over acceleration
		DataType _s1;                                                                       ///< Normalised distance to first corner on trapezoid
		DataType _s2;                                                                       ///< Normalised distance to second corner on trapezoid
		DataType _t1;                                                                       ///< Time at first corner of trapezoid
		DataType _t2;                                                                       ///< Time at second corner of trapezoid
		DataType _normalisedVel;
		DataType _normalisedAcc;
		
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
                                                   _startPoint(startPoint),
                                                   _endPoint(endPoint)
                                                   _dimensions(startPoint.size()),
                                                   _startTime(startTime),
{
	if(startPoint.size() != endPoint.size())
	{
		throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                            "Dimensions of input arguments do not match. "
		                            "The start point had " + std::to_string(startPoint.size()) + " elements, "
		                            "and the end point had " + std::to_string(endPoint.size()) + " elements.");
	}
	else if(maxVel <= 0 or maxAccel <= 0)
	{
		throw std::invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                            "Velocity and acceleration arguments were "
		                            + std::to_string(maxVel) + " and " + std::to_string(maxAcc) +
		                            " but must be positive.");
	}
	
	DataType normaliser -1;                                                                     // Needed to scale velocity to be dimensionless
	
	DataType vel = maxVel;                                                                      // Temporary placeholder
	
	for(int i = 0; i < this->_dimensions; i++)
	{
		DataType distance = abs(startPoint(i) - endPoint(i));                               // Magnitude of distance between points
		
		normaliser = (distance > normaliser) ? distance : normaliser;                       // Normaliser must be the largest distance
		
		DataType temp = sqrt(maxAccel*distance);                                            // Peak velocity as a function of max acceleration
		
		vel = (vel > temp) ? temp : vel;                                                    // Ensure max velocity does not exceed acceleration, distance constraints
	}
	
	this->_normalisedVelocity = vel/normaliser;                                                 // Need to normalise so we can scale between 0 and 1
	
	this->_normalisedAcceleration = maxAcc/normaliser;                                          // Scale the acceleration to match the velocity
	
	this->_dt = vel/maxAcc;                                                                     // Time between corners of trapezoid
	
	this->_t1 = this->_startTime + dt;                                                          // Time to reach 1st corner of trapezoid
	
	this->_s1 = (this->_dt*this->_dt*this->_normalisedAcc)/2.0;                                 // Normalised distance to first corner of trapezoid
	
	this->_t2 = this->_t1 + (1 - 2*this->_s1 + 0)/this->_normalisedVel;                         // Time to reach 2nd corner
	
	this->_s2 = this->_s1 + (this->_t2 - this->_t1)/this->_normalisedVel;                       // Normalised distance to second corner of trapezoid
	
	this->_endTime = this->_t2 + this->_dt;                                                     // End time of the trajectory
	
	if(this->_s1 >= 1.0 or this->_s2 >= 1.0)
	{
		throw std::logic_error("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Normalised distance must be between 0 and 1, but "
		                       "the distance to the first corner was " + std::to_string(this->_s1) + ", and "
		                       "the distance to the second corner was " + std::to_string(this->_s2) + ".");
	}
	else if(s1 > s2)
	{
		throw std::logic_error("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Normalised distance to first corner of waypoint was greater than the distance "
		                       "to the second corner (" + std::to_string(this->_s1) + " > "
		                       + std::to_string(this->_s2) + ").");
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the state for the given time                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
State<DataType> TrapezoidalVelocity<DataType>::get_state(const DataType &time)
{
	DataType s, sd, sdd, t;                                                                     // Scalars for interpolation
	
	if(time < this->_startTime)
	{
		s   = 0.0;
		sd  = 0.0;
		sdd = 0.0;
	}
	else if(time < this->_t1)
	{
		t = time - this->_startTime;
		
		s   = (t*t*this->_normalisedAcc)/2.0;
		sd  =    t*this->_normalisedAcc;
		sdd =      this->_normalisedAcc;
	}
	else if(time < this->_t2)
	{
		t = time - this->_t1;
		
		s   = this->_s1 + t*this->_normalisedVel;
		sd  = this->_normalisedVel;
		sdd = 0;
	}
	else if(time < this->_endTime)
	{
		t = time - this->_t2;
		
		s   =  this->s_2 + t*this->_normalisedVel - (t*t*this->_normalisedAcc)/2.0;
		sd  =  this->_normalisedVel - t*this->_normalisedAcc;
		sdd = -this->_normalisedAcc;
	}
	else
	{
		s   = 1.0;
		sd  = 0.0;
		sdd = 0.0;
	}
	
	State<DataType> state = { (1-s)*this->_startPoint - s*this->_endPoint,                      // Position
	                            sd*(this->_endPoint   -   this->_startPoint),                   // Velocity
	                           sdd*(this->_endPoint   -   this->_startPoint) };                 // Acceleration
	
	return state;
}

#endif
