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
                                    const DataType                                &maxAccel,
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
	
		DataType _coastDistance;                                                            ///< Distance covered at maximum speed
		DataType _coastTime;                                                                ///< Length of time to move at max speed
		DataType _endPoint;                                                                 ///< Final position on the trajectory
		DataType _endTime;                                                                  ///< Final time for the trajectory
		DataType _normalisedVel;                                                            ///< Maximum velocity, normalised so total distance  = 1
		DataType _normalisedAcc;                                                            ///< Maximum acceleration, normalised so total distance = 1
		DataType _rampDistance;                                                             ///< Distance traveled whilst accelerating
		DataType _rampTime;                                                                 ///< Length of time to accelerate
		DataType _startPoint;                                                               ///< Starting position
		DataType _startTime;                                                                ///< Time at which the trajectory begins
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                                   const Vector<DataType,Dynamic> &endPoint,
                                                   const DataType                 &maxVel,
                                                   const DataType                 &maxAccel,
                                                   const DataType                 &startTime)
                                                   :
                                                   _startPoint(startPoint),
                                                   _endPoint(endPoint),
                                                   _startTime(startTime)
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
		                            + std::to_string(maxVel) + " and " + std::to_string(maxAccel) +
		                            " but must be positive.");
	}
	
	this->_dimensions = startPoint.size();                                                      // Assign to variable in base class
	
	DataType normaliser = -1;                                                                   // Needed to scale velocity to be dimensionless
	
	DataType vel = maxVel;                                                                      // Temporary placeholder
	
	for(int i = 0; i < this->_dimensions; i++)
	{
		DataType distance = abs(startPoint(i) - endPoint(i));                               // Magnitude of distance between points
		
		normaliser = (distance > normaliser) ? distance : normaliser;                       // Normaliser must be the largest distance
		
		DataType temp = sqrt(maxAccel*distance);                                            // Peak velocity as a function of max acceleration
		
		vel = (vel > temp) ? temp : vel;                                                    // Ensure max velocity does not exceed acceleration, distance constraints
	}
	
	this->_normalisedVelocity = vel/normaliser;                                                 // Need to normalise so we can scale between 0 and 1
	
	this->_normalisedAcceleration = maxAccel/normaliser;                                        // Scale the acceleration to match the velocity
	
	this->_rampTime = vel/maxAccel;                                                             // Length of time to accelerate to max speed
	
	this->_rampDistance = 0.5*this->_normalisedAcceleration*this->_rampTime*this->_rampTime;    // s = 0.5*a*t^2
	
	this->_coastTime = (1.0 - 2*this->_rampDistance)/this->_normalisedVelocity;                 // Time spent moving at max speed
	
	this->_coastDistance = this->_normalisedVelocity*this->_coastTime;                          // Distance travelled moving at max speed
	
	this->_endTime = 2*this->_rampTime + this->_coastTime;                                      // Total time passed
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the state for the given time                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
State<DataType> TrapezoidalVelocity<DataType>::query_state(const DataType &time)
{
	State<DataType> state;                                                                      // Value to be returned
	
	if(time <= this->_startTime)
	{
		state.position     = this->_startPosition;
		state.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
		state.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
	}
	else if(time > this->_endTime)
	{
		state.position     = this->_endPosition;
		state.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
		state.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
	}
	else
	{
		DataType elapsedTime = time - this->_startTime;
		
		DataType s, sd, sdd;                                                                // Interpolating scalars
		
		if(elapsedTime < this->_rampTime)
		{
			  s = this->_normalisedAcc*this->elapsedTime*this->elapsedTime/2.0;
			 sd = this->_normalisedAcc*this->_elapsedTime;
			sdd = this->_normalisedAcc;
		}
		else if(elapsedTime < this->_rampTime + this->_coastTime)
		{
			  s = this->_rampDistance + this->_normalisedVel*(this->elapsedTime - this->_rampTime);
			 sd = this->_normalisedVel;
			sdd = 0.0;
		}
		else
		{
			DataType t = this->_elapsedTime - this->_coastTime - this->_rampTime;
			
			  s =  this->_rampDistance  + this->_coastDistance - this->_normalisedAcc*t*t/2.0;
			 sd =  this->_normalisedVel - this->_normalisedAcc*this->t;
			sdd = -this->_normalisedAcc;
		}
		
		// Interpolate the state
		state.position     = (1.0 - s)*this->_startPosition + s*this->_endPosition;
		state.velocity     = sd*(this->_endPosition - this->_startPosition);
		state.acceleration = sdd*(this->_endPosition - this->_startPosition);
	}
	
	return state;
}

#endif
