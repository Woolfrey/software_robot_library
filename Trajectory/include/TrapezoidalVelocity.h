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
		DataType _normalisedVel;                                                            ///< Maximum velocity, normalised so total distance  = 1
		DataType _normalisedAcc;                                                            ///< Maximum acceleration, normalised so total distance = 1
		DataType _rampDistance;                                                             ///< Distance traveled whilst accelerating
		DataType _rampTime;                                                                 ///< Length of time to accelerate
				
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
                                                   const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
                                                   const DataType &maxVel,
                                                   const DataType &maxAccel,
                                                   const DataType &startTime)
{
	// Assign initial values in underlying TrajectoryBase class
	this->_dimensions = startPoint.size();                                                      // Assign to variable in base class
	this->_startTime  = startTime;
	
	// Assign values to this TrapezoidalVelocity object
	this->_startPoint = startPoint;
	this->_endPoint   = endPoint;
	
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
	

	DataType normaliser = -1;                                                                   // Needed to scale velocity to be dimensionless
	
	DataType vel = maxVel;                                                                      // Temporary placeholder
	
	for(int i = 0; i < this->_dimensions; i++)
	{
		DataType distance = abs(startPoint(i) - endPoint(i));                               // Magnitude of distance between points
		
		normaliser = (distance > normaliser) ? distance : normaliser;                       // Normaliser must be the largest distance
		
		DataType temp = sqrt(maxAccel*distance);                                            // Peak velocity as a function of max acceleration
		
		vel = (vel > temp) ? temp : vel;                                                    // Ensure max velocity does not exceed acceleration, distance constraints
	}
	
	this->_normalisedVel = vel/normaliser;                                                      // Need to normalise so we can scale between 0 and 1
	
	this->_normalisedAcc = maxAccel/normaliser;                                                 // Scale the acceleration to match the velocity
	
	this->_rampTime = vel/maxAccel;                                                             // Length of time to accelerate to max speed
	
	this->_rampDistance = 0.5*this->_normalisedAcc*this->_rampTime*this->_rampTime;             // s = 0.5*a*t^2
	
	this->_coastTime = (1.0 - 2*this->_rampDistance)/this->_normalisedVel;                      // Time spent moving at max speed
	
	this->_coastDistance = this->_normalisedVel*this->_coastTime;                               // Distance travelled moving at max speed
	
	this->_endTime = this->_startTime + 2*this->_rampTime + this->_coastTime;                   // Total time passed
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
		state.position     = this->_startPoint;
		state.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
		state.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
	}
	else if(time > this->_endTime)
	{
		state.position     = this->_endPoint;
		state.velocity     = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
		state.acceleration = Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions);
	}
	else
	{
		DataType elapsedTime = time - this->_startTime;
		
		DataType s, sd, sdd;                                                                // Interpolating scalars
		
		if(elapsedTime < this->_rampTime)
		{
			  s = this->_normalisedAcc*elapsedTime*elapsedTime/2.0;
			 sd = this->_normalisedAcc*elapsedTime;
			sdd = this->_normalisedAcc;
		}
		else if(elapsedTime < this->_rampTime + this->_coastTime)
		{
			  s = this->_rampDistance + this->_normalisedVel*(elapsedTime - this->_rampTime);
			 sd = this->_normalisedVel;
			sdd = 0.0;
		}
		else
		{
			DataType t = elapsedTime - this->_coastTime - this->_rampTime;
			
			  s =  this->_rampDistance  + this->_coastDistance + this->_normalisedVel*t - this->_normalisedAcc*t*t/2.0;
			 sd =  this->_normalisedVel - this->_normalisedAcc*t;
			sdd = -this->_normalisedAcc;
		}
		
		// Interpolate the state
		
		state.position     = (1.0 - s)*this->_startPoint + s*this->_endPoint;
		state.velocity     = sd*(this->_endPoint - this->_startPoint);
		state.acceleration = sdd*(this->_endPoint - this->_startPoint);
	}
	
	return state;
}

#endif
