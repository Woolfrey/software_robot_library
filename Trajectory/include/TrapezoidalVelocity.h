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
		DataType _s1;
		DataType _s2;
		DataType _t1;                                                                       ///< Time at first corner of trapezoid
		DataType _t2;                                                                       ///< Time at second corner of trapezoid
		DataType _normalisedVel;
		DataType _normalisedAcc;
		
		
		Vector<DataType,Dynamic> _startPoint;                                               ///< Start point
		Vector<DataType,Dynamic> _endPoint;                                                 ///< End point
		
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
		throw invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Dimensions of input arguments do not match. "
		                       "The start point had " + to_string(startPoint.size()) + " elements, "
		                       "and the end point had " + to_string(endPoint.size()) + " elements.");
	}
	else if(maxVel <= 0 or maxAccel <= 0)
	{
		throw invalid_argument("[ERROR] [TRAPEZOIDAL VELOCITY] Constructor: "
		                       "Velocity and acceleration arguments were "
		                       + to_string(maxVel) + " and " + to_string(maxAcc) + " but must be positive.");
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
	
	this->_normalisedVel = vel/normaliser;                                                      // Need to normalise so we can scale between 0 and 1
	
	this->_normalisedAcc = maxAcc/normaliser;
	
	this->_dt = vel/maxAcc;                                                                     // Time between corners of trapezoid
	
	this->_t1 = this->_startTime + dt;                                                          // Time to reach 1st corner of trapezoid
	
	this->_s1 = (this->_dt*this->_dt*this->_normalisedAcc)/2.0;
	
	this->_t2 = this->_t1 + (1 - 2*this->_s1 + 0)/this->_normalisedVel;
	
	this->_s2 = this->_s1 + (this->_t2 - this->_t1)/this->_normalisedVel;
	
	this->_endTime = this->_t2 + this->_dt;
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
	
	// Figure out which section of the trapezoid we are on
	
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
		sd  = t*this->_normalisedAcc;
		sdd = this->_normalisedAcc;
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
		
		s   = this->s_2 + t*this->_normalisedVel - (t*t*this->_normalisedAcc)/2.0;
		sd  = this->_normalisedVel - t*this->_normalisedAcc;
		sdd = -this->_normalisedAcc;
	}
	else
	{
		s   = 1.0;
		sd  = 0.0;
		sdd = 0.0;
	}
	
	pos = (1-s)*this->_startPoint - s*this->_endPoint;
	vel =  sd*(this->_endPoint - this->_startPoint);
	acc = sdd*(this->_endPoint - this->_startPoint);
	
	return true;
}


#endif
