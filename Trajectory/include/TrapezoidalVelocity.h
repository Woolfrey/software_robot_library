    ////////////////////////////////////////////////////////////////////////////////////////////////////  
   //                                                                                                //
  //                              A constant velocity trajectory                                    //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Vector, Eigen::Matrix, Eigen::Quaternion
#include <TrajectoryBase.h>

using namespace Eigen;                                                                              // Eigen::Vector

template <class DataType>
class TrapezoidalVelocity : public TrajectoryBase<DataType>
{
	public:
		// Empty constructor
		TrapezoidalVelocity() {}
		
		// Constructor for translations
		TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                    const Vector<DataType,Dynamic> &endPoint,
                                    const DataType                 &maxVel,
                                    const DataType                 &maxAcc,
                                    const DataType                 &startTime;
		
		// Get state for translations
		bool get_state(Vector<DataType,Dynamic> &pos,
		               Vector<DataType,Dynamic> &vel,
		               Vector<DataType,Dynamic> &acc,
		               const DataType           &time);			
				
	private:
		DataType _maxVel;                                                                   ///< Maximum speed
		DataType _maxAccel;                                                                 ///< Maximum acceleration
		DataType _dt;                                                                       ///< Ration of speed over acceleration
		DataType _t1;
		DataType _t2;
		
		Vector<DataType,Dynamic> p1;
		Vector<DataType,Dynamic> p2;
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
	
	// Ensure the maximum velocity is not too large
	for(int i = 0; i < startPoint.size(); i++)
	{
		DataType temp = sqrt(maxAcc*abs(endPoint(i) - startPoint(i)));
		
		if( this->_maxVel >= temp)
		{
			cout << "[WARNING] [TRAPEZOIDAL VELOCITY] Constructor: "
			     << "Maximum velocity too high for the distance travelled. "
			     << "Overriding value.\n";
			     
			this->_maxVel = temp;
		}
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
