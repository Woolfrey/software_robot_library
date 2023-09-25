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
                                    const DataType                 &maxAcc);
		
		// Get state for translations
		bool get_state(Vector<DataType,Dynamic> &pos,
		               Vector<DataType,Dynamic> &vel,
		               Vector<DataType,Dynamic> &acc,
		               const DataType           &time);			
				
	private:
		DataType _maxVel;                                                                   ///< Maximum speed
		DataType _maxAccel;                                                                 ///< Maximum acceleration
		DataType _dt;                                                                       ///< Ration of speed over acceleration
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                                   const Vector<DataType,Dynamic> &endPoint,
                                                   const DataType                 &maxVel,
                                                   const DataType                 &maxAcc)                          
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
    return true;
}


#endif
