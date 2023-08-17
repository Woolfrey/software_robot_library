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
		DataType _maxVel,
		         _maxAcc;
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
TrapezoidalVelocity<DataType>::TrapezoidalVelocity(const Vector<DataType,Dynamic> &startPoint,
                                                   const Vector<DataType,Dynamic> &endPoint,
                                                   const DataType                 &maxVel,
                                                   const DataType                 &maxAcc)
                                                   :
                                                   TrajectoryBase(_startTime,_endTime,dimensions),
                                                   maxVel(velocity),
                                                   maxAcc(acceleration)
                                         
{
	
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
