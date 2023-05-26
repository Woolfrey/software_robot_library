#include <TrapezoidalVelocity.h>


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
TrapezoidalVelocity::TrapezoidalVelocity(const Eigen::VectorXf &startPoint,
                                         const Eigen::VectorXf &endPoint,
                                         const float           &velocity,
                                         const float           &acceleration)
                                         :
                                         TrajectoryBase(_startTime,_endTime,dimensions),
                                         maxVel(velocity),
                                         maxAcc(acceleration)
                                         
{
	
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the state for the given time                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool TrapezoidalVelocity::get_state(Eigen::VectorXf &pos,
                                    Eigen::VectorXf &vel,
                                    Eigen::VectorXf &acc,
                                    const float     &time)
{
    return true;
}
