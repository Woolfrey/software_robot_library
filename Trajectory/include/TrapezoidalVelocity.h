    ////////////////////////////////////////////////////////////////////////////////////////////////////  
   //                                                                                                //
  //                              A constant velocity trajectory                                    //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include <Eigen/Geometry>                                                                           // Eigen::Vector, Eigen::Matrix, Eigen::Quaternion
#include "TrajectoryBase.h"

class TrapezoidalVelocity : public TrajectoryBase
{
	public:
		// Empty constructor
		TrapezoidalVelocity() {}
		
		// Constructor for translations
		TrapezoidalVelocity(const Eigen::VectorXf &startPoint,
                                    const Eigen::VectorXf &endPoint,
                                    const float           &velocity,
                                    const float           &acceleration);
		
		// Get state for translations
		bool get_state(Eigen::VectorXf &pos,
		               Eigen::VectorXf &vel,
		               Eigen::VectorXf &acc,
		               const float &time);			
				
	private:
		float maxVel, maxAcc;
		
};                                                                                                  // Semicolon needed after class declaration

#endif
