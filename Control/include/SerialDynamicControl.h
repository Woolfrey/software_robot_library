    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                        A class for dynamic control of a serial link robot                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIALDYNAMICCONTROL_H_
#define SERIALDYNAMICCONTROL_H_

#include <Eigen/Core>
#include <QPSolver.h>  
#include <SerialLink.h>                                                                         

class SerialDynamicControl : public SerialLink,
                             public QPSolver
{
	public:
		SerialDynamicControl();

		Eigen::VectorXf accelerate_endpoint(const Eigen::VectorXf &accel,
                                                    const Eigen::VectorXf &redundant);
                                                    
                Eigen::VectorXf feedback_linearization();                                           // Return C*qdot + g
                
	private:
		
		bool get_accel_limit(float lower, float upper, const int &jointNum);
		
		float get_penalty_torque(const int &jointNum);
};                                                                                                 // Semicolon needed after class declaration

#endif

