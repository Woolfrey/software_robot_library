    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                            A base class for polynomial trajectories                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <Eigen/Geometry>                                                                          // Eigen::VectorXf, Eigen::AngleAxisf
#include <math.h>                                                                                  // M_PI
#include <vector>                                                                                  // std::vector
#include <iostream>
class Polynomial
{
	public:
	
		Polynomial();

		// Constructor with only positions, delegates zero velocity
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order) :
		Polynomial(startPoint,
                          endPoint,
                          Eigen::VectorXf::Zero(startPoint.size()),
                          Eigen::VectorXf::Zero(endPoint.size()),
                          startTime,
                          endTime,
                          order) {}
		
		// Constructor with positions and velocities
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const Eigen::VectorXf &startVelocity,
			   const Eigen::VectorXf &endVelocity,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order);
		
		// Constructor with orientation, delegates zero velocity
		Polynomial(const Eigen::AngleAxisf &startPoint,
			   const Eigen::AngleAxisf &endPoint,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order) :
		Polynomial(startPoint,
               	   endPoint,
               	   Eigen::Vector3f::Zero(),
               	   Eigen::Vector3f::Zero(),
               	   startTime,
               	   endTime,
               	   order) {}
		
		// Constructor with orientation and velocity
		Polynomial(const Eigen::AngleAxisf &startPoint,
			   const Eigen::AngleAxisf &endPoint,
			   const Eigen::Vector3f &startVelocity,
			   const Eigen::Vector3f &endVelocity,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order);
		
		// Get the desired state for translations
		bool get_state(Eigen::VectorXf &pos,
			       Eigen::VectorXf &vel,
			       Eigen::VectorXf &acc,
			       const float &time);
		
		// Get the desired state for rotations
		bool get_state(Eigen::AngleAxisf &rot,
			       Eigen::Vector3f &vel,
			       Eigen::Vector3f &acc,
			       const float &time);
		
	private:
	
		bool isValid = false;                                                              // Won't do calcs if this is false
		Eigen::AngleAxisf R0;                                                              // Start rotation: R(t) = R0*dR(t)
		float t0, tf;                                                                      // Start time and end time			
		unsigned int m;                                                                    // Number of dimensions
		unsigned int n;                                                                    // Order of polynomial
		std::vector<std::vector<float>> coeff;                                            
		
		bool times_are_sound(const float &startTime, const float &endTime);
		
		bool compute_coefficients(const std::vector<float> &startPoint,
					   const std::vector<float> &endPoint,
					   const std::vector<float> &startVelocity,
					   const std::vector<float> &endVelocity);
		
};                                                                                                 // Semicolon needed after class declaration

#endif
