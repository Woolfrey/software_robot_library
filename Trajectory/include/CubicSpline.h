    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A minimum acceleration trajectory across multiple points                   //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <Eigen/Dense>                                                                              // Matrix inverse
#include <Polynomial.h>                                                                             // Custom class

class CubicSpline
{
	public:
	
		CubicSpline() {}                                                                    // Won't compile without {} for some reason ಠ_ಠ 

		CubicSpline(const std::vector<Eigen::VectorXf> &waypoint,
			    const std::vector<float> &time);

		bool get_state(Eigen::VectorXf &pos,
			       Eigen::VectorXf &vel,
			       Eigen::VectorXf &acc,
			       const float     &time);
					   
	private:
	
		std::vector<Polynomial> spline;                                                     // Array of cubic polynomials
		std::vector<float> _time;
		
		unsigned int dimensions;
		unsigned int numPoints;                                                             // Number of waypoints
							  
		std::vector<Eigen::VectorXf> compute_velocities(const std::vector<Eigen::VectorXf> &pos,
								const std::vector<float> &time);
	
};                                                                                                  // Semicolon needed after class declaration

#endif
