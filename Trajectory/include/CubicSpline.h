    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A minimum acceleration trajectory across multiple points                   //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <Polynomial.h>                                                                             // Custom class

class CubicSpline
{
	public:
	
		CubicSpline() {}                                                                    // Won't compile without {} for some reason ಠ_ಠ 

		CubicSpline(const std::vector<Eigen::VectorXf> &waypoint,
			    const std::vector<float> &time);
					
		CubicSpline(const std::vector<Eigen::AngleAxisf> &waypoint,
			    const std::vector<float> &time);

		bool get_state(Eigen::VectorXf &pos,
			       Eigen::VectorXf &vel,
			       Eigen::VectorXf &acc,
			       const float &time);
		
		bool get_state(Eigen::AngleAxisf &rot,
			       Eigen::Vector3f &vel,
			       Eigen::Vector3f &acc,
			       const float &time);
					   
	private:
	
		bool isValid = false;                                                               // Won't do calcs if this is false
		std::vector<Polynomial> spline;                                                     // Array of cubic polynomials
		std::vector<float> t;                                                               // Time at which to pass each waypoint
		unsigned int m;                                                                     // Number of dimensions
		unsigned int n;                                                                     // Number of waypoints (for n-1 splines)
		
		bool times_are_sound(const std::vector<float> &time);
							  
		std::vector<Eigen::VectorXf> compute_velocities(const std::vector<Eigen::VectorXf> &pos,
								  const std::vector<float> &time);
	
};                                                                                                  // Semicolon needed after class declaration

#endif
