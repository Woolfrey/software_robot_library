    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A minimum acceleration trajectory in Cartesian space                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <CubicSpline.h>                                                                            // Custom trajectory class
#include <Eigen/Geometry>                                                                           // Eigen::Isometry3f, Eigen::AngleAxisf
#include <vector>                                                                                   // std::vector

class CartesianTrajectory
{
	public:
	
		CartesianTrajectory();
		
		CartesianTrajectory(const std::vector<Eigen::Isometry3f> &waypoint,
                                   const std::vector<float> &time);

		bool get_state(Eigen::Isometry3f &pose,
                               Eigen::VectorXf &vel,
                               Eigen::VectorXf &acc,
                               const float &time);
	
	private:
	
		bool isValid = false;                                                               // Won't do calcs if this is false
		CubicSpline translationTrajectory;                                                  // Translation component
		CubicSpline orientationTrajectory;                                                  // Orientation component
		unsigned int n;                                                                     // Number of waypoints
		
};                                                                                                  // Semicolon needed after class declaration

#endif
