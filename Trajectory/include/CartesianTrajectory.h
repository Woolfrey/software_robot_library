    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A minimum acceleration trajectory in Cartesian space                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <CubicSpline.h>                                                                            // Custom trajectory class
#include "Pose.h"

class CartesianTrajectory
{
	public:
	
		CartesianTrajectory();
		
		CartesianTrajectory(const std::vector<Pose>  &waypoint,
		                    const std::vector<float> &time);
		                    
		bool get_state(Pose                      &pose,
			       Eigen::Matrix<float,6,1>  &vel,
			       Eigen::Matrix<float,6,1>  &acc,
			       const float               &time);
	
	private:
		unsigned int numPoints;

		CubicSpline trajectory;                                                             // This is the underlying trajectory object      
		
};                                                                                                  // Semicolon needed after class declaration

#endif
