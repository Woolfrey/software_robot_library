/**
 * @file    MinimumArcLength.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   A class that generates arcs over a 2D plane with a minimum path length.
 *
 * @details This is used for creating curves over a 2D plane with a minimum path length. Internally
 *          it is parameterised in polar coordinates. The angle parameter follows a cubic spline,
 *          whereas the radius parameter is a hyperbolic cosine. It is suitable for short paths for
 *          differential drive robots subject to non-holonomic constraints.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef MINIMUM_ARC_LENGTH_H
#define MINIMUM_ARC_LENGTH_H

#include <Math/Polynomial.h>                                                                        // Generates odd polynomial functions
#include <Model/Pose2D.h>                                                                           // SE(2) object
#include <Trajectory/TrajectoryBase.h>                                                              // Standardisation

namespace RobotLibrary { namespace Trajectory {

/**
 * Generates a trajectory for a wheeled mobile robot in SE(2).
 * The underlying representation is in polar coordinates since it is congruent with the underactuated system.
 */
class MinimumArcLength : public TrajectoryBase
{
	public:
	
 		/**
 		 * @brief Constructor. Note that for this trajectory the final orientation cannot be constrained.s
 		 * @param startPose The initial pose of the robot in a global coordinate frame.
 		 * @param endPoint The final point (x,y) for the robot in a global coordinate frame.
 		 * @param startTime The start time for the trajectory.
 		 * @param endTime The end time for the trajectory
 		 */
 		 MinimumArcLength(const RobotLibrary::Model::Pose2D &startPose,
 		                  const Eigen::Vector2d &endPoint,
 		                  const double &startTime,
 		                  const double &endTime);
		
		/**
		 * @brief Get the desired pose (x, y, \psi) and velocity (v, \omega) for the given time.
		 * @details The RobotLibrary::Trajectory::State struct defines position and velocity as
		 *          vectors, so the position will be a 3x1 vector, and the velocity and acceleration is 2x1.
		 *          Note this overrides the virtual method in the base class.
		 * @param time The time at which to query the underlying trajecory.
		 * @return A State data structure containing the desired pose and control input.
		 */
        RobotLibrary::Trajectory::State
        query_state(const double &time);
 		
 	private:
 	
        bool _straightLine = false;                                                                 ///< Flags if the trajectory is a straight line or not

        double _c1, _c2;                                                                            ///< Coefficients for radial parameter interpolation
        
        int _directionChange;                                                                       ///< We need this to determine the correct heading
        
 	    RobotLibrary::Model::Pose2D _initialPose;                                                   ///< We need this to transform the trajectory between coordinate frames
 	  
 		RobotLibrary::Math::Polynomial _polynomial;                                                 ///< Forms a trajectory for 1 parameter
	
	    RobotLibrary::Trajectory::State _state;                                                     ///< Send this back when querying the state
	    
};                                                                                                  // Semicolon needed after class declaration

} } // namespace
#endif
