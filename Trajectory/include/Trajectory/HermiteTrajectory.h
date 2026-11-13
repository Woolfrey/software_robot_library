/**
 * @file    HermiteTrajectory.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    March 2026
 * @version 1.0
 * @brief   Generates a trajectory between two poses in SE(2) for a mobile robot.
 * 
 * @details This class uses a Hermite spline to generate a trajectory for a mobile robot whose
 *          kinematics is described by a unicycle model. Its configuration is SE(2), and its control
 *          space is linear velocity (m/s) and angular velocity (rad/s).
 *
 * @copyright (c) 2026 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef HERMITE_TRAJECTORY_H
#define HERMITE_TRAJECTORY_H

#include <Math/Polynomial.h>
#include <Model/Pose2D.h>
#include <Trajectory/DataStructures.h>
#include <vector>                                                                                   // std::vector

namespace RobotLibrary { namespace Trajectory {

/**
 * @brief A data structure for SE(2) robots
 */
struct PlanarState
{
    RobotLibrary::Model::Pose2D pose;
    Eigen::Vector<double,3> twist;
    Eigen::Vector<double,3> acceleration;
};

/**
 * @brief A class for generating splines on the Cartesian plane
 */
class HermiteTrajectory
{
    public:
        
        /**
         * @brief Constructor.
         */     
        HermiteTrajectory(const RobotLibrary::Model::Pose2D &startPose,
                          const RobotLibrary::Model::Pose2D &endPose,
                          const double &startTime,
                          const double &endTime,
                          const Eigen::Vector3d &startTwist = {0.0, 0.0, 0.0});
                              
        /**
         * @brief Get the desired state for the robot at the given time.
         * @param time The thing we are querying.
         * @return A data structure containing the desired pose, and velocity.               
         */
        RobotLibrary::Trajectory::PlanarState
        query_state(const double &time);
        
    private:

        double _startTime;                                                                          ///< When the this trajectory begins
        
        double _endTime;                                                                            ///< When this trajectory ends
        
        Eigen::Vector3d _startTwist;                                                                ///< Initial input

        RobotLibrary::Math::Polynomial _arcParameterTrajectory;                                     ///< Arc parameter s(t)
                        
        RobotLibrary::Math::Polynomial _xSpline;                                                    ///< x-position as a function of arc length s
        
        RobotLibrary::Math::Polynomial _ySpline;                                                    ///< y-position as a function of arc length s
          
        RobotLibrary::Model::Pose2D _startPose;                                                     ///< Initial pose for the trajectory
            
        RobotLibrary::Model::Pose2D _endPose;                                                       ///< Final pose for the trajectory
       
};                                                                                                  // Semicolon needed after a class declaration

} }

#endif
