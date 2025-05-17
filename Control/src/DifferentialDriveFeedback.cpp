/**
 * @file    DifferentialDriveFeedback.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Source files for the DifferentialDriveFeedback control class.
 *
 * @details This class provides method for implementing nonlinear feedback control of a differential
 *          drive robot.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Control/DifferentialDriveFeedback.h>

namespace RobotLibrary { namespace Control {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDriveFeedback::DifferentialDriveFeedback(const double &xPositionGain,
                                                     const double &yPositionGain,
                                                     const double &orientationGain,
                                                     const RobotLibrary::Model::DifferentialDriveParameters &parameters)
: DifferentialDrive(parameters),
  _xPositionGain(xPositionGain),
  _yPositionGain(yPositionGain),
  _orientationGain(orientationGain)
{
    if (xPositionGain <= 0 or yPositionGain <= 0 or orientationGain <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE FEEDBACK] Constructor: "
                                    "Feedback control gains must be positive, but "
                                    "the x position gain was " + std::to_string(xPositionGain) + ", "
                                    "the y position gain was " + std::to_string(yPositionGain) + ", and "
                                    "the orientation gain was " + std::to_string(orientationGain) + ".");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Solve the control to track a desired trajectory                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
DifferentialDriveFeedback::track_trajectory(const RobotLibrary::Model::Pose2D &desiredPose,
                                            const Eigen::Vector2d &desiredVelocity)
{
    // Pose error in the GLOBAL frame
    double e_x   = desiredPose.translation()[0] - _pose.translation()[0];
    double e_y   = desiredPose.translation()[1] - _pose.translation()[1];
    double e_psi = desiredPose.angle() - _pose.angle();
    
    // Position error in the LOCAL frame
    double epsilon_x =  e_x * cos(_pose.angle()) + e_y * sin(_pose.angle());
    double epsilon_y = -e_x * sin(_pose.angle()) + e_y * cos(_pose.angle());
    
    // Compute the feedback control
    double linearVelocity  = desiredVelocity[0] * cos(e_psi) + _xPositionGain * epsilon_x;
    double angularVelocity = desiredVelocity[1] + desiredVelocity[0] * epsilon_y + _yPositionGain * epsilon_y + _orientationGain * sin(e_psi);
    
    // Limit the results
    RobotLibrary::Model::Limits linear, angular;
    compute_limits(linear, angular, _velocity);
    
    linearVelocity  = std::clamp(linearVelocity,   linear.lower,  linear.upper);
    angularVelocity = std::clamp(angularVelocity, angular.lower, angular.upper);
    
    return {linearVelocity, angularVelocity};
}

} } // Namespace                                                                                      
