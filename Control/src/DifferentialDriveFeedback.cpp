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
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Control/DifferentialDriveFeedback.h>

namespace RobotLibrary { namespace Control {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDriveFeedback::DifferentialDriveFeedback(const RobotLibrary::Model::DifferentialDriveParameters &modelParameters,
                                                     const RobotLibrary::Control::DifferentialDriveFeedbackParameters &controlParameters)
: DifferentialDriveBase(controlParameters.controlFrequency,
                        controlParameters.minimumSafeDistance,
                        modelParameters),
  _orientationGain(controlParameters.orientationGain),
  _xPositionGain(controlParameters.xPositionGain),
  _yPositionGain(controlParameters.yPositionGain)
{
    if (_xPositionGain <= 0 or _yPositionGain <= 0 or _orientationGain <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE FEEDBACK] Constructor: "
                                    "Feedback control gains must be positive, but "
                                    "the x position gain was " + std::to_string(_xPositionGain) + ", "
                                    "the y position gain was " + std::to_string(_yPositionGain) + ", and "
                                    "the orientation gain was " + std::to_string(_orientationGain) + ".");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Solve the control to track a desired trajectory                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
DifferentialDriveFeedback::track_trajectory(const RobotLibrary::Model::Pose2D &desiredPose,
                                            const Eigen::Vector2d &desiredVelocity)
{
    // Kanayama, Y., Kimura, Y., Miyazaki, F., & Noguchi, T. (1990, May).
    // A stable tracking control method for an autonomous mobile robot.
    // In Proceedings., IEEE International Conference on Robotics and Automation (pp. 384-389). IEEE.
    
    // Pose error in the GLOBAL frame
    Eigen::Vector3d e = _pose.error(desiredPose);
    
    // Position error in the LOCAL frame
    double epsilon_x =  e[0] * cos(_pose.angle()) + e[1] * sin(_pose.angle());
    double epsilon_y = -e[0] * sin(_pose.angle()) + e[1] * cos(_pose.angle());
    
    // Compute the feedback control
    double linearVelocity  = desiredVelocity[0] * cos(e[2]) + _xPositionGain * epsilon_x;
    double angularVelocity = desiredVelocity[1] + desiredVelocity[0] * epsilon_y + _yPositionGain * epsilon_y + _orientationGain * sin(e[2]);
    
    // Limit the results
    RobotLibrary::Model::Limits linear, angular;
    compute_control_limits(linear, angular, _velocity);
    
    linearVelocity  = std::clamp(linearVelocity,   linear.lower,  linear.upper);
    angularVelocity = std::clamp(angularVelocity, angular.lower, angular.upper);
    
    return {linearVelocity, angularVelocity};
}

} } // Namespace                                                                                      
