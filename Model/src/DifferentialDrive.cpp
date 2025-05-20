/**
 * @file    DifferentialDrive.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A base class for control of differential drives robots.
 *
 * @details This class is a simple model that serves as a base for control classes.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#include <Model/DifferentialDrive.h>

namespace RobotLibrary { namespace Model {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDrive::DifferentialDrive(const RobotLibrary::Model::DifferentialDriveParameters &parameters)
: _controlFrequency(parameters.controlFrequency),
  _inertia(parameters.inertia),
  _mass(parameters.mass),
  _maxAngularAcceleration(parameters.maxAngularAcceleration),
  _maxAngularVelocity(parameters.maxAngularVelocity),
  _maxLinearAcceleration(parameters.maxLinearAcceleration),
  _maxLinearVelocity(parameters.maxLinearVelocity),
  _propagationUncertainty(parameters.propagationUncertainty)
{
    if (_mass <= 0 or _inertia <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] Constructor: "
                                    "Mass and inertia must be positive. Received mass = " +
                                    std::to_string(_mass) + ", inertia = " + std::to_string(_inertia));
    }

    if (_controlFrequency <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] Constructor: "
                                    "Control frequency must be positive. Received = " +
                                    std::to_string(_controlFrequency));
    }

    if (_maxLinearVelocity <= 0 or _maxAngularVelocity <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] Constructor: "
                                    "Velocity limits must be positive. Received maxLinearVelocity = " +
                                    std::to_string(_maxLinearVelocity) +
                                    ", maxAngularVelocity = " + std::to_string(_maxAngularVelocity));
    }

    if (_maxLinearAcceleration <= 0 or _maxAngularAcceleration <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] Constructor: "
                                    "Acceleration limits must be positive. Received maxLinearAcceleration = " +
                                    std::to_string(_maxLinearAcceleration) +
                                    ", maxAngularAcceleration = " + std::to_string(_maxAngularAcceleration));
    }

    std::string message;
    if (not RobotLibrary::Math::is_positive_definite(_propagationUncertainty, message))
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] Constructor: "
                                    "Propagation uncertainty matrix is not positive definite: " + message);
    }
}


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Udate the pose & velocity                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
DifferentialDrive::update_state(const RobotLibrary::Model::Pose2D &pose,
                                const Eigen::Vector2d &velocity,
                                const Eigen::Matrix3d &covariance)
{
    if (abs(velocity[0]) > _maxLinearVelocity)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] update_state(): "
                                    "(Absolute) linear velocity greater than maximum ("
                                    + std::to_string(abs(velocity[0])) + " > " + std::to_string(_maxLinearVelocity) + ").");
    }
    else if (abs(velocity[1]) > _maxAngularVelocity)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE] update_state(): "
                                    "(Absolute) angular velocity greater than maximum ("
                                    + std::to_string(abs(velocity[1])) + " > " + std::to_string(_maxAngularVelocity) + ").");
    }
    
    _pose = pose;                                                                                   // Simple enough...! 
    _covariance  = covariance;                                                                      // Uncertainty of the pose   
    _velocity[0] = velocity[0];                                                                     // Linear velocity  
    _velocity[1] = velocity[1];                                                                     // Angular velocity
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Partial derivative of propagation equation w.r.t. configuration               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d
DifferentialDrive::configuration_matrix(const RobotLibrary::Model::Pose2D &pose,
                                        const Eigen::Vector2d &velocity)
{
    Eigen::Matrix3d A;
    
    A << 1.0, 0.0, -velocity[0] * sin(pose.angle()) / _controlFrequency,
         0.0, 1.0,  velocity[0] * cos(pose.angle()) / _controlFrequency,
         0.0, 0.0,                      velocity[1] / _controlFrequency;
 
    return A;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Get the next predicted pose based a given pose & velocity               //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Pose2D
DifferentialDrive::predicted_pose(const RobotLibrary::Model::Pose2D &currentPose,
                                  const Eigen::Vector2d &controlInput)
{
    Eigen::Vector2d translation = {currentPose.translation()[0] + controlInput[0] * cos(currentPose.angle()) / _controlFrequency,
                                   currentPose.translation()[1] + controlInput[0] * sin(currentPose.angle()) / _controlFrequency};
    
    double angle = currentPose.angle() + controlInput[1] / _controlFrequency;
    
    return RobotLibrary::Model::Pose2D(translation, angle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Get the predicted covariance                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d
DifferentialDrive::predicted_covariance(const RobotLibrary::Model::Pose2D &currentPose,
                                        const Eigen::Vector2d &controlInput,
                                        const Eigen::Matrix3d &currentCovariance)
{
    Eigen::Matrix3d A = configuration_matrix(currentPose, controlInput);
    
    return A * currentCovariance * A.transpose() + _propagationUncertainty;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Calculate instantaneous limits on linear & angular velocity              //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
DifferentialDrive::compute_limits(RobotLibrary::Model::Limits &linear,
                                  RobotLibrary::Model::Limits &angular,
                                  const Eigen::Vector2d &currentVelocity)
{
    linear.upper = std::min(_maxLinearVelocity,
                            currentVelocity[0] + _maxLinearAcceleration / _controlFrequency);
    
    linear.lower = std::max(-_maxLinearVelocity,
                             currentVelocity[0] - _maxLinearAcceleration / _controlFrequency);
                             
    angular.upper = std::min(_maxAngularVelocity,
                             currentVelocity[1] + _maxAngularAcceleration / _controlFrequency);
    
    angular.lower = std::max(-_maxAngularVelocity,
                              currentVelocity[1] - _maxAngularAcceleration / _controlFrequency);
                              
    if (linear.lower >= linear.upper)
    {
        throw std::logic_error("[ERROR] [DIFFERENTIAL DRIVE] compute_limits(): "
                               "Lower bound for linear velocity is greater than upper bound ("
                               + std::to_string(linear.lower) + " > " + std::to_string(linear.upper) +
                               "). How did that happen???");
    }
    else if (angular.lower >= angular.upper)
    {
        throw std::logic_error("[ERROR] [DIFFERENTIAL DRIVE] compute_limits(): "
                               "Lower bound for angular velocity is greater than upper bound ("
                               + std::to_string(angular.lower) + " > " + std::to_string(angular.upper) +
                               "). How did that happen???");
    }
}
                                  
} } // namespace
