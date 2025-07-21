/**
 * @file    DifferentialDrive.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   A base class for control of differential drives robots.
 *
 * @details This class is a simple model that serves as a base for control classes.
 *
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#include <Model/DifferentialDrive.h>

namespace RobotLibrary { namespace Model {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDrive::DifferentialDrive(const RobotLibrary::Model::DifferentialDriveParameters &parameters)
: _inertia(parameters.inertia),
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
DifferentialDrive::configuration_jacobian(const RobotLibrary::Model::Pose2D &pose,
                                          const Eigen::Vector2d &velocity,
                                          const double &controlFrequency)
{
    Eigen::Matrix3d A;
    
    A << 1.0, 0.0, -velocity[0] * sin(pose.angle()) / controlFrequency,
         0.0, 1.0,  velocity[0] * cos(pose.angle()) / controlFrequency,
         0.0, 0.0,                                                 1.0;
 
    return A;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Partial derivative of propagation equation w.r.t. control                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,3,2>
DifferentialDrive::control_jacobian(const RobotLibrary::Model::Pose2D &pose,
                                    const double &controlFrequency)
{
    Eigen::Matrix<double,3,2> B;
    
    B << cos(pose.angle()) / controlFrequency,                    0.0,
         sin(pose.angle()) / controlFrequency,                    0.0,
                                          0.0, 1.0 / controlFrequency;

    return B;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Get the next predicted pose based a given pose & velocity               //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Pose2D
DifferentialDrive::predicted_pose(const RobotLibrary::Model::Pose2D &currentPose,
                                  const Eigen::Vector2d &currentVelocity,
                                  const double &controlFrequency)
{
    Eigen::Vector2d translation = {currentPose.translation()[0] + currentVelocity[0] * cos(currentPose.angle()) / controlFrequency,
                                   currentPose.translation()[1] + currentVelocity[0] * sin(currentPose.angle()) / controlFrequency};
    
    double angle = currentPose.angle() + currentVelocity[1] / controlFrequency;
    
    return RobotLibrary::Model::Pose2D(translation, angle);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Get the predicted covariance                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d
DifferentialDrive::predicted_covariance(const RobotLibrary::Model::Pose2D &currentPose,
                                        const Eigen::Vector2d &currentVelocity,
                                        const Eigen::Matrix3d &currentCovariance,
                                        const double &controlFrequency)
{
    Eigen::Matrix3d A = configuration_jacobian(currentPose, currentVelocity, controlFrequency);
    
    return A * currentCovariance * A.transpose() + _propagationUncertainty;
}
                                  
} } // namespace
