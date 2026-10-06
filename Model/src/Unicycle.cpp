/**
 * @file    Unicycle.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A base class for control of UNICYCLEs robots.
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

#include <Model/Unicycle.h>

namespace RobotLibrary { namespace Model {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
Unicycle::Unicycle(const RobotLibrary::Model::UnicycleParameters &parameters)
: RigidBody2D(parameters.mass, parameters.inertia),
  _maxAngularAcceleration(parameters.maxAngularAcceleration),
  _maxAngularVelocity(parameters.maxAngularVelocity),
  _maxLinearAcceleration(parameters.maxLinearAcceleration),
  _maxLinearVelocity(parameters.maxLinearVelocity),
  _minimumSafeDistance(parameters.minimumSafeDistance),
  _propagationUncertainty(parameters.propagationUncertainty),
  _robotRadii(parameters.robotRadii),
  _robotLengths(parameters.robotLengths)
{
    if (_mass <= 0.0 or _inertia <= 0.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] Constructor: "
                                    "Mass and inertia must be positive. Received mass = " +
                                    std::to_string(_mass) + ", inertia = " + std::to_string(_inertia));
    }
    
    if (_maxLinearVelocity <= 0.0 or _maxAngularVelocity <= 0.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] Constructor: "
                                    "Velocity limits must be positive. Received maxLinearVelocity = " +
                                    std::to_string(_maxLinearVelocity) +
                                    ", maxAngularVelocity = " + std::to_string(_maxAngularVelocity));
    }

    if (_maxLinearAcceleration <= 0.0 or _maxAngularAcceleration <= 0.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] Constructor: "
                                    "Acceleration limits must be positive. Received maxLinearAcceleration = " +
                                    std::to_string(_maxLinearAcceleration) +
                                    ", maxAngularAcceleration = " + std::to_string(_maxAngularAcceleration));
    }
    
    if (_minimumSafeDistance < 0.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] Constructor: "
                                    "Minimum safe distance must be positive. Received minimumSafeDistance = " +
                                    std::to_string(_minimumSafeDistance) + ".");
    }

    std::string message;
    if (not RobotLibrary::Math::is_positive_definite(_propagationUncertainty, message))
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] Constructor: "
                                    "Propagation uncertainty matrix is not positive definite: " + message);
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Udate the pose & velocity                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Unicycle::update_state(const RobotLibrary::Model::Pose2D &pose,
                       const Eigen::Vector2d &velocity,
                       const Eigen::Matrix3d &covariance)
{
    if (abs(velocity[0]) > _maxLinearVelocity)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] update_state(): "
                                    "(Absolute) linear velocity greater than maximum ("
                                    + std::to_string(abs(velocity[0])) + " > " + std::to_string(_maxLinearVelocity) + ").");
    }
    else if (abs(velocity[1]) > _maxAngularVelocity)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE] update_state(): "
                                    "(Absolute) angular velocity greater than maximum ("
                                    + std::to_string(abs(velocity[1])) + " > " + std::to_string(_maxAngularVelocity) + ").");
    }

    _covariance  = covariance;                                                                      // Uncertainty of the pose   
    
    // Update properties of underlying RigidBody2D object  
    _pose     = pose;                                                                               // Simple enough...!
    _twist[0] = velocity[0] * cos(_pose.angle());
    _twist[1] = velocity[0] * sin(_pose.angle());
    _twist[2] = velocity[1];
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Get the linear and angular velocity                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
Unicycle::velocity()
{
    Eigen::Vector2d velocity = { _twist[0] * cos(_pose.angle()) + _twist[1] * sin(_pose.angle()),
                                 _twist[2] };
                                 
    return velocity;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Partial derivative of propagation equation w.r.t. configuration               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d
Unicycle::configuration_jacobian(const RobotLibrary::Model::Pose2D &pose,
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
Unicycle::control_jacobian(const RobotLibrary::Model::Pose2D &pose,
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
Unicycle::predicted_pose(const RobotLibrary::Model::Pose2D &currentPose,
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
Unicycle::predicted_covariance(const RobotLibrary::Model::Pose2D &currentPose,
                               const Eigen::Vector2d &currentVelocity,
                               const Eigen::Matrix3d &currentCovariance,
                               const double &controlFrequency)
{
    Eigen::Matrix3d A = configuration_jacobian(currentPose, currentVelocity, controlFrequency);
    
    return A * currentCovariance * A.transpose() + _propagationUncertainty;
}
                                  
} } // namespace
