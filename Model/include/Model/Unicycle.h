/**
 * @file    Unicycle.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
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

#ifndef UNICYCLE_H
#define UNICYCLE_H

#include <Model/DataStructures.h>                                                                    // Limits
#include <Model/Pose2D.h>
#include <Model/RigidBody2D.h>
#include <cmath>                                                                                    // M_PI

namespace RobotLibrary { namespace Model {

/**
 * @brief A struct for collating parameters for the Unicycle
 */
struct UnicycleParameters
{
    double inertia                = 0.5 * 5.0 * 0.25 * 0.25;                                        ///< Rotational inertia (kg*m^2)
    double mass                   = 5.0;                                                            ///< Weight (kg)
    double maxAngularAcceleration = 10.0;                                                           ///< Maximum rotational acceleration (rad/s/s)
    double maxAngularVelocity     = 100.0 * M_PI / 30.0;                                            ///< Maximum rotational speed (rad/s)
    double maxLinearAcceleration  = 5.0;                                                            ///< Maximum forward acceleration (m/s/s)
    double maxLinearVelocity      = 2.0;                                                            ///< Maximum forward speed (m/s)
    double minimumSafeDistance    = 0.5;                                                            ///< Used for collision detection

    Eigen::Matrix3d propagationUncertainty = Eigen::Matrix3d::Identity();                           ///< Uncertainty of configuration propagation in Kalman filter

    Eigen::Vector3d robotRadii    = {0.3, 0.3, 0.3};
    Eigen::Vector3d robotLengths  = {-0.0, 0.0, 0.0};

    UnicycleParameters() = default;
};

/**
 * @brief A struct that fully defines the state of a differential drive robot.
 */
struct UnicycleState
{
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();                                       ///< Uncertainty of the pose
    Eigen::Vector2d velocity = {0.0, 0.0};                                                          ///< Linear & angular velocity, respectively
    RobotLibrary::Model::Pose2D pose;                                                               ///< Position & orientation relative to a reference frame.

    UnicycleState() = default;

    UnicycleState(const RobotLibrary::Model::Pose2D& p,
                  const Eigen::Vector2d& v)
    : pose(p), velocity(v) {}
};

/**
 * @brief A class for modeling the kinematics of a planar, mobile robot.
 */
class Unicycle : public RigidBody2D
{
    public:
    
        /**
         * @brief Constructor.
         */
        Unicycle(const RobotLibrary::Model::UnicycleParameters &parameters);
        
        /**
         * @brief Update the internal pose, covariance, and velocity of the model
         * @param pose The position & orientation relative to a global coordinate frame
         * @param covariance The uncertainty of the pose
         * @param velocity The linear & angular velocity
         */
        void
        update_state(const RobotLibrary::Model::Pose2D &pose,
                     const Eigen::Vector2d &velocity,
                     const Eigen::Matrix3d &covariance = Eigen::Matrix3d::Identity());
        
        /**
         * @brief Get the current velocity.
         * @return A 2D vector containing the linear velocity (m/s), and angular velocity (rad/s).
         */
        Eigen::Vector2d
        velocity();
        
        /**
         * @brief Get the (predicted) next pose given a current pose & velocity.
         * @param currentPose The current position & orientation.
         * @param currentVelocity The current linear & angular velocity.
         * @return A pose object as SE(2).
         */
        RobotLibrary::Model::Pose2D
        predicted_pose(const RobotLibrary::Model::Pose2D &currentPose,
                       const Eigen::Vector2d &currentVelocity,
                       const double &controlFrequency);
       
        /**
         * @brief Get uncertainty of the predicted pose.
         * @param currentPose The current position & orientation
         * @param currentCovariance The current uncertainty (to be propagated)
         * @param currentVelocity The current linear & angular velocity
         * @return A 3x3 matrix.
         */
        Eigen::Matrix3d
        predicted_covariance(const RobotLibrary::Model::Pose2D &currentPose,
                             const Eigen::Vector2d &controlInput,
                             const Eigen::Matrix3d &currentCovariance,
                             const double &controlFrequency);
             
        /**
         * @brief Partial derivative of configuration propagation w.r.t configuration.
         * @param pose The current position & orientation
         * @param velocity The current linear & angular velocity
         * @return A 3x3 matrix.
         */
        Eigen::Matrix3d
        configuration_jacobian(const RobotLibrary::Model::Pose2D &pose,
                               const Eigen::Vector2d &velocity,
                               const double &controlFrequency);
           
        /**
         * @brief Partial derivative of configuration propagation w.r.t. control input.
         * @param pose The current position & orientation.
         * @param velocity The current linear and angular velocity.
         * @return a 3x2 matrix.
         */
        Eigen::Matrix<double,3,2>
        control_jacobian(const RobotLibrary::Model::Pose2D &pose,
                         const double &controlFrequency);
                         
    protected:
         
        double _maxAngularAcceleration;                                                             ///< Maximum rotational acceleration (rad/s/s)
        
        double _maxAngularVelocity;                                                                 ///< Maximum rotational speed (rad/s)
        
        double _maxLinearAcceleration;                                                              ///< Maximum forward/backward acceleration (m/s/s)
        
        double _maxLinearVelocity;                                                                  ///< Maximum forward/backward speed (m/s)
        
        double _minimumSafeDistance;                                                                ///< For safety / collision checking
        
        Eigen::Matrix3d _covariance = Eigen::Matrix3d::Identity();                                  ///< Uncertainty of the pose

        Eigen::Matrix3d _propagationUncertainty = Eigen::Matrix3d::Identity();                      ///< Additional uncertainty on pose propagation
        
        Eigen::Vector3d _robotRadii = {0.2,0.3,0.0};

        Eigen::Vector3d _robotLengths = {-0.2, 0.0, 0.2};
}; 

} } // namespace

#endif                                                                                             // Semicolon needed after class declaration
