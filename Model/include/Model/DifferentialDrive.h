/**
 * @file    DifferentialDrive.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
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

#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Model/DataStructures.h>
#include <Model/Pose2D.h>

namespace RobotLibrary { namespace Model {

/**
 * @brief A class for modeling the kinematics of a planar, mobile robot.
 */
class DifferentialDrive
{
    public:
    
        /**
         * @brief Constructor.
         */
        DifferentialDrive(const RobotLibrary::Model::DifferentialDriveParameters &parameters);
        
        /**
         * @brief Update the internal pose, covariance, and velocity of the model
         * @param pose The position & orientation relative to a global coordinate frame
         * @param covariance The uncertainty of the pose
         * @param velocity The linear & angular velocity
         */
        void
        update_state(const RobotLibrary::Model::Pose2D &pose,
                     const Eigen::Matrix3d &covariance,
                     const Eigen::Vector2d &velocity);

        /**
         * @brief Compute the instantaneous limits on the linear & angular velocity.
         * @param Storage for the linear velocity limits.
         * @param Storage for the angular velocity limits.
         */
        void
        compute_limits(RobotLibrary::Model::Limits &linear,
                       RobotLibrary::Model::Limits &angular,
                       const Eigen::Vector2d &currentVelocity);
                                       
        /**
         * @brief Get the current pose of the robot.
         * @return What you asked for.
         */
        RobotLibrary::Model::Pose2D
        pose() const { return _pose; }
        
        /**
         * @brief Get the current velocity of the robot.
         */
        Eigen::Vector2d
        velocity() const { return _velocity; }
        
        /**
         * @brief Get the mass of the robot.
         */
        double
        mass() const { return _mass; }
        
        /**
         * @brief Get the inertia of the robot.
         */
        double
        inertia() const { return _inertia; }
        
        /**
         * @brief Get the (predicted) next pose given a current pose & velocity.
         * @param currentPose The current position & orientation.
         * @param currentVelocity The current linear & angular velocity.
         * @return A pose object as SE(2).
         */
        RobotLibrary::Model::Pose2D
        predicted_pose(const RobotLibrary::Model::Pose2D &currentPose,
                       const Eigen::Vector2d &currentVelocity);

        /**
         * @brief Get uncertainty of the predicted pose.
         * @param currentPose The current position & orientation
         * @param currentCovariance The current uncertainty (to be propagated)
         * @param currentVelocity The current linear & angular velocity
         * @return A pose object as SE(2)
         */
        Eigen::Matrix3d
        predicted_covariance(const RobotLibrary::Model::Pose2D &currentPose,
                             const Eigen::Matrix3d &currentCovariance,
                             const Eigen::Vector2d &controlInput);
                       
        /**
         * @brief Partial derivative of configuration propagation w.r.t configuration.
         * @param pose The current position & orientation
         * @param velocity The current linear & angular velocity
         * @return A pose object as SE(2)
         */
        Eigen::Matrix3d
        configuration_matrix(const RobotLibrary::Model::Pose2D &pose,
                             const Eigen::Vector2d &velocity);     
    protected:
        
        double _controlFrequency;                                                                   ///< Used in certain methods
        
        double _inertia;                                                                            ///< Rotational inertia of the robot (kg*m^2)
        
        double _mass;                                                                               ///< Mass of the robot (kg)
        
        double _maxAngularAcceleration;                                                             ///< Maximum rotational acceleration (rad/s/s)
        
        double _maxAngularVelocity;                                                                 ///< Maximum rotational speed (rad/s)
        
        double _maxLinearAcceleration;                                                              ///< Maximum forward/backward acceleration (m/s/s)
        
        double _maxLinearVelocity;                                                                  ///< Maximum forward/backward speed (m/s)
        
        Eigen::Vector2d _velocity = {0.0, 0.0};                                                     ///< Forward speed (m/s), and turn rate (rad/s)      
                
        RobotLibrary::Model::Pose2D _pose;                                                          ///< Position & orientation of the robot
        
        Eigen::Matrix3d _covariance = Eigen::Matrix3d::Identity();                                  ///< Uncertainty of the pose

        Eigen::Matrix3d _propagationUncertainty = Eigen::Matrix3d::Identity();                      ///< Additional uncertainty on pose propagation

}; 

} } // namespace

#endif                                                                                             // Semicolon needed after class declaration
