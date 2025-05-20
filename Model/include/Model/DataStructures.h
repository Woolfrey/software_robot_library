/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.1
 * @brief   Structs used in the Model classes.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef MODEL_DATA_STRUCTS_H
#define MODEL_DATA_STRUCTS_H

#include <Model/Pose.h>
#include <Model/Pose2D.h>

namespace RobotLibrary { namespace Model {

class Link;

/**
 * @brief A data structure for joint limits.
 *        It could represent position, velocity, acceleration, or torque depending on the context.
 */
struct Limits
{
     double lower;                                                                                  ///< Upper limit
     double upper;                                                                                  ///< Lower limit
};                                                                                                  // Semicolon needed after declaring a data structure

/**
 * @brief A structure containing necessary information for defining a reference frame on a kinematic tree.
 */
struct ReferenceFrame
{
     Link *link = nullptr;                                                                          ///< The link it is attached to
     Pose relativePose;                                                                             ///< Pose with respect to local link frame
};

/**
 * @brief A struct for collating parameters for the DifferentialDrive
 */
struct DifferentialDriveParameters
{
    double controlFrequency       = 100.0;                                                          ///< Used by controllers
    double inertia                = 0.5 * 5.0 * 0.25 * 0.25;                                        ///< Rotational inertia (kg*m^2)
    double mass                   = 5.0;                                                            ///< Weight (kg)
    double maxAngularAcceleration = 10.0;                                                           ///< Maximum rotational acceleration (rad/s/s)
    double maxAngularVelocity     = 100.0 * M_PI / 30.0;                                            ///< Maximum rotational speed (rad/s)
    double maxLinearAcceleration  = 5.0;                                                            ///< Maximum forward acceleration (m/s/s)
    double maxLinearVelocity      = 2.0;                                                            ///< Maximum forward speed (m/s)
    Eigen::Matrix3d propagationUncertainty = Eigen::Matrix3d::Identity();                           ///< Uncertainty of configuration propagation in Kalman filter

    DifferentialDriveParameters() = default;
};

/**
 * @brief A struct that fully defines the state of a differential drive robot.
 */
struct DifferentialDriveState
{
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();                                       ///< Uncertainty of the pose
    Eigen::Vector2d velocity = {0.0, 0.0};                                                          ///< Linear & angular velocity, respectively
    RobotLibrary::Model::Pose2D pose;                                                               ///< Position & orientation relative to a reference frame.
};


/**
 * @brief A struct that defines an ellipsoid in 2D or 3D space.
 * 
 * The ellipsoid is represented as a quadratic form:
 * (x - centerPoint)^T * shapeMatrix.inverse() * (x - centerPoint) <= 1
 * 
 * @tparam Dim The dimension of the space (e.g., 2 or 3)
 * 
 * @note shapeMatrix must be symmetric and positive-definite.
 */
template <int Dim>
struct Ellipsoid
{
    Eigen::Matrix<double, Dim, Dim> shapeMatrix;                                                    ///< Positive-definite matrix that determines shape
    Eigen::Vector<double, Dim> centerPoint;                                                         ///< Location of center
};

} } // namespace

#endif
