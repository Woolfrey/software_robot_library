/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Contains custom structs used in Control classes.
 *
 * @copyright (c) 2025 Jon Woolfrey
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 *            Contact: jonathan.woolfrey@gmail.com
 *
 * @see https://github.com/Woolfrey/software_robot_library
 * @see https://github.com/Woolfrey/software_simple_qp
 */

#ifndef CONTROL_DATA_STRUCTS_H
#define CONTROL_DATA_STRUCTS_H

#include <Math/QPSolver.h>

#include <Eigen/Core>                                                                               // Eigen::Matrix

namespace RobotLibrary { namespace Control {
/**
 * @brief A data structure for passing control parameters to the SerialLinkBase class in a single argument.
 */
struct SerialLinkParameters
{
    SerialLinkParameters() = default;                                                               ///< This enables default options
    
    double jointPositionGain    = 100.0;                                                            ///< Scales the position error feedback  
    double jointVelocityGain    = 20.0;                                                             ///< Scales the velocity error feedback
    double maxJointAcceleration = 5.0;                                                              ///< Limits joint acceleration
    double minManipulability    = 1e-04;                                                            ///< Threshold for singularity avoidance
    
    unsigned int controlFrequency = 500;                                                            ///< Rate at which control loop operates.    
    
    Eigen::Matrix<double,6,6> cartesianPoseGain = (Eigen::MatrixXd(6,6) << 10.0,  0.0,   0.0,  0.0,  0.0,  0.0,
                                                                            0.0, 10.0,   0.0,  0.0,  0.0,  0.0,
                                                                            0.0,  0.0,  10.0,  0.0,  0.0,  0.0, 
                                                                            0.0,  0.0,   0.0,  5.0,  0.0,  0.0,
                                                                            0.0,  0.0,   0.0,  0.0,  5.0,  0.0,
                                                                            0.0,  0.0,   0.0,  0.0,  0.0,  5.0).finished(); ///< Scales pose error feedback
                                                                             
    Eigen::Matrix<double,6,6> cartesianVelocityGain = (Eigen::MatrixXd(6,6) << 20.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                                                                                0.0, 20.0,  0.0, 0.0, 0.0, 0.0,
                                                                                0.0,  0.0, 20.0, 0.0, 0.0, 0.0, 
                                                                                0.0,  0.0,  0.0, 2.0, 0.0, 0.0,
                                                                                0.0,  0.0,  0.0, 0.0, 2.0, 0.0,
                                                                                0.0,  0.0,  0.0, 0.0, 0.0, 2.0).finished(); ///< Scales twist error feedback                                                                                                                        
                                                                          
    SolverOptions<double> qpsolver = SolverOptions<double>();                                       ///< Parameters for the underlying QP solver
};

/**
 * @brief A data structure for parameters in the feedback control class.
 */
struct DifferentialDriveFeedbackParameters
{
    DifferentialDriveFeedbackParameters() = default;
    
    double controlFrequency    = 100.0;                                                             ///< Rate at which control is computed
    double minimumSafeDistance =   1.0;                                                             ///< Used in collision avoidance
    double orientationGain     =  10.0;                                                             ///< Feedback gain on orientation error   
    double xPositionGain       =   5.0;                                                             ///< Feedback gain on x position error
    double yPositionGain       =  25.0;                                                             ///< Feedback gain on y position error
};

/**
 * @brief A data structure containing parameters for model predictive control.
 */
struct DifferentialDrivePredictiveParameters
{
    DifferentialDrivePredictiveParameters() = default;

    double controlFrequency         = 100.0;                                                        ///< Rate at which control is calculated / implemented
    double exponent                 = -0.1;                                                         ///< Scales the pose error weight across the horizon
    double maximumControlStepNorm   = 1e-10;                                                        ///< Threshold for terminating optimisation
    double minimumSafeDistance      = 1.0;                                                          ///< Used in collision avoidance
    unsigned int numberOfRecursions = 2;                                                            ///< Number of forward & backward passes to optimise control
    unsigned int predictionSteps    = 10;                                                           ///< Length of prediction horizon
    
    Eigen::Matrix3d poseErrorWeight
    = (Eigen::MatrixXd(3,3) << 200.0,   0.00,  0.00,
                                 0.0, 200.00, -0.09, 
                                 0.0,  -0.09,  0.10).finished();
};

} } // namespace

#endif
