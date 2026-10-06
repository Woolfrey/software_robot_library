/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    August 2025
 * @version 2.0
 *
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
                                                                                  
    std::vector<double> jointPositionGains;
    
    std::vector<double> jointVelocityGains;
                                                                          
    SolverOptions<double> qpsolver = SolverOptions<double>();                                       ///< Parameters for the underlying QP solver
};

/**
 * @brief A data structure for parameters in the feedback control class.
 */
struct UnicycleFeedbackParameters
{
    UnicycleFeedbackParameters() = default;
    
    double controlBarrierScalar =  10.0;                                                            ///< Determines deceleration toward obstacles
    double controlFrequency     = 100.0;                                                            ///< Rate at which control is computed
    double lowpassFilterGain    =   0.9;                                                            ///< Used to smooth angular velocity signal
    double orientationGain      =  10.0;                                                            ///< Feedback gain on orientation error
    double translationGain      =   5.0;                                                            ///< Feedback gain on x position error

    SolverOptions<double> qpSolver = SolverOptions<double>();                                       ///< For underlying QP solver
};

/**
 * @brief A data structure containing parameters for model predictive control.
 */
struct UnicyclePredictiveParameters
{
    UnicyclePredictiveParameters() = default;

    double controlFrequency         = 100.0;                                                        ///< Rate at which control is calculated / implemented
    double exponent                 = 0.01;                                                         ///< Scales the pose error weight across the horizon
    double maximumControlStepNorm   = 1e-04;                                                        ///< Threshold for terminating optimisation
    double obstaclePotentialScalar  = 1e-03;                                                        ///< Scales the magnitude of the repulsion force
    unsigned int numberOfRecursions = 10;                                                           ///< Number of forward & backward passes to optimise control
    unsigned int predictionSteps    = 50;                                                           ///< Length of prediction horizon
    
    Eigen::Matrix3d poseErrorWeight = (Eigen::MatrixXd(3,3) << 200.0,   0.00,  0.00,
                                                                 0.0, 200.00, -0.09, 
                                                                 0.0,  -0.09,  0.10).finished();
};

/**
 * @brief A data structure containing parameters for model predictive control.
 */
struct DDPThreeCircleFootprintParameters
{
    DDPThreeCircleFootprintParameters() = default;

    double controlFrequency         = 100.0;                                                        ///< Rate at which control is calculated / implemented
    double exponent                 = 0.01;                                                         ///< Scales the pose error weight across the horizon
    double maximumControlStepNorm   = 1e-04;                                                        ///< Threshold for terminating optimisation
    double obstaclePotentialScalar  = 1e-03;                                                        ///< Scales the magnitude of the repulsion force
    unsigned int numberOfRecursions = 10;                                                           ///< Number of forward & backward passes to optimise control
    unsigned int predictionSteps    = 50;                                                           ///< Length of prediction horizon
    
    Eigen::Matrix3d poseErrorWeight
    = (Eigen::MatrixXd(3,3) << 200.0,   0.00,  0.00,
                                 0.0, 200.00, -0.09, 
                                 0.0,  -0.09,  0.10).finished();
};

/**
 * @brief A container for a control barrier function.
 * @note Standard form for optimsation is -\dot{b}^T * u \le \alpha(b)
 */
struct BarrierConstraints
{
    double scalar;                                                                                  ///< Right-hand-side of the CBF
    
    Eigen::Matrix<double,1,Eigen::Dynamic> rowVector;                                               ///< Left-hand-side of the CBF
};

} } // namespace

#endif
