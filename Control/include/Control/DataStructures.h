/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   Contains custom structs used in Control classes.
 *
 * @copyright Copyright (c) 2025 Jon Woolfrey
 *
 * @license Open Source / Commercial Use License (OSCL)
 *
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
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
    // NOTE: These are for the control class itself:
    SerialLinkParameters() = default;                                                               ///< This enables default options
    
    double jointPositionGain    = 100.0;                                                            ///< Scales the position error feedback  
    double jointVelocityGain    = 20.0;                                                             ///< Scales the velocity error feedback
    double maxJointAcceleration = 5.0;                                                              ///< Limits joint acceleration
    double minManipulability    = 1e-04;                                                            ///< Threshold for singularity avoidance
    
    unsigned int controlFrequency = 100;                                                            ///< Rate at which control loop operates.    
    
    Eigen::Matrix<double,6,6> cartesianStiffness = (Eigen::MatrixXd(6,6) << 10.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                                                                             0.0, 10.0,  0.0, 0.0, 0.0, 0.0,
                                                                             0.0,  0.0, 10.0, 0.0, 0.0, 0.0, 
                                                                             0.0,  0.0,  0.0, 2.0, 0.0, 0.0,
                                                                             0.0,  0.0,  0.0, 0.0, 2.0, 0.0,
                                                                             0.0,  0.0,  0.0, 0.0, 0.0, 2.0).finished(); ///< Scales pose error feedback
                                                                             
    Eigen::Matrix<double,6,6> cartesianDamping = (Eigen::MatrixXd(6,6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
                                                                          0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
                                                                          0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
                                                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.2).finished(); ///< Scales twist error feedback                                                                                                                        
                                                                          
    SolverOptions<double> qpsolver = SolverOptions<double>();                                       ///< Parameters for the underlying QP solver
};

struct DifferentialDrivePredictiveParameters
{
    DifferentialDrivePredictiveParameters() = default;
  
    unsigned int numberOfRecursions = 2;
    
    unsigned int predictionSteps = 10;
    
    Eigen::Matrix3d initialPoseErrorWeight = Eigen::Matrix3d::Identity();
    
    Eigen::Matrix3d finalPoseErrorWeight = Eigen::Matrix3d::Identity();
};

} } // namespace

#endif
