/**
 * @file    DifferentialDrivePredictive.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Source files for the MPC class.
 *
 * @details This class solves the predictive control problem for differential drive robot over a
 *          finite number of steps. It uses quadratic functions for both the final step, and all
 *          intermediate steps. The weighting on the final & intermediate pose errors are used as
 *          constructor arguments, whereas the weighting on intermediate control values is based on
 *          the robot's mass & inertia in the RobotLibary::Model::DifferentialDrive class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see http://github.com/Woolfrey/software_simple_qp for info on the QP solver.
 */
 
#include <Control/DifferentialDrivePredictive.h>

namespace RobotLibrary { namespace Control {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDrivePredictive::DifferentialDrivePredictive(RobotLibrary::Model::DifferentialDriveParameters &modelParameters,
                                                         RobotLibrary::Control::DifferentialDrivePredictiveParameters &controlParameters,
                                                         SolverOptions<double> &solverOptions)
: DifferentialDriveBase(controlParameters.controlFrequency,
                        controlParameters.minimumSafeDistance,
                        modelParameters),
  QPSolver<double>(solverOptions),
 _numberOfRecursions(controlParameters.numberOfRecursions),
 _predictionSteps(controlParameters.predictionSteps),
 _threshold(controlParameters.maximumControlStepNorm)
{
    // Ensure weighting matrices are positive definite:
    std::string message;
    if (not RobotLibrary::Math::is_positive_definite(controlParameters.poseErrorWeight, message))
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE MPC] Constructor: "
                                    "Initial pose error weight matrix is not positive definite: " + message);
    }
    
    // Set size of vectors
    _predictedStates.resize(_predictionSteps + 1);                                                  // 1 for current state + N for predicted states
    _poseErrorWeight.resize(_predictionSteps);
    _controlWeight.resize(_predictionSteps);

    // Pose Error Weights (Normalized Exponential)
    double exponent = controlParameters.exponent;

    // Precompute denominator for normalization
    double denominator = 0.0;
    for (int j = 0; j < _predictionSteps; ++j)
    {
        denominator += std::exp(exponent * j);
    }

    // Assign normalized exponential pose weights
    for (int j = 0; j < _predictionSteps; ++j)
    {
        double scalar = std::exp(exponent * j) / denominator;
        _poseErrorWeight[j] = scalar * controlParameters.poseErrorWeight;
    }

    // Control Weights (Exponentially Scaled Inertia Matrix)
    Eigen::Matrix2d inertiaMatrix;
    inertiaMatrix << _mass,  0.0,
                      0.0, _inertia;

    std::vector<double> expWeights(_predictionSteps);

    // Compute exponential profile
    for (int i = 0; i < _predictionSteps; ++i)
    {
        expWeights[i] = std::exp(exponent * i);
    }

    // Anchor at R_0 = M or R_{N-1} = M depending on direction of growth
    double scale = (exponent >= 0.0)
                 ? 1.0 / expWeights.back()   // Ensure R_{N-1} = M
                 : 1.0 / expWeights.front(); // Ensure R_0 = M

    // Assign scaled control effort weights
    for (int i = 0; i < _predictionSteps; ++i)
    {
        double weight = expWeights[i] * scale;
        _controlWeight[i] = weight * inertiaMatrix;
    }
    
    // Set up the constraint matrices in advance to save time:

    _controlConstraintMatrix << -1.0,  0.0,
                                 0.0, -1.0,
                                 1.0,  0.0,
                                 0.0,  1.0;
                                 
    _obstacleConstraintMatrix.resize(0,2);                                                         
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Update the state                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
DifferentialDrivePredictive::update_state(const RobotLibrary::Model::Pose2D &pose,
                                          const Eigen::Vector2d &velocity,
                                          const Eigen::Matrix3d &covariance)
{
    RobotLibrary::Model::DifferentialDrive::update_state(pose, velocity, covariance);               // Update the underlying model
    
    // Transfer these from the base class so we can access them via index in the
    // backward / forward recursions
    _predictedStates[0].pose       = _pose;
    _predictedStates[0].velocity   = _velocity;
    _predictedStates[0].covariance = _covariance;
    
    // Shift the predicted states backward
    for (int i = 1; i < _predictionSteps - 1; ++i)
    {
        _predictedStates[i] = _predictedStates[i+1];
    }
} 

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Solve the trajectory tracking problem                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
DifferentialDrivePredictive::track_trajectory(const std::vector<RobotLibrary::Model::DifferentialDriveState>   &desiredStates,
                                              const std::vector<std::vector<RobotLibrary::Math::Ellipsoid<2>>> &obstacles)
{
    using namespace Eigen;
    using namespace RobotLibrary::Model;
    
    // Ensure inputs are sound
    if (desiredStates.size() != _predictionSteps + 1)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE PREDICTIVE] track_trajectory(): "
                                    "This controller requires N + 1 = " + std::to_string(_predictionSteps+1) + " "
                                    "desired states for the trajectory tracking, but received "
                                    + std::to_string(desiredStates.size()) + ".");
    }
    
    // Run the optimisations
    for (int i = 0; i < _numberOfRecursions; ++i)
    {
        double largestStepChange = 0.0;                                                             // Store largest step change in control for this recursion

        Vector3d costateVector;                                                                     // i.e. Lagrange multipliers
        
        // Backwards recursions
        for (int j = _predictionSteps; j >= 0; --j)
        {    

            if (j == _predictionSteps)
            {
                costateVector = _poseErrorWeight[j] * _predictedStates[j].pose.error(desiredStates[j].pose);
            }
            else
            {
                // Values used in this scope    
                double angle             = _predictedStates[j].pose.angle();
                Matrix3d K               = _poseErrorWeight[j];
                Matrix2d M               = _controlWeight[j];
                Pose2D currentPose       = _predictedStates[j].pose;
                Vector2d currentVelocity = _predictedStates[j].velocity;
                Vector2d desiredVelocity = desiredStates[j].velocity;
                Vector3d nextPoseError   = _predictedStates[j+1].pose.error(desiredStates[j].pose);
                Vector3d errorCorrection = K * nextPoseError + costateVector;

                // Partial derivative of state propagation w.r.t. configuration                                                    
                Eigen::Matrix<double,3,3> dfdx = configuration_jacobian(currentPose, currentVelocity, _controlFrequency);
                    
                // Partial derivative of state propagation w.r.t. control.
                Eigen::Matrix<double,3,2> dfdu = control_jacobian(currentPose, _controlFrequency);
                dfdu(2,1) = 1.0;

                // Partial derivative of state propagation w.r.t. configuration   
                Eigen::Vector<double,2> dLdu = - M * (desiredVelocity - currentVelocity)
                                               - dfdu.transpose() * errorCorrection;
                                                                      
                // Second derivative of Lagrangian w.r.t. control (i.e. Hessian)
                Eigen::Matrix<double,2,2> d2Ldu2 = M + dfdu.transpose() * K * dfdu;

                // Mixed derivatives of Lagrangian 
                Eigen::Matrix<double,2,3> d2Ldudx = dfdu.transpose() * K * dfdx;
                d2Ldudx(0,2) += (errorCorrection[0] * sin(angle) - errorCorrection[0] * cos(angle)) / _controlFrequency;
                                                                           
                // Set up the constraint for the control input du
                RobotLibrary::Model::Limits linear, angular;
                
                compute_control_limits(linear, angular, currentVelocity);                           // Limits are computed w.r.t. current velocity
                
                _controlConstraintVector << currentVelocity[0] - linear.lower,                      // -dv <= v - v_min
                                            currentVelocity[1] - angular.lower,                     // -dw <= w - w_min
                                            linear.upper  - currentVelocity[0],                     //  dv <= v_max - v
                                            angular.upper - currentVelocity[1];                     //  dw <= w_max - ws
                
                // Combine the constraints
                unsigned int numRows = _controlConstraintVector.size() + _obstacleConstraintVector.size();
                _constraintMatrix.resize(numRows, 2);
                _constraintMatrix.block(0,0,4,2)         = _controlConstraintMatrix;
                _constraintMatrix.block(4,0,numRows-4,2) = _obstacleConstraintMatrix;
                
                _constraintVector.resize(numRows);
                _constraintVector.segment(0,4)         = _controlConstraintVector;
                _constraintVector.segment(4,numRows-4) = _obstacleConstraintVector;
                
                // Solve the control
                Eigen::Vector3d dx = currentPose.error(desiredStates[j].pose);
                Eigen::Vector2d du = {0.0, 0.0};                                                    // We want to solve for this                                        
                try
                {
                    du = QPSolver<double>::solve(d2Ldu2,
                                                 dLdu + d2Ldudx * dx,
                                                 _constraintMatrix,
                                                 _constraintVector,
                                                 du);
                }
                catch (const std::exception &exception)
                {
                    throw std::runtime_error(std::string(exception.what()) + " "
                                             "Failed on recursion no. " + std::to_string(i) + " "
                                             "at step no. " + std::to_string(j) + ".");
                }
                
                double norm = du.norm();
                
                if (norm > largestStepChange) largestStepChange = norm;
                
                _predictedStates[j].velocity += du;                                                 // Increment control input

                costateVector = dfdx.transpose() * errorCorrection;
            }
        }
        
        // Forward recursions
        for (int j = 0; j < _predictionSteps; ++j)
        {       
            _predictedStates[j+1].pose =
            RobotLibrary::Model::DifferentialDrive::predicted_pose(_predictedStates[j].pose,
                                                                   _predictedStates[j].velocity,
                                                                   _controlFrequency);
                                              
            _predictedStates[j+1].covariance =
            RobotLibrary::Model::DifferentialDrive::predicted_covariance(_predictedStates[j].pose,
                                                                         _predictedStates[j].velocity,
                                                                         _predictedStates[j].covariance,
                                                                         _controlFrequency);
        }
        
        if (largestStepChange < _threshold) break;
    }
    
    return _predictedStates[0].velocity;                                                            // Only return the 1sts
}

} } // namespace
