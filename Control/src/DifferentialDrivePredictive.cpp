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
DifferentialDrivePredictive::DifferentialDrivePredictive(RobotLibrary::Control::DifferentialDrivePredictiveParameters &controlParameters,
                                                         RobotLibrary::Model::DifferentialDriveParameters &modelParameters,
                                                         SolverOptions<double> &solverOptions)
: DifferentialDrive(modelParameters),
  QPSolver<double>(solverOptions),
 _numberOfRecursions(controlParameters.numberOfRecursions),
 _predictionSteps(controlParameters.predictionSteps),
 _threshold(controlParameters.maxControlStepNorm),
 _finalPoseErrorWeight(controlParameters.finalPoseErrorWeight)
{
    // Ensure weighting matrices are positive definite:
    std::string message;
    if (not RobotLibrary::Math::is_positive_definite(controlParameters.initialPoseErrorWeight, message))
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE MPC] Constructor: "
                                    "Initial pose error weight matrix is not positive definite: " + message);
    }
    else if (not RobotLibrary::Math::is_positive_definite(controlParameters.finalPoseErrorWeight, message))
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE MPC] Constructor: "
                                    "Final pose error weight matrix is not positive definite: " + message);
    }
    
    // Set size of vectors
    _predictedStates.resize(_predictionSteps + 1);                                                  // 1 for current state + N for predicted states
    _intermediatePoseErrorWeight.resize(_predictionSteps);
    _intermediateControlWeight.resize(_predictionSteps);

    // Compute the intermediate weighting matrices using exponential growth / decay:
    
    double normK0 = controlParameters.initialPoseErrorWeight.norm();
    
    double normKN = controlParameters.finalPoseErrorWeight.norm();
    
    double decayRate = (1.0 / _predictionSteps) * std::log(normKN / normK0);
    
    Eigen::Matrix2d inertiaMatrix;
    inertiaMatrix << _mass,      0.0,
                       0.0, _inertia;
    
    for (int i = 0; i < _predictionSteps; ++i)
    {
        _intermediatePoseErrorWeight[i] = std::exp(decayRate * i) * controlParameters.initialPoseErrorWeight;
        
        double scale = (decayRate >= 0.0)
                     ? std::exp(decayRate * (i -_predictionSteps - 1))                              // Increasing, so final inertia == true inertia
                     : std::exp(decayRate * i);                                                     // Decreasing, so initial inertia == true inertia
        
        _intermediateControlWeight[i] = scale * inertiaMatrix;
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
DifferentialDrivePredictive::track_trajectory(const std::vector<RobotLibrary::Model::DifferentialDriveState> &desiredStates)
{
    // Ensure inputs are sound
    if (desiredStates.size() != _predictionSteps)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE PREDICTIVE] track_trajectory(): "
                                    "This controller has N = " + std::to_string(_predictionSteps) + " steps, "
                                    "but the input argument had " + std::to_string(desiredStates.size()) + " elements.");
    }
    
    // Run the optimisations
    for (int i = 0; i < _numberOfRecursions; ++i)
    {
        double largestStepChange = 0.0;                                                             // Store largest step change in control for this recursion
        
        Eigen::Vector3d costateVector;                                                              // i.e. Lagrange multipliers
        
        // Backwards recursions
        for (int j = _predictionSteps - 1; j >= 0; --j)
        {    
            Eigen::Vector3d poseError = _predictedStates[j+1].pose.error(desiredStates[j].pose);    // Error at step j+1 is affected by control input at step j

            if (j == _predictionSteps - 1)
            {
                costateVector = _finalPoseErrorWeight * poseError;                                  // Only need to evaluate costate vector at final steps   
            }
            else
            {
                double angle = _predictedStates[j].pose.angle();                                    // Used in multiple places
                
                // Partial derivative of state propagation w.r.t. configuration                                                    
                Eigen::Matrix<double,3,3> dfdx;
                
                dfdx << 1.0, 0.0, -_predictedStates[j].velocity[0] * sin(angle) / _controlFrequency,
                        0.0, 1.0,  _predictedStates[j].velocity[0] * cos(angle) / _controlFrequency,
                        0.0, 0.0,  _predictedStates[j].velocity[1] / _controlFrequency;
                        
                // Partial derivative of state propagation w.r.t. control.
                // NOTE: This last element should be 1 / _controlFrequency, but it works better as 1.0?
                Eigen::Matrix<double,3,2> dfdu;
                
                dfdu << cos(angle) / _controlFrequency,  0.0,
                        sin(angle) / _controlFrequency,  0.0,
                                                   0.0,  1.0; 

                // Partial derivative of state propagation w.r.t. configuration   
                // NOTE TO SELF: I THINK INDEXING ON desiredStates IS WRONG?                                               
                Eigen::Vector<double,2> dLdu = -_intermediateControlWeight[j] * (desiredStates[j].velocity - _predictedStates[j].velocity)
                                               - dfdu.transpose() * ( _intermediatePoseErrorWeight[j] * poseError + costateVector );
                                                                      
                // Second derivative of Lagrangian w.r.t. control (i.e. Hessian)
                Eigen::Matrix<double,2,2> d2Ldu2 = _intermediateControlWeight[j]
                                                 + dfdu.transpose() * _intermediatePoseErrorWeight[j] * dfdu;
                                                 
                d2Ldu2(0,0) += 1e-04;
                d2Ldu2(1,1) += 1e-04;
                
                // Mixed derivatives of Lagrangian 
                Eigen::Matrix<double,2,3> d2Ldudx = dfdu.transpose() * _intermediatePoseErrorWeight[j] * dfdx;
                d2Ldudx(0,2) += (costateVector[1] * cos(angle) - costateVector[0] * sin(angle)) / _controlFrequency;
                                                                           
                // Set up the constraint for the control input du
                RobotLibrary::Model::Limits linear, angular;
                
                compute_limits(linear, angular, _predictedStates[j].velocity);                      // Limits are computed w.r.t. current velocity
                
                _controlConstraintVector << _predictedStates[j].velocity[0] - linear.lower,         // -dv <= v - v_min
                                            _predictedStates[j].velocity[1] - angular.lower,        // -dw <= w - w_min
                                            linear.upper  - _predictedStates[j].velocity[0],        //  dv <= v_max - v
                                            angular.upper - _predictedStates[j].velocity[1];        //  dw <= w_max - ws
        
                // Set up the constraints for the obstacles
                /*
                for (int k = 0; k < obstacles.size(); ++k)
                {
                    _obstacleConstraintMatrix.row(k) = ...
                    _obstacleConstraintVector.row(k) = ...
                }
                */
                
                // Combine the constraints
                unsigned int numRows = _controlConstraintVector.size() + _obstacleConstraintVector.size();
                _constraintMatrix.resize(numRows, 2);
                _constraintMatrix.block(0,0,4,2)         = _controlConstraintMatrix;
                _constraintMatrix.block(4,0,numRows-4,2) = _obstacleConstraintMatrix;
                
                _constraintVector.resize(numRows);
                _constraintVector.segment(0,4)         = _controlConstraintVector;
                _constraintVector.segment(4,numRows-4) = _obstacleConstraintVector;
                
                // Solve the control
                Eigen::Vector3d dx = _predictedStates[j].pose.error(_predictedStates[j+1].pose);
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
                
                _predictedStates[j].velocity += du;                                                 // Increment control input

                double norm = du.norm();
                if (norm > largestStepChange) largestStepChange = norm;                             // Save the largest
                       
                costateVector = _intermediatePoseErrorWeight[j] * poseError + dfdx.transpose() * costateVector;
            }
        }
        
        // Forward recursions
        for (int j = 0; j < _predictionSteps; ++j)
        {       
            _predictedStates[j+1].pose = predicted_pose(_predictedStates[j].pose,
                                                        _predictedStates[j].velocity);
                                              
            _predictedStates[j+1].covariance = predicted_covariance(_predictedStates[j].pose,
                                                                    _predictedStates[j].velocity,
                                                                    _predictedStates[j].covariance);
        }
        
        if (largestStepChange < _threshold) break;                                                  // Break early if step change is tiny
    }
    
    return _predictedStates[0].velocity;                                                            // Only return the 1sts
}

} } // namespace
