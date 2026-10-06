/**
 * @file    UnicycleFeedback.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Source files for the UnicycleFeedback control class.
 *
 * @details This class provides method for implementing nonlinear feedback control of a differential
 *          drive robot.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Control/UnicycleFeedback.h>

namespace RobotLibrary { namespace Control {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
UnicycleFeedback::UnicycleFeedback(const RobotLibrary::Model::UnicycleParameters &modelParameters,
                                   const RobotLibrary::Control::UnicycleFeedbackParameters &controlParameters)
: UnicycleBase(controlParameters.controlFrequency,
               modelParameters),
  QPSolver<double>(controlParameters.qpSolver),
  _controlBarrierScalar(controlParameters.controlBarrierScalar),
  _lowpassFilterGain(controlParameters.lowpassFilterGain),
  _orientationGain(controlParameters.orientationGain),
  _translationGain(controlParameters.translationGain)
{
    // Check that the inputs are sound
    if (_translationGain <= 0
    or  _orientationGain <= 0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE FEEDBACK] Constructor: "
                                    "Feedback control gains must be positive, but "
                                    "the translation gain was " + std::to_string(_translationGain) + ", and "
                                    "the orientation gain was " + std::to_string(_orientationGain) + ".");
    }
    
    if (_controlBarrierScalar < 0.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE FEEDBACK] Constructor: "
                                    "Control barrier function gain was " + std::to_string(_controlBarrierScalar) + " "
                                    "but must be positive.");
    }
    
    if (_lowpassFilterGain < 0.0
    or  _lowpassFilterGain >= 1.0)
    {
        throw std::invalid_argument("[ERROR] [UNICYCLE FEEDBACK] Constructor: "
                                    "Lowpass filter gain was " + std::to_string(_lowpassFilterGain) + " "
                                    "but must be greater than or equal to 0.0, or less than 1.0");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Solve the control to track a desired trajectory                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
UnicycleFeedback::track_trajectory(const RobotLibrary::Model::Pose2D &desiredPose,
                                   const Eigen::Vector2d &desiredVelocity,
                                   const std::vector<RobotLibrary::Model::Obstacle2D> &obstacles)
{
    using namespace Eigen;
    using namespace RobotLibrary;
    
    Vector2d desiredLinearVelocity = {desiredVelocity[0] * cos(desiredPose.angle()),
                                      desiredVelocity[0] * sin(desiredPose.angle())};               // Unpack velocity as a vector
       
    Vector2d headingVector = {cos(_pose.angle()),
                              sin(_pose.angle())};                                              
                              
    Vector2d translationError = desiredPose.translation() - _pose.translation();
    
    double linearVelocity = headingVector.transpose() * (desiredLinearVelocity + _translationGain * translationError);

    double squaredErrorNorm = translationError.squaredNorm();
    
    double threshold = 4e-9;                                                                        // Check for singularity

    double orientationError = (squaredErrorNorm > threshold)                                        // If not singular...
                            ? atan2(translationError[1], translationError[0]) - _pose.angle()       // Compute angle between current position and desired position
                            : desiredPose.angle() - _pose.angle();                                  // Otherwise use desired angle from trajectory
                            
    orientationError = atan2(sin(orientationError), cos(orientationError));                         // wrap to (-pi, pi] 
    
    Vector2d translationErrorDerivative = (desiredLinearVelocity - linearVelocity * headingVector);
    
    double crossProduct = translationError[0] * translationErrorDerivative[1]
                        - translationError[1] * translationErrorDerivative[0];                      // Need to compute 2D cross-product manually
    
    double angularVelocity = (squaredErrorNorm > threshold)                                         // If not singular...
                           ? crossProduct / squaredErrorNorm                                        // Change in angle error from change in velocity
                           : desiredVelocity[1];                                                    // Otherwise use reference value from trajectory

    double alpha = 0.9;
    
    angularVelocity = alpha * _twist[2] + (1.0 - alpha) * angularVelocity;                          // Low-pass filter to smooth out angular velocity command
                       
    angularVelocity += _orientationGain * orientationError;                                         // Feedforward + feedback control


    // Solve a QP problem of the form:
    // min_u 1/2 (u_d - u)^T M (u_d - u)
    //  subject to: B * u <= z
    // Hessian H == M, and f == - M * u_d
    
    Vector2d f = { -_mass    * linearVelocity,
                   -_inertia * angularVelocity };
                   
    // Compute speed limits
    Model::Limits linear, angular;
    
    compute_control_limits(linear, angular, velocity());

    _controlConstraintVector <<  linear.upper,
                                angular.upper,
                                -linear.lower,
                               -angular.lower;


    // Need this for computing CBF
    Model::UnicycleState state;
    state.pose = _pose;
    state.velocity[0] = cos(_pose.angle()) * _twist[0] + sin(_pose.angle()) * _twist[1];
    state.velocity[1] = _twist[2];
        
    // Compute obstacle constraints
    int n = obstacles.size();                                                                       // Makes referencing easier                             
    _obstacleConstraintMatrix.resize(n,2);
    _obstacleConstraintVector.resize(n);
    
    for (int i = 0; i < n; ++i)
    {
        const auto &[scalar, rowVector] = compute_barrier_constraints(state , obstacles[i]);

        _obstacleConstraintMatrix.row(i) = rowVector;
        
        _obstacleConstraintVector(i) = scalar;
    }
      
    _constraintMatrix.resize(4+n,2);
    _constraintMatrix.block(0,0,4,2) = _controlConstraintMatrix;
    _constraintMatrix.block(4,0,n,2) = _obstacleConstraintMatrix;
    
    _constraintVector.resize(4+n);
    _constraintVector.head(4) = _controlConstraintVector;
    _constraintVector.tail(n) = _obstacleConstraintVector; 
 
    return QPSolver<double>::solve(_inertiaMatrix,f,_constraintMatrix, _constraintVector, velocity());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Compute the constraint barrier vector and scalar for an obstacle               //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Control::BarrierConstraints
UnicycleFeedback::compute_barrier_constraints(const RobotLibrary::Model::UnicycleState &state,
                                              const RobotLibrary::Model::Obstacle2D &obstacle)
{
    using namespace Eigen;                                                                          // For brevity

    Vector2d robotPosition = state.pose.translation();

    RobotLibrary::Math::ShapeQuery query = obstacle.query_point(robotPosition);
    
    double distance = query.signedDistance - _minimumSafeDistance;                                  // Subtract for added safety

    if (distance < 0.0)
    {
        throw std::runtime_error("[ERROR] [UNICYCLE FEEDBACK] compute_barrier_constraints(): "
                                 "Collision with '" + obstacle.name() + "' obstacle detected.");
    }

    double angle = state.pose.angle();
    
    Vector2d heading(cos(angle), sin(angle));                                                       // A unit vector
    
    double projection = heading.dot(query.unitVector);                                              // NOTE: This is negative if the robot is facing the obstacle
    
    // Set up row vector for constraint matrix
    Eigen::Vector2d rowVector;
    rowVector[0] = projection;
    rowVector[1] = 0.0;
    
    double gamma = 10.0; // NEED TO RE-EXAMINE THIS?
    
    return RobotLibrary::Control::BarrierConstraints{ gamma * distance,
                                                     -rowVector};
}

} } // Namespace                                                                                      
