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
  _orientationGain(controlParameters.orientationGain),
  _xPositionGain(controlParameters.xPositionGain),
  _yPositionGain(controlParameters.yPositionGain)
{
    // Check that the inputs are sound
    if (_xPositionGain   <= 0
    or  _yPositionGain   <= 0
    or  _orientationGain <= 0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE FEEDBACK] Constructor: "
                                    "Feedback control gains must be positive, but "
                                    "the x position gain was " + std::to_string(_xPositionGain) + ", "
                                    "the y position gain was " + std::to_string(_yPositionGain) + ", and "
                                    "the orientation gain was " + std::to_string(_orientationGain) + ".");
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
    
    // Kanayama, Y., Kimura, Y., Miyazaki, F., & Noguchi, T. (1990, May).
    // A stable tracking control method for an autonomous mobile robot.
    // In Proceedings., IEEE International Conference on Robotics and Automation (pp. 384-389). IEEE.
    
    // Pose error in the GLOBAL frame
    Vector3d e = _pose.error(desiredPose);
   
    // Position error in the LOCAL frame
    double epsilon_x =  e[0] * cos(_pose.angle()) + e[1] * sin(_pose.angle());
    double epsilon_y = -e[0] * sin(_pose.angle()) + e[1] * cos(_pose.angle());
    
    // Solve a QP problem of the form:
    // min_u 1/2 (u_d - u)^T M (u_d - u)
    //  subject to: B * u <= z
    // Hessian H == M, and f == - M * u_d
                           
Vector2d f = { -_mass    * (desiredVelocity[0] * cos(e[2]) + _xPositionGain * epsilon_x), 
               -_inertia * (desiredVelocity[1] + desiredVelocity[0] * (_yPositionGain * epsilon_y + _orientationGain * sin(e[2]))) };
               
    // Compute speed limits
    Model::Limits linear, angular;
    
    compute_control_limits(linear, angular, velocity());

    _controlConstraintVector <<  linear.upper,
                                angular.upper,
                                -linear.lower,
                               -angular.lower;
    
    // Compute obstacle constraints
    int n = obstacles.size();                                                                       // Makes referencing easier                             
    _obstacleConstraintMatrix.resize(n,2);
    _obstacleConstraintVector.resize(n);

    for (int i = 0; i < n; ++i)
    {

        Model::UnicycleState state;
        state.pose = _pose;
        state.velocity[0] = cos(_pose.angle()) * _twist[0] + sin(_pose.angle()) * _twist[1];
        state.velocity[1] = _twist[2];

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
        throw std::runtime_error("[ERROR] [DIFFERENTIAL DRIVE FEEDBACK] compute_barrier_constraints(): "
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
