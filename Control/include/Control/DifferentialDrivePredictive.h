/**
 * @file    DifferentialDrivePredictive.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A class that performs model predictive control for a differential drive robot.
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
 */
 
#ifndef DIFFERENTIAL_DRIVE_PREDICTIVE_H
#define DIFFERENTIAL_DRIVE_PREDICTIVE_H

#include <Control/DataStructures.h>
#include <Control/DifferentialDriveBase.h>

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that performs nonlinear feedback control for trajectory tracking of a differential
 *        drive mobile robot.
 */
class DifferentialDrivePredictive : public RobotLibrary::Control::DifferentialDriveBase,
                                    public QPSolver<double>
{
    public:
    
        /** 
         * @brief Constructor.
         * @param xPositionGain Feedback gain on x-position error
         * @param yPositionGain Feedback gain on y-position error
         * @param orientationGain Feedback gain on orientation error
         * @param parameters Model parameters for the base class.
         */
        DifferentialDrivePredictive(RobotLibrary::Model::DifferentialDriveParameters &modelParameters,
                                    RobotLibrary::Control::DifferentialDrivePredictiveParameters &controlParameters,
                                    SolverOptions<double> &solverOptions);
          
        /**
         * @brief Solve the model predictive control to track a trajectory.
         * @param desiredStates A series of desired poses & velocities sampled from a trajectory.
         * @param obstacles A vector of k obstacles across N+1 prediction steps.
         * @return The linear & angular velocity to track the trajectory.
         */
        Eigen::Vector2d
        track_trajectory(const std::vector<RobotLibrary::Model::DifferentialDriveState>    &desiredStates,
                         const std::vector<std::vector<RobotLibrary::Model::Ellipsoid<2>>> &obstacles);
        
        /**
         * @brief Get the predicted state at a specified step.
         * @param i The ith step of the prediction horizon, between 0 and N
         */
        RobotLibrary::Model::DifferentialDriveState
        predicted_state(const unsigned int &i);
        
        /**
         * @brief Retrieve the (current and) predicted states for the robot.
         */
        std::vector<RobotLibrary::Model::DifferentialDriveState>
        predicted_states() const { return _predictedStates; }
        
        /**
         * @brief Update the current state given new information.
         * @param pose The position & orientation of the robot relative to a global reference frame.
         * @param velocity The linear & angular speed.
         * @param covariance The uncertainty of the pose.
         */
        void
        update_state(const RobotLibrary::Model::Pose2D &pose,
                     const Eigen::Vector2d &velocity,
                     const Eigen::Matrix3d &covariance = Eigen::Matrix3d::Identity());
        
        private:
        
        double _threshold = 1e-10;                                                                  ///< Terminates algorithm early if this threshold is reached
        
        unsigned int _predictionSteps;                                                              ///< Number of steps in the prediction horizon
        
        unsigned int _numberOfRecursions;                                                           ///< Number of backward + forward passes
       
        Eigen::Matrix3d _finalPoseErrorWeight;                                                      ///< Weighting matrix on the final pose error
        
        Eigen::Matrix<double,4,2> _controlConstraintMatrix;                                         ///< Used in the QP solver to ensure control inputs are within bounds
        
        Eigen::Matrix<double,Eigen::Dynamic,2> _obstacleConstraintMatrix;                           ///< Used in the QP solver to avoid collision with obstacles
        
        Eigen::Matrix<double,Eigen::Dynamic,2> _constraintMatrix;                                   ///< Full constraint matrix passed to the QP solver
        
        Eigen::Vector<double,4> _controlConstraintVector;                                           ///< Use in the QP solver to ensure control input is within limits
        
        Eigen::VectorXd _obstacleConstraintVector;                                                  ///< Used in the QP solve to avoid collision with obstacles
        
        Eigen::VectorXd _constraintVector;                                                          ///< Full constraint vector passed to the QP solver
        
        std::vector<Eigen::Matrix3d> _poseErrorWeight;                                              ///< Weighting matrix on the intermediate pose error
        
        std::vector<Eigen::Matrix2d> _controlWeight;                                                ///< Weighting on the intermediate control
  
        std::vector<RobotLibrary::Model::DifferentialDriveState> _predictedStates;                  ///< Pose, velocity, and covariance over the prediction horizon
                
};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
