/**
 * @file    UnicyclePredictive.h
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
 *          the robot's mass & inertia in the RobotLibary::Model::Unicycle class.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef UNICYCLE_PREDICTIVE_H
#define UNICYCLE_PREDICTIVE_H

#include <Control/DataStructures.h>
#include <Control/UnicycleBase.h>
#include <Math/Ellipsoid.h>
#include <chrono>

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that performs nonlinear feedback control for trajectory tracking of a differential
 *        drive mobile robot.
 */
class UnicyclePredictive : public RobotLibrary::Control::UnicycleBase
{
    public:
    
        /** 
         * @brief Constructor.
         * @param xPositionGain Feedback gain on x-position error
         * @param yPositionGain Feedback gain on y-position error
         * @param orientationGain Feedback gain on orientation error
         * @param parameters Model parameters for the base class.
         */
        UnicyclePredictive(RobotLibrary::Model::UnicycleParameters &modelParameters,
                           RobotLibrary::Control::UnicyclePredictiveParameters &controlParameters);
          
        /**
         * @brief Solve the model predictive control to track a trajectory.
         * @param desiredStates A series of desired poses & velocities sampled from a trajectory.
         * @param obstacles A vector of k obstacles across N+1 prediction steps.
         * @return The linear & angular velocity to track the trajectory.
         */
        Eigen::Vector2d
        track_trajectory(const std::vector<RobotLibrary::Model::UnicycleState>  &desiredStates,
                         const std::vector<std::vector<RobotLibrary::Model::Obstacle2D>> &obstacles);
        
        /**
         * @brief Get the predicted state at a specified step.
         * @param i The ith step of the prediction horizon, between 0 and N
         */
        RobotLibrary::Model::UnicycleState
        predicted_state(const unsigned int &i);
        
        /**
         * @brief Retrieve the (current and) predicted states for the robot.
         */
        std::vector<RobotLibrary::Model::UnicycleState>
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
        
        double _obstaclePotentialScalar = 1e-01;                                                    ///< Determines magnitude of repulsive force

        double _potentialCutoffDistance = 2.0;

        double _threshold = 1e-06;                                                                  ///< Terminates algorithm early if this threshold is reached
        
        unsigned int _predictionSteps;                                                              ///< Number of steps in the prediction horizon
        
        unsigned int _numberOfRecursions;                                                           ///< Number of backward + forward passes

        std::vector<Eigen::Matrix3d> _poseErrorWeight;                                              ///< Weighting matrix on the intermediate pose error
  
        std::vector<RobotLibrary::Model::UnicycleState> _predictedStates;                           ///< Pose, velocity, and covariance over the prediction horizon

};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
