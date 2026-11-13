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
 * @brief A data structure containing parameters for model predictive control.
 */
struct UnicyclePredictiveParameters
{
    UnicyclePredictiveParameters() = default;

    double controlFrequency         = 100.0;                                                        ///< Rate at which control is calculated / implemented
    double exponent                 = 0.01;                                                         ///< Scales the pose error weight across the horizon
    double maximumControlStepNorm   = 1e-04;                                                        ///< Threshold for terminating optimisation
    double obstaclePotentialScalar  = 1e-03;                                                        ///< Scales the magnitude of the repulsion force
    double potentialCutoffDistance  = 2.0;                                                          ///< Distance from obstacle after which the potential should not apply
    double solverFractionTime       = 0.8;                                                          ///< Fraction of of (1/frequency) [s] allowed before algorithm terminates
    unsigned int numberOfRecursions = 10;                                                           ///< Number of forward & backward passes to optimise control
    unsigned int predictionSteps    = 50;                                                           ///< Length of prediction horizon
    
    Eigen::Matrix3d poseErrorWeight = (Eigen::MatrixXd(3,3) << 200.0,   0.00,  0.00,
                                                                 0.0, 200.00, -0.09, 
                                                                 0.0,  -0.09,  0.10).finished();
};

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
         * @brief Data structure for successful completion of MPC optimisation.
         */
        struct Result
        {
            double solutionTime          = 0.0;                                                     ///< How long it took to solve the optimisation problem
            int recursions               = 0;                                                       ///< No. of backward & forward passes of the DDP algorithm
            Eigen::Vector2d controlInput = {0.0, 0.0};                                              ///< Optimal control input to implement
        };
        
        /**
         * @brief Thrown when MPC algorithm detects a collision.
         */
        struct CollisionException : std::runtime_error
        {
            CollisionException(unsigned int recursion,
                               unsigned int predictionStep,
                               unsigned int obstacleIndex,
                               const Eigen::Vector2d lastBestSolution,
                               const std::string &message)
            : std::runtime_error(message),
              recursion(recursion),
              predictionStep(predictionStep),
              lastBestSolution(lastBestSolution),
              obstacleIndex(obstacleIndex) {}
              
            unsigned int recursion;                                                                 ///< Which iteration of the solver it failed at [i]
            unsigned int predictionStep;                                                            ///< Which prediction step [j]
            unsigned int obstacleIndex;                                                             ///< Identifier for which obstacle [k]
            Eigen::Vector2d lastBestSolution = {0.0, 0.0};
        };
                                         
        /**
         * @brief Solve the model predictive control to track a trajectory.
         * @param desiredStates A series of desired poses & velocities sampled from a trajectory.
         * @param obstacles A vector of k obstacles across N+1 prediction steps.
         * @param predictionSteps Length of the prediction horizon
         * @param numberOfRecursions How many times to perform backward & forward pass of DDP algorithm
         * @return The optimal linear & angular velocity, and solver diagnostics.
         */
        [[nodiscard]] Result
        track_trajectory(const std::vector<RobotLibrary::Model::UnicycleState> &desiredStates,
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
        
        double _maximumSolutionTime;                                                                ///< Solver will terminate early if it takes longer than this
        
        double _obstaclePotentialScalar = 1e-01;                                                    ///< Determines magnitude of repulsive force

        double _potentialCutoffDistance = 2.0;                                                      ///< Obstacles are ignored beyond this distance

        double _threshold = 1e-06;                                                                  ///< Terminates algorithm early if this threshold is reached
        
        unsigned int _predictionSteps;                                                              ///< Number of steps in the prediction horizon
        
        unsigned int _numberOfRecursions;                                                           ///< Number of backward + forward passes

        std::vector<Eigen::Matrix3d> _poseErrorWeight;                                              ///< Weighting matrix on the intermediate pose error
  
        std::vector<RobotLibrary::Model::UnicycleState> _predictedStates;                           ///< Pose, velocity, and covariance over the prediction horizon

};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
