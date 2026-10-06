/**
 * @file    UnicycleFeedback.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A class for feedback control of a differential drive robot.
 *
 * @details This class inherits the RobotLibrary::Model::UnicycleBase and provides a simple
 *          method for nonlinear feedback control.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef UNICYCLE_FEEDBACK_H
#define UNICYCLE_FEEDBACK_H

#include <Control/UnicycleBase.h>
#include <Math/QPSolver.h>

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that performs nonlinear feedback control for trajectory tracking of a differential
 *        drive mobile robot.
 */
class UnicycleFeedback : public RobotLibrary::Control::UnicycleBase,
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
        UnicycleFeedback(const RobotLibrary::Model::UnicycleParameters &modelParameters = RobotLibrary::Model::UnicycleParameters(),
                         const RobotLibrary::Control::UnicycleFeedbackParameters &controlParameters = RobotLibrary::Control::UnicycleFeedbackParameters());

        /**
         * @brief Solve the (nonlinear) feedback control problem to track a trajectory.
         * @param desiredPose The desired position & orientation defined by the trajectory.
         * @param desiredVelocity The desired linear & angular velocity defined by the trajectory.
         * @return The linear & angular velocity to track the trajectory.
         */
        Eigen::Vector2d
        track_trajectory(const RobotLibrary::Model::Pose2D &desiredPose,
                         const Eigen::Vector2d &desiredVelocity,
                         const std::vector<RobotLibrary::Model::Obstacle2D> &obstacles);

        private:
            
            double _controlBarrierScalar = 10.0;                                                    ///< Dictates decelaration toward obstacles
            double _lowpassFilterGain    =  0.9;                                                    ///< Used to smooth angular velocity signal
            double _orientationGain      =  1.0;                                                    ///< Feedback gain on orientation error  
            double _translationGain      = 10.0;                                                    ///< Feedback gain on y-translation error           

            /**
             * @brief Compute the control barrier constraints for an obstacle.
             */
            RobotLibrary::Control::BarrierConstraints
            compute_barrier_constraints(const RobotLibrary::Model::UnicycleState &state,
                                        const RobotLibrary::Model::Obstacle2D &obstacle);
};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
