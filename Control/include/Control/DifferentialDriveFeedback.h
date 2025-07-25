/**
 * @file    DifferentialDriveFeedback.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A class for feedback control of a differential drive robot.
 *
 * @details This class inherits the RobotLibrary::Model::DifferentialDriveBase and provides a simple
 *          method for nonlinear feedback control.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef DIFFERENTIAL_DRIVE_FEEDBACK_H
#define DIFFERENTIAL_DRIVE_FEEDBACK_H

#include <Control/DataStructures.h>
#include <Control/DifferentialDriveBase.h>

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that performs nonlinear feedback control for trajectory tracking of a differential
 *        drive mobile robot.
 */
class DifferentialDriveFeedback : public RobotLibrary::Control::DifferentialDriveBase
{
    public:
    
        /** 
         * @brief Constructor.
         * @param xPositionGain Feedback gain on x-position error
         * @param yPositionGain Feedback gain on y-position error
         * @param orientationGain Feedback gain on orientation error
         * @param parameters Model parameters for the base class.
         */
        DifferentialDriveFeedback(const RobotLibrary::Model::DifferentialDriveParameters &modelParameters = RobotLibrary::Model::DifferentialDriveParameters(),
                                  const RobotLibrary::Control::DifferentialDriveFeedbackParameters &controlParameters = RobotLibrary::Control::DifferentialDriveFeedbackParameters());

        /**
         * @brief Solve the (nonlinear) feedback control problem to track a trajectory.
         * @param desiredPose The desired position & orientation defined by the trajectory.
         * @param desiredVelocity The desired linear & angular velocity defined by the trajectory.
         * @return The linear & angular velocity to track the trajectory.
         */
        Eigen::Vector2d
        track_trajectory(const RobotLibrary::Model::Pose2D &desiredPose,
                         const Eigen::Vector2d &desiredVelocity);

        private:
            
            double _orientationGain = 5.0;                                                          ///< Feedback gain on orientation error
            double _xPositionGain   = 1.0;                                                          ///< Feedback gain on x-translation error       
            double _yPositionGain   = 50.0;                                                         ///< Feedback gain on y-translation error           

};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
