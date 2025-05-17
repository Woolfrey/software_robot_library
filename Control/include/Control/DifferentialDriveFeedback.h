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
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef DIFFERENTIAL_DRIVE_FEEDBACK_H
#define DIFFERENTIAL_DRIVE_FEEDBACK_H

#include <Model/DifferentialDrive.h>

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that performs nonlinear feedback control for trajectory tracking of a differential
 *        drive mobile robot.
 */
class DifferentialDriveFeedback : public RobotLibrary::Model::DifferentialDrive
{
    public:
    
        /** 
         * @brief Constructor.
         * @param xPositionGain Feedback gain on x-position error
         * @param yPositionGain Feedback gain on y-position error
         * @param orientationGain Feedback gain on orientation error
         * @param parameters Model parameters for the base class.
         */
        DifferentialDriveFeedback(const double &xPositionGain = 1.0,
                                  const double &yPositionGain = 100.0,
                                  const double &orientationGain = 5.0,
                                  const RobotLibrary::Model::DifferentialDriveParameters &parameters = RobotLibrary::Model::DifferentialDriveParameters());

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
            
            double _xPositionGain = 1.0;                                                            ///< Feedback gain on x-translation error
            
            double _yPositionGain = 100.0;                                                          ///< Feedback gain on y-translation error
            
            double _orientationGain = 5.0;                                                          ///< Feedback gain on orientation error

};                                                                                                  // Semicolon needed after class declaration

} } // Namespace                                                                                      

#endif
