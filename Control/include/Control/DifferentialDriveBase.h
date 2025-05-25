/**
 * @file    DifferentialDriveBase.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A base class to standardise all control classes of differential drive robots.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef DIFFERENTIAL_DRIVE_BASE_H
#define DIFFERENTIAL_DRIVE_BASE_H

#include <Model/DifferentialDrive.h>

namespace RobotLibrary { namespace Control {

class DifferentialDriveBase : public RobotLibrary::Model::DifferentialDrive
{
    public:
    
        DifferentialDriveBase(const double &controlFrequency,
                              const double &minimumSafeDistance,
                              const RobotLibrary::Model::DifferentialDriveParameters &modelParameters);
                              
        /**
         * @brief Get the predicted pose at the next time step given the current state.
         */
        RobotLibrary::Model::Pose2D
        predicted_pose()
        { 
            return RobotLibrary::Model::DifferentialDrive::predicted_pose(_pose, _velocity, _controlFrequency);
        }
    
    protected:

        double _controlFrequency;
        double _minimumSafeDistance;

        /**
         * @brief Compute the instantaneous limits on the linear & angular velocity.
         * @param Storage for the linear velocity limits.
         * @param Storage for the angular velocity limits.
         */
        void
        compute_control_limits(RobotLibrary::Model::Limits &linear,
                               RobotLibrary::Model::Limits &angular,
                               const Eigen::Vector2d &currentVelocity);
};
 
} } // namespace

#endif
