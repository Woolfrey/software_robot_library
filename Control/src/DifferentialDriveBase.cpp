/**
 * @file    DifferentialDriveBase.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A base class to standardise all control classes of differential drive robots.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Control/DifferentialDriveBase.h>

namespace RobotLibrary { namespace Control {
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
DifferentialDriveBase::DifferentialDriveBase(const double &controlFrequency,
                                             const double &minimumSafeDistance,
                                             const RobotLibrary::Model::DifferentialDriveParameters &modelParameters)
: DifferentialDrive(modelParameters),
  _controlFrequency(controlFrequency),
  _minimumSafeDistance(minimumSafeDistance)
{
    if (_controlFrequency <= 0.0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE BASE] Constructor: "
                                    "Control frequency must be positive ("
                                    + std::to_string(_controlFrequency) + " <= 0.0).");
    }
    else if (_minimumSafeDistance <= 0.0)
    {
        throw std::invalid_argument("[ERROR] [DIFFERENTIAL DRIVE BASE] Constructor: "
                                    "Minimum safe distance must be positive ("
                                    + std::to_string(_minimumSafeDistance) + " <= 0.0)");
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Calculate instantaneous limits on linear & angular velocity              //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
DifferentialDriveBase::compute_control_limits(RobotLibrary::Model::Limits &linear,
                                              RobotLibrary::Model::Limits &angular,
                                              const Eigen::Vector2d &currentVelocity)
{
    linear.upper = std::min(_maxLinearVelocity,
                            currentVelocity[0] + _maxLinearAcceleration / _controlFrequency);
    
    linear.lower = std::max(-_maxLinearVelocity,
                             currentVelocity[0] - _maxLinearAcceleration / _controlFrequency);
                             
    angular.upper = std::min(_maxAngularVelocity,
                             currentVelocity[1] + _maxAngularAcceleration / _controlFrequency);
    
    angular.lower = std::max(-_maxAngularVelocity,
                              currentVelocity[1] - _maxAngularAcceleration / _controlFrequency);
                              
    if (linear.lower >= linear.upper)
    {
        throw std::logic_error("[ERROR] [DIFFERENTIAL DRIVE] compute_limits(): "
                               "Lower bound for linear velocity is greater than upper bound ("
                               + std::to_string(linear.lower) + " > " + std::to_string(linear.upper) +
                               "). How did that happen???");
    }
    else if (angular.lower >= angular.upper)
    {
        throw std::logic_error("[ERROR] [DIFFERENTIAL DRIVE] compute_limits(): "
                               "Lower bound for angular velocity is greater than upper bound ("
                               + std::to_string(angular.lower) + " > " + std::to_string(angular.upper) +
                               "). How did that happen???");
    }
}

} } // namespace

