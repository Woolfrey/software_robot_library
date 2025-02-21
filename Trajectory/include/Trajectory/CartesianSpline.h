/**
 * @file    CartesianSpline.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class that defines trajectories for position & orientation in 3D space.
 * 
 * @details This class generates trajectories for position & orientation by using a spline to
 *          interpolate over a series of poses.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef CARTESIANSPLINE_H_
#define CARTESIANSPLINE_H_

#include "RobotLibrary/Model/Pose.h"
#include "SplineTrajectory.h"

namespace RobotLibrary { namespace Trajectory {

/**
 * @brief A data structure for returning state information from functions.
 */
struct CartesianState
{
     RobotLibrary::Model::Pose pose;                                                                ///< Does this really need an explanation?
     Eigen::Vector<double,6>   twist;                                                               ///< Linear and angular velocity
     Eigen::Vector<double,6>   acceleration;                                                        ///<  Linear and angular acceleration
};                                                                                                  // Semicolon needed after declaration

/**
 * A class that defines splines in 3D space (i.e. SE(3))
 */
class CartesianSpline
{
    public:
        
        /**
         * @brief Empty constructor.
         */
        CartesianSpline() {}
        
        /**
         * @brief Constructor for cubic splines.
         * @poses An array of poses (position & orientation) to pass through.
         * @times The time at which to pass through each pose.
         * @startTwist The initial linear & angular velocity.
         */
        CartesianSpline(const std::vector<RobotLibrary::Model::Pose> &poses,
                        const std::vector<double> &times,
                        const Eigen::Vector<double,6> &startTwist);
                        
        /**
         * @brief Constructor for cubic splines, but only 2 poses are specified.
         * @param startPose The initial position & orientation.
         * @param endPose The final position & orientation.
         * @param startTwist the initial linear & angular velocity.
         * @param endTwist The final linear & angular velocity.
         */
        CartesianSpline(const RobotLibrary::Model::Pose &startPose,
                        const RobotLibrary::Model::Pose &endPose,
                        const Eigen::Vector<double,6> &startTwist,
                        const double &startTime,
                        const double &endTime)
        :
        CartesianSpline(std::vector<RobotLibrary::Model::Pose>{startPose, endPose},
                        std::vector<double>{startTime, endTime},
                        startTwist) {}
        /**
         * @brief Get the state for the given time.
         */               
        CartesianState
        query_state(const double &time);
        
        /**
         * @brief As it says.
         */
        double
        end_time() const { return _spline.end_time(); }
        
    private:
        
        SplineTrajectory _spline;                                                                   ///< Underlying trajectory over real numbers
};

} }

#endif
