/**
 * @file    CartesianSpline.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   A class that defines trajectories for position & orientation in 3D space.
 * 
 * @details This class generates trajectories for position & orientation by using a spline to
 *          interpolate over a series of poses.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef CARTESIAN_SPLINE_H_
#define CARTESIAN_SPLINE_H_

#include <Model/Pose.h>
#include <Trajectory/DataStructures.h>
#include <Trajectory/SplineTrajectory.h>

namespace RobotLibrary { namespace Trajectory {

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
        RobotLibrary::Trajectory::CartesianState
        query_state(const double &time);
        
        /**
         * @brief As it says.
         */
        double
        end_time() const { return _spline.end_time(); }
        
    private:
        
        RobotLibrary::Trajectory::SplineTrajectory _spline;                                         ///< Underlying trajectory over real numbers
};

} } // namespace

#endif
