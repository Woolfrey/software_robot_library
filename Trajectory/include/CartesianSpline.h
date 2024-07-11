/**
 * @file   CartesianSpline.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining splines over poses (i.e. SE(3)).
 */
 
#ifndef CARTESIANSPLINE_H_
#define CARTESIANSPLINE_H_

#include "Pose.h"
#include "SplineTrajectory.h"

/**
 * A data structure for returning state information from functions.
 */
struct CartesianState
{
     Pose  pose;                                                                                    ///< Does this really need an explanation?
     Eigen::Vector<double,6> twist;                                                                 ///< Linear and angular velocity
     Eigen::Vector<double,6> acceleration;                                                          ///<  Linear and angular acceleration
};                                                                                                  // Semicolon needed after declaration

/**
 * A class that defines splines in 3D space (i.e. SE(3))
 */
class CartesianSpline
{
    public:
        
        /**
         * Constructor for cubic splines.
         * @poses An array of poses (position & orientation) to pass through.
         * @times The time at which to pass through each pose.
         * @startTwist The initial linear & angular velocity.
         */
        CartesianSpline(const std::vector<Pose> &poses,
                        const std::vector<double> &times,
                        const Eigen::Vector<double,6> &startTwist);
                        
        /**
         * Constructor for cubic splines, but only 2 poses are specified.
         * @param startPose The initial position & orientation.
         * @param endPose The final position & orientation.
         * @param startTwist the initial linear & angular velocity.
         * @param endTwist The final linear & angular velocity.
         */
        CartesianSpline(const Pose &startPose,
                        const Pose &endPose,
                        const Eigen::Vector<double,6> &startTwist,
                        const double &startTime,
                        const double &endTime)
        :
        CartesianSpline(std::vector<Pose>{startPose, endPose},
                        std::vector<double>{startTime, endTime},
                        startTwist) {}
        /**
         * Get the state for the given time.
         */               
        CartesianState
        query_state(const double &time);
        
    private:
        
        SplineTrajectory _spline;                                                                   ///< Underlying trajectory over real numbers
};

#endif
