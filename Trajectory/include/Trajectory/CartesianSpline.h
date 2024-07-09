/**
 * @file   CartesianSpline.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining splines over poses (i.e. SE(3)).
 */
 
#ifndef CARTESIANSPLINE_H_
#define CARTESIANSPLINE_H_

#include <Model/Pose.h>
#include <SplineTrajectory.h>

// NOTE TO SELF: I NEED TO MOVE THIS SOMEWHERE ELSE
/**
 * A data structure for returning state information from functions.
 */
template <typename DataType>
struct CartesianState
{
     Pose<DataType>            pose;                                                                ///< Does this really need an explanation?
     Eigen::Vector<DataType,6> twist;                                                               ///< Linear and angular velocity
     Eigen::Vector<DataType,6> acceleration;                                                        ///<  Linear and angular acceleration
};                                                                                                  // Semicolon needed after declaration

template <class DataType>
class CartesianSpline
{
    public:
        
        /**
         * Constructor for cubic splines.
         * @poses An array of poses (position & orientation) to pass through.
         * @times The time at which to pass through each pose.
         * @startTwist The initial linear & angular velocity.
         */
        CartesianSpline(const std::vector<Pose<DataType>> &poses,
                        const std::vector<DataType> &times,
                        const Eigen::Vector<DataType,6> &startTwist);
                        
        /**
         * Constructor for cubic splines, but only 2 poses are specified.
         * @param startPose The initial position & orientation.
         * @param endPose The final position & orientation.
         * @param startTwist the initial linear & angular velocity.
         * @param endTwist The final linear & angular velocity.
         */
        CartesianSpline(const Pose<DataType> &startPose,
                        const Pose<DataType> &endPose,
                        const Eigen::Vector<DataType,6> &startTwist,
                        const DataType &startTime,
                        const DataType &endTime)
        :
        CartesianSpline(std::vector<Pose<DataType>>{startPose, endPose},
                        std::vector<DataType>{startTime, endTime},
                        startTwist) {}
        /**
         * Get the state for the given time.
         */               
        inline
        CartesianState<DataType>
        query_state(const DataType &time);
        
    private:
        
        SplineTrajectory<DataType> _spline;                                                         // Underlying trajectory over real numbers
};

#endif
