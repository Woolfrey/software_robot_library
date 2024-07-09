/**
 * @file   CartesianSpline.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining splines over poses (i.e. SE(3)).
 */
 

#ifndef CARTESIANSPLINE_H_
#define CARTESIANSPLINE_H_

#include "Model/Pose.h"
#include "SplineTrajectory.h"


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


  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                              Constructor for cubic splines                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
CartesianSpline<DataType>::CartesianSpline(const std::vector<Pose<DataType>> &poses,
                                           const std::vector<DataType> &times,
                                           const Eigen::Vector<DataType,6> &startTwist)
{
    if(poses.size() < 2)
    {
        throw std::invalid_argument("[ERROR] [CARTESIAN SPLINE] Constructor: "
                                    "A minimum number of 2 poses is required to create a trajectory.");
    }
    else if(poses.size() != times.size())
    {
        throw std::invalid_argument("[ERROR] [CARTESIAN SPLINE] Constructor: "
                                    "Dimensions of arguments do not match. "
                                    "There were " + std::to_string(poses.size()) + " poses, but "
                                    + std::to_string(times.size()) + " times.");
    }
    
    // Convert the orientation from a quaternion to a vector
    std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> positions(poses.size());                    // NOTE: The SplineTrajectory class expects a Dynamic size vector
    for(int i = 0; i < positions.size(); i++)
    {   
        positions[i].resize(6);
        
        positions[i].head(3) = poses[i].translation();                                              // First part is just the translation
        
        DataType angle = 2*acos(poses[i].quaternion().w());                                         // w = cos(0.5*angle)
        
        if(abs(angle) < 1e-04) positions[i].tail(3).setZero();
        else                   positions[i].tail(3) = angle*poses[i].quaternion().vec().normalized();

        positions[i].tail(3) = angle*poses[i].quaternion().vec().normalized();
    }
    
    this->_spline = SplineTrajectory<DataType>(positions,times,startTwist);                         // Generates a cubic spline. Velocities automatically calculated.  
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
CartesianState<DataType>
CartesianSpline<DataType>::query_state(const DataType &time)
{
    State<DataType> state = this->_spline.query_state(time);                                        // Get the state as a 6x1 vector over real numbers
    
    // Now we need to convert the "position" vector to a pose

    DataType angle = state.position.tail(3).norm();                                                 // Norm of the angle*axis component

    Pose<DataType> pose;                                                                            // We need to compute this

    if(abs(angle) < 1e-04) pose = Pose<DataType>(state.position.head(3),
                                                 Eigen::Quaternion<DataType>(1,0,0,0));             // Assume zero rotation
    else
    {
      Eigen::Vector<DataType,3> axis = state.position.tail(3).normalized();                         // Ensure magnitude of 1 
      
      pose = Pose<DataType>(state.position.head(3), 
                            Eigen::Quaternion<DataType>(cos(0.5*angle),
                                                        sin(0.5*angle)*axis(0),
                                                        sin(0.5*angle)*axis(1),
                                                        sin(0.5*angle)*axis(2)));
    }

    CartesianState<DataType> returnValue = {pose, state.velocity, state.acceleration};              // Put them together in data structur
    
    return returnValue;
}

#endif
