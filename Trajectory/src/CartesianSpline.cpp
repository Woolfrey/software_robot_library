/**
 * @file    CartesianSpline.cpp
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
 
#include <Trajectory/CartesianSpline.h>

namespace RobotLibrary { namespace Trajectory {

  //////////////////////////////////////////////////////////////////////////////////////////////////// 
 //                              Constructor for cubic splines                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianSpline::CartesianSpline(const std::vector<RobotLibrary::Model::Pose> &poses,
                                 const std::vector<double> &times,
                                 const Eigen::Vector<double,6> &startTwist)
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
    std::vector<Eigen::VectorXd> positions(poses.size());                                           // NOTE: The SplineTrajectory class expects a Dynamic size vector
    for(int i = 0; i < positions.size(); i++)
    {   
        positions[i].resize(6);                                                                     // 3 for position, 3 for orientation
        
        positions[i].head(3) = poses[i].translation();                                              // First part is just the translation
        
        double angle = 2.0 * acos(std::clamp(poses[i].quaternion().normalized().w(), -1.0, 1.0));   // Get angle component
        
        if (abs(angle) < 1e-03) positions[i].tail(3).setZero();                                     // Practically zero
        else                    positions[i].tail(3) = angle * poses[i].quaternion().vec().normalized();
    }
    
    this->_spline = SplineTrajectory(positions,times,startTwist);                                   // Generates a cubic spline. Velocities automatically calculated.  
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Trajectory::CartesianState
CartesianSpline::query_state(const double &time)
{
    RobotLibrary::Trajectory::State state = this->_spline.query_state(time);                        // Get the state as a 6x1 vector over real numbers
    
    // Now we need to convert the "position" vector to a pose

    double angle = state.position.tail(3).norm();                                                   // Norm of the angle * saxis component

    RobotLibrary::Model::Pose pose;                                                                 // We need to compute this

    if (angle < 1e-04) pose = RobotLibrary::Model::Pose(state.position.head(3),
                                                        Eigen::Quaterniond(1,0,0,0));               // Assume zero rotation
    else
    {
      Eigen::Vector<double,3> axis = state.position.tail(3).normalized();                           // Ensure magnitude of 1 
      
      pose = RobotLibrary::Model::Pose(state.position.head(3), 
                                       Eigen::Quaterniond(cos(0.5*angle),
                                                          sin(0.5*angle)*axis(0),
                                                          sin(0.5*angle)*axis(1),
                                                          sin(0.5*angle)*axis(2)));
    }

    RobotLibrary::Trajectory::CartesianState returnValue = {pose, state.velocity, state.acceleration};   // Put them together in data structur
    
    return returnValue;
}

} } // namespace
