/**
 * @file    SplineTrajectory.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   This class gives a spline as a function of time over a finite series of points.
 * 
 * @details Given a set of points and times, this class will generate piece-wise polynomials to interpolate
 *          between them. It ensures continuity at the velocity and acceleration level.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
#include <Trajectory/SplineTrajectory.h>

namespace RobotLibrary { namespace Trajectory {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Constructor                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
SplineTrajectory::SplineTrajectory(const std::vector<RobotLibrary::Trajectory::State> &waypoints,
                                   const std::vector<double> &times,
                                   const unsigned int        &polynomialOrder)
: TrajectoryBase(waypoints.front(),
                 waypoints.back(),
                 times.front(),
                 times.back())
{
    if(waypoints.size() != times.size())
    {
        throw std::invalid_argument("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                    "Size of input vectors do not match. "
                                    "There were " + std::to_string(waypoints.size()) + " waypoints, "
                                    "but " + std::to_string(times.size()) + " times.");
    }
    else if(waypoints.size() < 2)
    {
        throw std::invalid_argument("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                    "A  minimum of 2 waypoints is required to construct a trajectory.");
    }
    
    for(int i = 0; i < times.size()-1; i++)
    {
        if(times[i] == times[i+1])
        {
            throw std::invalid_argument("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                        "The time for waypoint " + std::to_string(i) + " was the same "
                                        "as the time for waypoint " + std::to_string(i+1) + ". ("
                                        + std::to_string(times[i]) + " == " + std::to_string(times[i+1]) + "). "
                                        "You cannot move faster than the speed of light! ヽ༼ ಠ益ಠ ༽ﾉ");
        }
        else if(times[i] >= times[i+1])
        {
            throw std::logic_error("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                   "The time for waypoint " + std::to_string(i) + " was less "
                                   "than the time for waypoint " + std::to_string(i+1) + ". ("
                                   + std::to_string(times[i]) + " == " + std::to_string(times[i+1]) + "). "
                                   "You cannot go back in time! ヽ༼ ಠ益ಠ ༽ﾉ");
        }
    } 
              
    std::vector<RobotLibrary::Math::FunctionPoint> points(waypoints.size());
    
    for(int i = 0; i < this->_dimensions; i++)
    {
        for(int j = 0; j < waypoints.size(); j++)
        {
            points[j] = { waypoints[j].position[i],
                          waypoints[j].velocity[i],
                          waypoints[j].acceleration[i] };
        }
        
        this->_spline.emplace_back(points,times,polynomialOrder);
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Constructor                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
SplineTrajectory::SplineTrajectory(const std::vector<Eigen::VectorXd> &positions,
                                   const std::vector<double> &times,
                                   const Eigen::VectorXd &startVelocity)
: TrajectoryBase({positions.front(), startVelocity, Eigen::VectorXd::Zero(positions.front().size())},
                 {positions.back(), Eigen::VectorXd::Zero(positions.front().size()), Eigen::VectorXd::Zero(positions.front().size())},
                 times.front(),
                 times.back())
{
    if(positions.size() < 2)
    {
        throw std::invalid_argument("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                    "A minimum of 2 points is required to generate a trajectory.");
    }
    else if(positions.size() != times.size())
    {
        throw std::invalid_argument("[ERROR] [SPLINE TRAJECTORY] Constructor: "
                                    "Dimensions of arguments do not match. There were " + std::to_string(positions.size()) + " "
                                    "positions specified, but " + std::to_string(times.size()) + " times.");
    }
    
    this->_dimensions = positions.front().size();                                                   // This is in the underlying TrajectoryBase class
    
    // Create an individual spline for every dimension
    for(int j = 0; j < this->_dimensions; j++)
    { 
        std::vector<double> pos;                                                                    // We will use this to fit derivatives of a cubic spline
        
        for(int i = 0; i < positions.size(); i++)
        {
            pos.push_back(positions[i][j]);
        }
        
        std::vector<double> vel;                                                                    // This is conditional
        
        if(positions.size() > 2)                                                                    // We need to optimise the intermediate velocities
        {        
            // This function fits velocities at intermediate points to ensure
            // continuity down to acceleration level
            vel = RobotLibrary::Math::solve_cubic_spline_derivatives(pos,times, startVelocity[j], 0.0);
        }
        else
        {
            vel = {startVelocity[j], 0.0};                                                          // No intermediate points, so just use the start & end
        }
        
        std::vector<RobotLibrary::Math::FunctionPoint> points;                                      // Contains position, velocity, acceleration data
        
        for(int i = 0; i < positions.size(); i++)
        {
            points.push_back({pos[i], vel[i], 0.0});                                                // Acceleration doesn't actually matter for a cubic spline
        }
        
        this->_spline.emplace_back(points,times,3);                                                 // Create a 3rd order polynomial using specified function
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Query the state for the given time                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Trajectory::State
SplineTrajectory::query_state(const double &time)
{
         if(time <= this->_startTime) return this->_startPoint;
    else if(time >= this->_endTime)   return this->_endPoint;
    else
    {
        using namespace Eigen;
        
        RobotLibrary::Trajectory::State state = { VectorXd::Zero(this->_dimensions),
                                                  VectorXd::Zero(this->_dimensions),
                                                  VectorXd::Zero(this->_dimensions) };
                                  
        for(int i = 0; i < this->_dimensions; i++)
        {
            const auto &[pos, vel, acc] = this->_spline[i].evaluate_point(time);
            
            state.position[i]     = pos;
            state.velocity[i]     = vel;
            state.acceleration[i] = acc;
        }
        
        return state;
    }
}

} } // namespace
