/**
 * @file   SplineTrajectory.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining trajectories with waypoints joined over splines.
 */
 
#ifndef SPLINETRAJECTORY_H_
#define SPLINETRAJECTORY_H_

#include "../Math/MathFunctions.h"
#include "../Math/Spline.h"
#include "TrajectoryBase.h"

template <class DataType>
class SplineTrajectory : public TrajectoryBase<DataType>
{
    public:
        
        /**
         * An empty constructor.
         */
        SplineTrajectory() {}
        
        /**
         * Constructor.
         * @param waypoints The states (position, velocity, acceleration) for given points in time.
         * @param times The time at which to pass through each waypoint.
         * @param polynomialOrder The type of polynomial interpolation to use.
         */
        SplineTrajectory(const std::vector<State<DataType>> &waypoints,
                         const std::vector<DataType> &times,
                         const unsigned int &polynomialOrder);
        
        /**
         * A basic constructor for a cubic spline where the waypoints and start velocity are given.
         * The final velocity is assumed to be zero.
         * @param positions An array of positions to pass through.
         * @param times The time at which to pass through each position.
         * @param startVelocity The initial velocity of the trajectory.
         */
        SplineTrajectory(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &positions,
                         const std::vector<DataType> &times,
                         const Eigen::Vector<DataType, Eigen::Dynamic> &startVelocity)
                         : TrajectoryBase<DataType>({positions.front(), startVelocity, Eigen::Vector<DataType,Eigen::Dynamic>::Zero(positions.front().size())},
                                                    {positions.back(), Eigen::Vector<DataType,Eigen::Dynamic>::Zero(positions.front().size()), Eigen::Vector<DataType,Eigen::Dynamic>::Zero(positions.front().size())},
                                                    times.front(), times.back())
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
            
            this->_dimensions = positions.front().size();                                           // This is in the underlying TrajectoryBase class
            
            // Create an individual spline for every dimension
            for(int j = 0; j < this->_dimensions; j++)
            { 
                std::vector<DataType> pos;                                                          // We will use this to fit derivatives of a cubic spline
                
                for(int i = 0; i < positions.size(); i++)
                {
                    pos.push_back(positions[i][j]);                                                 // 
                }
                
                std::vector<DataType> vel;                                                          // This is conditional
                
                if(positions.size() > 2)                                                            // We need to optimise the intermediate velocities
                {        
                    // This function fits velocities at intermediate points to ensure
                    // continuity down to acceleration level
                    vel = solve_cubic_spline_derivatives(pos,times, startVelocity[j], 0.0);
                }
                else
                {
                    vel = {startVelocity[j], 0.0};                                                  // No intermediate points, so just use the start & end
                }
                
                std::vector<FunctionPoint<DataType>> points;                                        // Contains position, velocity, acceleration data
                
                for(int i = 0; i < positions.size(); i++)
                {
                    points.push_back({pos[i], vel[i], 0.0});                                        // Acceleration doesn't actually matter for a cubic spline
                }
                
                this->_spline.emplace_back(points,times,3);                                         // Create a 3rd order polynomial using specified function
            }
        }
                                                
        /**
         * Query the state for the given input time.
         * This overrides the virtual function in the TrajectoryBase class.
         * @param time The point at which to evaluate the trajectory.
         * @return The position, velocity, acceleration as a State data structure.
         */
        inline
        State<DataType>
        query_state(const DataType &time);
    
    private:
        
        std::vector<Spline<DataType>> _spline;
};                                                                                                  // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Constructor                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
SplineTrajectory<DataType>::SplineTrajectory(const std::vector<State<DataType>> &waypoints,
                                             const std::vector<DataType> &times,
                                             const unsigned int &polynomialOrder)
                                             : TrajectoryBase<DataType>(waypoints.front(),
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
              
    std::vector<FunctionPoint<DataType>> points(waypoints.size());
    
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
 //                             Query the state for the given time                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
State<DataType>
SplineTrajectory<DataType>::query_state(const DataType &time)
{
         if(time <= this->_startTime) return this->_startPoint;
    else if(time >= this->_endTime)   return this->_endPoint;
    else
    {
        using namespace Eigen;
        
        State<DataType> state = { Vector<DataType,Dynamic>::Zero(this->_dimensions),
                                  Vector<DataType,Dynamic>::Zero(this->_dimensions),
                                  Vector<DataType,Dynamic>::Zero(this->_dimensions) };
                                  
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

#endif
