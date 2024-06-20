/**
 * @file   SplineTrajectory.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining trajectories with waypoints joined over splines.
 */
 
#ifndef SPLINETRAJECTORY_H_
#define SPLINETRAJECTORY_H_
 
#include <Math/Spline.h>
#include <Trajectory/TrajectoryBase.h>

template <class DataType>
class SplineTrajectory : public TrajectoryBase<DataType>
{
    public:
        
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
              
    std::vector<std::array<DataType,3>> points(waypoints.size());
    
    for(int i = 0; i < this->_dimensions; i++)
    {
        for(int j = 0; j < waypoints.size(); j++)
        {
            points[j][0] = waypoints[j].position[i];
            points[j][1] = waypoints[j].velocity[i];
            points[j][2] = waypoints[j].acceleration[i];
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
