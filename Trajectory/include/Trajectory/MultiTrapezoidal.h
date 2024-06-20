/**
 * @file   MultiTrapezoidal.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class for waypoints connected via trapezoidal velocity profile trajectories.
 */

#ifndef MULTITRAPEZOIDAL_H_
#define MULTITRAPEZOIDAL_H_

#include <Trajectory/TrapezoidalVelocity.h>

#endif

template class <DataType>
class MultiTrapezoidal
{
    public:
    
        /**
         * Constructor.
         * @param waypoints An array of positions to pass through.
         * @param maxVelocity Defines the height for the trapezoidal velocity profile.
         * @param maxAcceleration Defines the slope of the trapezoidal velocity.
         * @param startTime When the trajectory commences.
         */
        MultiTrapezoidal(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
                         const DataType &maxVelocity,
                         const DataType &maxAcceleration,
                         const DataType &startTime);
    
        /**
         * Query the trajectory state for the given time.
         */
        inline
        State<DataType>
        query_state(const DataType &time);
        
    private:
    
        std::vector<DataType> _times;
        
        std::vector<TrapezoidalVelocity<DataType>> _trajectories;   
        
};                                                                                                  // Semicolon required after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
MultiTrapezoidal<DataType>::MultiTrapezoidal(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
                                             const DataType &maxVelocity,
                                             const DataType &maxAcceleration,
                                             const DataType &startTime)
{ 
    if(waypoints.size() < 2)
    {
          throw std::invalid_argument("[ERROR] [MULTI TRAPEZOIDAL] Constructor: "
                                      "A minimum of 2 waypoints is required, but you provided "
                                      + std::to_string(waypoints.size()) + ".");
    }
    
    this->_times.push_back(startTime);
    
    for(int i = 0; i < waypoints.size()-1; i++)
    {
        this->_trajectories.push_back(TrapezoidalVelocity<DataType>(waypoints[i], waypoints[i+1],
                                                                    maxVelocity, maxAcceleration,
                                                                    this->_times.back());

        this->_times.push_back(this->_trajectories.back().end_time());
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
State<DataType>
MultiTrapezoidal<DataType>::query_state(const DataType &time)
{
    for(int i = 0; i < this->_times.size()-1; i++)
    {
        if(time < this->_times[i+1])
        {
            return this->_trajectories[i].query_state(time);                                        // Must be on the ith trajectory
        }
        else
        {
            return this->_trajectories.back().query_state(time);                                    // Final trajectory
        }
    }
}

#endif
