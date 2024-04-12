/**
 * @file   Waypoints.h
 * @author Jon Woolfrey
 * @date   November 2023
 * @brief  A base class for trajectories connected via waypoints.
 */
 
#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_
 
#include <Polynomial.h>
#include <TrapezoidalVelocity.h>

template <class DataType, typename ClassType>
class Waypoints
{
	public:
		/**
		 * Empty constructor.
		 */
		Waypoints() {}
		
		/**
		 * Get the position for the given time.
		 * @param time The point at which to evaluate the trajectory.
		 * @return The position.
		 */
		 Eigen::Vector<DataType,Eigen::Dynamic> query_position(const DataType &time)
		 {
			return query_state(time).position;                                                   // Too easy lol
		 }
		 
		 /**
		  * Get the position, velocity, and acceleration for the given time.
		  * @param time The point at which to evaluate the state.
		  * @return A State data structure containing position, velocity, and acceleration.
		  */
		  State<DataType> query_state(const DataType &time)
		  {
		  	if(time <= this->_time.front())
		  	{
		  		return this->_trajectory.front().query_state(time);                             // Must be on the initial trajectory
		  	}
		  	else if(time > this->_time.back())
		  	{
		  		return this->_trajectory.back().query_state(time);                              // Must be on the final trajectory
		  	}
		  	else                                                                                 // Somewhere inbetween
		  	{
		  		unsigned int number;
		  		
		  		for(int i = 1; i < this->_time.size(); i++)
		  		{
		  			if(time <= this->_time[i])
		  			{
		  				number = i-1;
		  				break;
		  			}
		  		}
		  		
		  		return this->_trajectory[number].query_state(time);
		  	}
		}
	
	protected:
		
		unsigned int _numberOfWaypoints;
		
		std::vector<DataType> _time;                                                              // The time at which to  pass each waypoint
		
		std::vector<ClassType> _trajectory;                                                       // An array of trajectories
	
};                                                                                                  // Semicolon needed after class declaration

#endif
