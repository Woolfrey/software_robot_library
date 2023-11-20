/**
 * @file   Waypoints.h
 * @author Jon Woolfrey
 * @date   November 2023
 * @brief  A base class for trajectories connected via waypoints.
 */
 
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
			return query_state(time).position;                                          // Too easy lol
		 }
		 
		 /**
		  * Get the position, velocity, and acceleration for the given time.
		  * @param time The point at which to evaluate the state.
		  * @return A State data structure containing position, velocity, and acceleration.
		  */
		  State<DataType> query_state(const DataType &time)
		  {
		  	unsigned int trajectoryNumber;
		  	
		  	if(time < this->_time.front())	    trajectoryNumber = 0;                   // Not yet start; first trajectory
		  	else if(time >= this->_time.back()) trajectoryNumber = this->_numPoints-1;  // Finished; final trajectory
		  	else
		  	{
		  		for(int i = 0; i < this->numPoints-1; i++)
		  		{
		  			if(time < this->_time[i])
		  			{
		  				trajectoryNumber = i-1;     
		  				break;
		  			}
		  		}
		  	}
			
			return this->_trajectory[trajectoryNumber].query_state(time);
		}
	
	private:
		
		unsigned int _numberOfWaypoints;
		
		std::vector<DataType> _time;                                                        // The time at which to  pass each waypoint
		
		std::vector<ClassType> _trajectory;                                                 // An array of trajectories
	
};                                                                                                  // Semicolon needed after class declaration
