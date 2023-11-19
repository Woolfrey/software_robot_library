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
		 Vector<DataType,Dynamic> query_position(const DataType &time)
		 {
			return query_state(time).position;                                          // Too easy lol
		 }
		 
		 /**
		  * Get the position, velocity, and acceleration for the given time.
		  * @param time The point at which to evaluate the state.
		  * @return A State data structure containing position, velocity, and acceleration.
		  */
		  State query_state(const DataType &time)
		  {
		  	unsigned int trajectoryNum;
		  	
		  	if(time < this->_time.front())	    trajectoryNumber = 0;
		  	else if(time >= this->_time.back()) trajectoryNumber = this->_numPoints-1;
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
		
		unsigned int _numPoints;
		
		std::vector<DataType> _time;
		
		std::vector<ClassType> _trajectory;
	
};                                                                                                  // Semicolon needed after class declaration
