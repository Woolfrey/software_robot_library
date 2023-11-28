/**
 * @file   MultiTrapezoid.h
 * @author Jon Woolfrey
 * @date   November 2023
 * @brief  Specifies a series of trapezoidal velocity profiles across several points.
 */
 
 #ifndef MULTITRAPEZOID_H_
 #define MULTITRAPEZOID_H_
 
 #include <TrapezoidalVelocity.h>
 #include <Waypoints.h>
 
 template <class DataType>
 class MultiTrapezoid : public Waypoints<DataType,TrapezoidalVelocity<DataType>>
 {
 	public:
 		/**
 		 * Constructor.
 		 * @param waypoints The points to pass through along the trajecory.
 		 * @param maxVelocity The maximum speed to move at.
 		 * @param maxAcceleration The maximum acceleration & deceleration.
 		 */
 		MultiTrapezoid(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		               const DataType &maxVelocity,
 		               const DataType &maxAcceleration,
 		               const DataType &startTime);
 		               
		/**
		 * @return The time at which this trajectory finishes.
		 */
		DataType end_time() const { return this->_trajectory.back().end_time(); }
 };                                                                                                 // Semicolon needed after class declaration
 
  //////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
MultiTrapezoid<DataType>::MultiTrapezoid(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
                                         const DataType &maxVelocity,
                                         const DataType &maxAcceleration,
                                         const DataType &startTime)
{
	// Set the values in the underlying Waypoints class
	this->_numberOfWaypoints = waypoints.size();
	this->_time.push_back(startTime);
	
	if(this->_numberOfWaypoints < 2)
	{
		throw std::invalid_argument("[ERROR] [MULTI TRAPEZOID] Constructor: "
		                            "A minimum of 2 waypoints is required, but you provided "
		                            + std::to_string(waypoints.size()) + ".");
	}
	
	for(int i = 0; i < this->_numberOfWaypoints-1; i++)
	{
		this->_trajectory.push_back(TrapezoidalVelocity<DataType>(waypoints[i],waypoints[i+1],
		                                                          maxVelocity,maxAcceleration,
		                                                          this->_time.back()));

		this->_time.push_back(this->_trajectory.back().end_time());
	}
}

#endif
