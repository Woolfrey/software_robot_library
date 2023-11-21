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
	for(int i = 0; i < waypoints.size()-1; i++)
	{
		DataType t;
		
		if(i == 1) t = startTime;
		else       t = this->_trajectory[i-1].end_time();                                   // Start time of this trajectory is end time of previous
		
		this->_trajectory[i].emplace_back(waypoints[i],waypoints[i+1],maxVelocity,maxAcceleration,t);
	}
}

#endif
