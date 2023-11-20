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
 class MultiTrapezoid<DataType> : public Waypoints<DataType,TrapezoidalVelocity<DataType>>
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
 		               const DataType &maxAcceleration);
 	
 	private:
 };                                                                                                 // Semicolon needed after class declaration
 
  //////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                         //
//////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
MultiTrapezoid<DataType>::MultiTrapezoid(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
                                         const DataType &maxVelocity,
                                         const DataType &maxAcceleration)
{
	
}

#endif
