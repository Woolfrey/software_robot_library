/**
 * @file   Spline.h
 * @author Jon Woolfrey
 * @date   November 2023
 * @brief  Specifies a series of polynomial trajectories joined together.
 */
 
 #ifndef SPLINE_H_
 #define SPLINE_H_
 
 #include <Eigen/Dense>                                                                             // Allows matrix decomposition
 #include <Polynomial.h>
 #include <Waypoints.h>
 
 template <class DataType>
 class Spline : public Waypoints<DataType, ClassType>
 {
 	public:
 		Spline() {}
 		
	private:
	
 };                                                                                                 // Semicolon needed after class declaration
