/*
*	A minimum jerk trajectory between two points in space.
*
*	Can interpolate quaternions.
*/
#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <Eigen/Core>
#include <vector>


/*

class CubicSpline
{
	public:
		CubicSpline();								// Empty constructor
		
		
		
		CubicSpline(const std::vector<Eigen::VectorXf> &waypoints,		// Proper constructor
			    const std::vector<float> &times,
			    bool is_quaternion);
	
		void get_state(Eigen::VectorXf &pos,					// Get desired state at given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
	private:
		bool isQuaternion;
		
		int m, n;								// m dimensions, n waypoints
	
		std::vector<Eigen::VectorXf> point;					// Array of points to pass through
		std::vector<float> time;						// Array of times to pass each point
		std::vector<std::vector<float>> a, b, c, d;
		
};											// Needed after a class declaration

/******************** Empty constructor ********************/
CubicSpline::CubicSpline()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Proper constructor ********************
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &waypoints,
			const std::vector<float> &times,
			bool is_quaternion)
				:
				m(waypoints[0].size()),					// Dimensions of space
				n(waypoints.size()),						// Number of waypoints
				point(waypoints),
				time(times),
				isQuaternion(is_quaternion)
{
	// Check input dimensions are sound
	if(n < 3)
	{
		// ERROR: Need a minimum of 3 points to define a cubic spline
	}
	for(int i = 0; i < n-1; i++)
	{
		if(waypoints[i].size() != waypoints[i+1].size())
		{
			// ERROR: Vectors aren't all the same length.
		}
	}
	
	// Resize all the appropriate arrays
	
}

/******************** Get desired state at given time ********************
void CubicSpline::get_state(	Eigen::VectorXf &pos,
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time)
{
	// 1. Check input dimensions are sound
	
	// 2. Figure out where along the trajectory we are
	
	// 3. Compute the desired state
		
		// 3. a) For linear interoplation
		// 3. b) For quaternion interpolation
}
*/

#endif
