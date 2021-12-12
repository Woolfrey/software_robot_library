/*
*	A minimum acceleration trajectory across multiple points.
*
*	Can interpolate quaternions.
*/

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <Eigen/Geometry>							// Eigen::Quaternionf and the like
#include <vector>								// std::vector

class CubicSpline
{
	public:
		// Empty constructor
		CubicSpline() {}
		
		// Constructor for real numbers
		CubicSpline(const std::vector<Eigen::VectorXf> &points,
			const std::vector<float> &times);
			
		// Constructor for quaternions
		CubicSpline(const std::vector<Eigen::Quaternionf> &points,
			const std::vector<float> &times);
	
	private:
		// Variables
		bool isQuaternion;						// Self explanatory
		int m, n;							// m dimensions with n waypoints
		std::vector<Eigen::VectorXf> a, b, c, d;			// Spline coefficients
		std::vector<Eigen::VectorXf> point;				// Waypoints
		std::vector<float> time;					// Time to reach each waypoint
		
		// Functions		
		bool check_inputs(const std::vector<Eigen::VectorXf> &points,
				const std::vector<float> &times);
		
};										// Semicolon needed after class declaration

/******************** Constructor for real numbers ********************/
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &points,
			const std::vector<float> &times)
			:
			isQuaternion(false),
			m(points[0].size()),					// Number of dimensions
			n(points.size()),					// Number of waypoints
			point(points),
			time(times)
			
{
	check_inputs(points, times);						// Check that the inputs are sound
	
	// Initialize size of std::vector objects
	for(int i = 0; i < this->m; i++)
	{
		this->a.push_back(Eigen::VectorXf::Zero(this->n-1));		// NOTE: There are n-1 splines!
		this->b.push_back(Eigen::VectorXf::Zero(this->n-1));
		this->c.push_back(Eigen::VectorXf::Zero(this->n-1));
		this->d.push_back(Eigen::VectorXf::Zero(this->n-1));
	}
	
	// Relationship between acceleration & position: A*sddot = B*s
	Eigen::MatrixXf A(this->n, this->n);
	Eigen::MatrixXf B(this->n, this->n);
	
	if(!this->isQuaternion)
	{
		A(0,0) = (this->time[1] - this->time[0])/2;
		A(0,1) = (this->time[1] - this->time[0])/2 + (this->time[2] - this->time[1])/3;
		A(0,2) = (this->time[2] - this->time[1])/6;
	}
	else
	{
	}
	
}

/******************** Constructor for orientation ********************/
CubicSpline::CubicSpline(const std::vector<Eigen::Quaternionf> &points,
			const std::vector<float> &times)
			:
			isQuaternion(true)
{

}

/********************* Check the input dimensions to the constructors are correct ********************/
bool CubicSpline::check_inputs(const std::vector<Eigen::VectorXf> &points, const std::vector<float> &times)
{
	bool OK = true;
	// Check the dimensions are sound
	if(points.size() != times.size())
	{
		// ERROR: std::vector objects must be the same length!
		
		OK = false;
	}
	else if(points.size() < 3)
	{
		// ERROR: A minimum of 3 points is required for CubicSpline
		
		OK = false;
	}
	
	for(int i = 0; i < times.size()-1; i++)
	{
		if(times[i+1] < times[i])
		{
			// ERROR: Times are not in ascending order!
			OK = false;
		}
	}
	
	return OK;
}

#endif

/* OLD STUFF

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <Eigen/Core>
#include <vector>


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
/******************** Empty constructor ********************
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
