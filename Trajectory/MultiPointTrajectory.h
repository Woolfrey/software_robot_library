/*
*	Generates a trajectory in the real numbers of m dimensions across n waypoints.
*	If n = 2, it automatically generates a quintic polynomial (minimum jerk)
*	If n = 3, it automatically generates a cubic spline (minimum acceleration)
*	
*/

#ifndef MULTI_POINT_TRAJECTORY_H_
#define MULTI_POINT_TRAJECTORY_H_

#include <CubicSpline.h>
#include <Quintic.h>

class MultiPointTrajectory
{
	public:
		// Constructor
		MultiPointTrajectory(const std::vector<Eigen::VectorXf> &points,
				const std::vector<float> &times);
				
		// Get functions
		void get_state(Eigen::VectorXf &pos,				// Get the state for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
	
	private:
		int m, n;							// m dimensions with n waypoints
		
		CubicSpline cubic;						// Trajectory for n > 2
		Quintic quintic;						// Trajectory for n = 2
		
};										// Semicolon needed after a class declaration

#endif



/* OLD CODE
class MultiPointTrajectory
{
	public:
		MultiPointTrajectory(	std::vector<Eigen::VectorXf> &points,			// Constructor
					std::vector<float> &times); 
		
		void get_state(Eigen::VectorXf &pos,						// Get the desired state for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
				
		std::string get_type();							// Returns the type of trajectory
	
	private:
		
		int m, n;									// m dimensions, n waypoints
		
		int type;									// 1 = Quintic, 2 = CubicSpline
		
		CubicSpline cubic_trajectory;							// Trajectory between 3 or more points
		Quintic quintic_trajectory;							// Trajectory between 2 points
		
};												// Semicolon required after class declaration

/******************** Constructor ********************
MultiPointTrajectory::MultiPointTrajectory(	std::vector<Eigen::VectorXf> &points,
						std::vector<float> &times)
						:
						m(points[0].size()),
						n(points.size())						
{
	
	// Check that the input dimensions are all sound
	for(int i = 0; i < this->n-1; i++)
	{
		if(points[i].size() != points[i+1].size())
		{
			// ERROR: Vectors aren't all the same length
		}
	}
	if(points.size() != times.size())
	{
		// ERROR: There needs to be equal times for the number of points
	}
	
	// Create the appropriate type of trajectory
	if(this->n == 2)
	{
		this->type = 1;								// Tell the object to call Quintic in future
		this->quintic_trajectory = Quintic(points[0], points[1], times[0], times[1], false); // False = not a quaternion
	}
	else if(this->n > 2)
	{
		this->type = 2;								// Tell this object to call CubicSpline in future
	 	this->cubic_trajectory = CubicSpline(points, times, false);			// False = not a quaternion
	}
	else
	{	
		// ERROR: There has to be at least 2 points to define a trajectory
	}
}

/******************** Get the desired position, velocity, and acceleration for given time ********************
void MultiPointTrajectory::get_state(	Eigen::VectorXf &pos,
					Eigen::VectorXf &vel,
					Eigen::VectorXf &acc,
					const float &time)
{
	// Check inputs are sound
	if(pos.size() != this->n || vel.size() != this->n || acc.size() != this->n)
	{
		// ERROR: Vectors aren't all of the same length.
	}
	
	// Compute values based on type of trajectory
	switch(this->type)
	{
		case 1: 									// Quintic polynomial
			this->quintic_trajectory.get_state(pos, vel, acc, time);
			break;

		case 2: 									// Cubic spline
			this->cubic_trajectory.get_state(pos, vel, acc, time);
			break;
		
		default:
			// ERROR: Incorrect type defined. How did that happen?
			break;
	}
}

/******************** Returns the type of trajectory ********************
std::string MultiPointTrajectory::get_type()
{
	switch(this->type)
	{
		case 1:
			return "quintic polynomial";
		
		case 2:
			return "cubic spline";
		
		default:
			return "ERROR: unknown type. How did that happen?";
	}
}
*/
