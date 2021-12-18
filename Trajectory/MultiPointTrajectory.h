/*
*	Generates a trajectory in the real numbers of m dimensions across n waypoints.
*	If n = 2, it automatically generates a quintic polynomial (minimum jerk)
*	If n = 3, it automatically generates a cubic spline (minimum acceleration)
*	
*/

#ifndef MULTI_POINT_TRAJECTORY_H_
#define MULTI_POINT_TRAJECTORY_H_

#include <CubicSpline.h>							// Custom trajectory class
#include <Quintic.h>								// Custom trajectory class
#include <string>								// std::string

class MultiPointTrajectory
{
	public:
		// Constructor
		MultiPointTrajectory() {}
		
		MultiPointTrajectory(const std::vector<Eigen::VectorXf> &points,
				const std::vector<float> &times);
				
		// Get functions
		
		void get_state(Eigen::VectorXf &pos,				// Get the state for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
				
		std::string get_type();					// Get the type of trajectory
	
	private:
		int m, n;							// m dimensions with n waypoints
		
		CubicSpline cubic;						// Trajectory for n > 2
		Quintic quintic;						// Trajectory for n = 2
		
};										// Semicolon needed after a class declaration

/******************** Constructor ********************/
MultiPointTrajectory::MultiPointTrajectory(const std::vector<Eigen::VectorXf> &points,
					const std::vector<float> &times)
					:
					m(points[0].size()),
					n(points.size())
{
	// Check the inputs are sound
	if(points.size() != times.size())
	{
		std::cout << "ERROR: MultiPointTrajectory::MultiPointTrajectory() : Vectors are not the same length!" << std::endl;
	}
	else if(points.size() < 2)
	{
	std::cout << "ERROR: MultiPointTrajectory::MultiPointTrajectory() : A minimum of 2 points is needed to generate a trajectory." << std::endl;
	}
	
	if(this->n < 3)	this->quintic = Quintic(points[0], points[1], times[0], times[1]);
	else			this->cubic = CubicSpline(points, times);
}

/******************** Get the state for the given time ********************/
void MultiPointTrajectory::get_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	if(pos.size() != this->m || vel.size() != this->m || acc.size() != this->m)
	{
		std::cout << "ERROR: MultiPointTrajectory::get_state() : Vectors must have a length of " << this->m << "." << std::endl;
	}
	
	if(this->n < 3) 	this->quintic.get_state(pos, vel, acc, time);
	else			this->cubic.get_state(pos, vel, acc, time);
}

std::string MultiPointTrajectory::get_type()
{
	if(this->n < 2)	return "ERROR: Unknown type! How did this happen?";
	else if(this->n == 2)	return "quintic polynomial";
	else			return "cubic spline";
}

#endif
