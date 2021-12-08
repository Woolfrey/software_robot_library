/*
*	A trajectory generator in Cartesian space.
*/

#ifndef CARTESIAN_TRAJECTORY_H_
#define CARTESIAN_TRAJECTORY_H_

#include <CubicSpline.h>
#include <Quintic.h>

class CartesianTrajectory
{
	public:
		// Constructor
		CartesianTrajectory(const std::vector<Eigen::Affine3f> &poses,
				const std::vector<float> &times);
		
		void get_state(Eigen::Affine3f &pose,				// Get the desired state for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
		
	private:
		int n;								// Number of waypoints
		
		// Trajectories for n > 2
		CubicSpline cubicPosition;
		CubicSpline cubicOrientation;
		
		// Trajectories for n = 2
		Quintic quinticPosition;
		Quintic quinticOrientation;
		
};										// Semicolon needed after a class declaration

CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Affine3f> &poses,
					const std::vector<float> &times)
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

#endif

/*
class CartesianTrajectory
{
	public:
		CartesianTrajectory();								// Empty constructor
		
		CartesianTrajectory(const std::vector<Eigen::Affine3f> &waypoints,		// Proper constructor
				const std::vector<float> &times);
		
		std::string get_type();							// Return the type of trajectory being used
		
		void get_state(Eigen::Affine3f &pos,						// Get the desired state for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
				
	
	private:
		int n;										// Number of waypoints
		int type;									// 1 = Quintic, 2 = Cubic Spline
		
		CubicSpline cubic_position;							// Cubic spline object for position trajectory
		CubicSpline cubic_orientation;						// Cubic spline object for orientation trajector
		
		Quintic quintic_position;							// Quintic object for position trajectory
		Quintic quintic_orientation;							// Cubic object for orientation trajectory
		
		// These are used to receive and combine state information from the separate
		// position and orientation trajectories 
		Eigen::VectorXf position, quaternion, linear_vel, angular_vel, linear_acc, angular_acc;
		float angle;
		Eigen::Vector3f axis;

};												// Semicolon needed after a class declaration

/******************** Empty constructor ********************
CartesianTrajectory::CartesianTrajectory()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Proper constructor ********************
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Affine3f> &waypoints,
					const std::vector<float> &times)
					:
					n(waypoints.size())					// Get the number of waypoints
{

	// Check that the input dimensions are sound
	if(times.size() != waypoints.size())
	{
		// ERROR: Vectors are not of the same length.
	}

	// Extract the position vectors, and the orientation as quaternion vectors
	std::vector<Eigen::VectorXf> p;
	p.resize(this->n);
	std::vector<Eigen::VectorXf> q;
	q.resize(this->n);
	Eigen::Quaternionf temp;
	
	for(int i = 0; i < n; i++)
	{
		p[i].resize(3);
		p[i](0) = waypoints[i].translation().x();
		p[i](1) = waypoints[i].translation().y();
		p[i](2) = waypoints[i].translation().z();
		
		temp = Eigen::Quaternionf(waypoints[i].rotation());				// Get the rotation as a quaternion
		temp.normalize();								// Ensure unit norm
		q[i].resize(4);						
		q[i](0) = temp.w();								// Put Quaternionf object in to Vector4f object
		q[i](1) = temp.x();
		q[i](2) = temp.y();
		q[i](3) = temp.z();
	}
	
	// Create the appropriate type of trajectory
	if(this->n == 2)									// Create quintic polynomial trajectories
	{
		this->type = 1;								// Tell this object to call quintic trajectory in the future
		this->quintic_position    = Quintic(p[0], p[1], times[0], times[1], false);	// False = linear interpolation
		this->quintic_orientation = Quintic(q[0], q[1], times[0], times[1], true); 	// True = quaternion interpolation
	}
	else if(this->n > 2)									// Create cubic spline trajectories
	{
		this->type = 2;								// Tell this object to call the cubic spline in the future
		this->cubic_position    = CubicSpline(p, times, false);			// False = linear interpolation
		this->cubic_orientation = CubicSpline(q, times, true);			// True = quaternion interpolation
	}
	else
	{
		// ERROR: Minimum of 2 waypoints is required.
	}
	
	// The trajectory classes only accept a VectorXf object when using pass-by-reference.
	// Need to resize the storage variables here. Is there a better way to do this?
	position.resize(3);
	linear_vel.resize(3);
	linear_acc.resize(3);
	
	quaternion.resize(4);	// Note: The trajectory classes check that all the input
	angular_vel.resize(4); // arguments are the same size to avoid errors. We can
	angular_acc.resize(4); // simply ignore that last value of the angular vel, acc
	
}
/******************** Get the type of trajectory that was generated ********************
std::string CartesianTrajectory::get_type()
{
	switch(this->type)
	{
		case 1:
			return "quintic polynomial";
		
		case 2:
			return "cubic spline";
		
		default:
			return "ERROR: unknown. How did this happen?";
	}
}

/******************** Get the desired pose, velocity, and acceleration for the given time ********************
void CartesianTrajectory::get_state(Eigen::Affine3f &pos, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	if(vel.size() != 6 || acc.size() != 6)						// Check input dimensions are sound
	{
		// ERROR: Vectors not of appropriate size.
	}
	
	// Call the relevant trajectory object that was created upon construction
	switch(this->type)
	{
		case 1:									// Quintic polynomial (2 points)
			this->quintic_position.get_state(this->position,			// Get the linear component
							  this->linear_vel,
							  this->linear_acc,
							  time);
						
			this->quintic_orientation.get_state(this->quaternion,		// Get the angular component
						  	     this->angular_vel,
							     this->angular_acc,
							     time);		
			break;
		
		case 2:									// Cubic spline (>2 points)
			this->cubic_position.get_state(this->position,			// Get the linear component
							this->linear_vel,
							this->linear_acc,
							time);
						
			this->cubic_orientation.get_state(this->quaternion,			// Get the angular component
						  	   this->angular_vel,
							   this->angular_acc,
							   time);	
			break;
			
		default:
			// ERROR: Trajectory type incorrectly specified. How did this happen?
			break;
	}
	
	// Combine the linear and angular components
	this->angle = 2*acos(this->quaternion(0));						// Angle encoded in the quaternion
	for(int i = 0; i < 3; i++)
	{
		this->axis(i) = this->quaternion(i+1);					// Vector component of the quaternion
		
		vel(i)   = linear_vel(i);
		vel(i+3) = angular_vel(i);
		acc(i)   = linear_acc(i);
		acc(i+3) = angular_acc(i);
	}
	this->axis.normalize();								// Normalise to get the axis
	pos = Eigen::Translation3f(this->position)*Eigen::AngleAxisf(this->angle, this->axis);
}
*/
