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
		// Constructor(s)
		CartesianTrajectory() {}
		
		CartesianTrajectory(const std::vector<Eigen::Isometry3f> &poses,
				const std::vector<float> &times);
		
		// Get Functions
		void get_state(Eigen::Isometry3f &pose,			// Get the desired state for the given time
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
		
		// These variables are used to combine position & orientation
		Eigen::VectorXf pos, linearVel, linearAcc;			// Trajectory classes only accept VectorXf for translation
		Eigen::Vector3f angularVel, angularAcc;
		Eigen::Quaternionf quat;
		
};										// Semicolon needed after a class declaration

/******************** Constructor ********************/
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Isometry3f> &poses,
					const std::vector<float> &times)
					:
					n(poses.size()),
					pos(Eigen::VectorXf(3)),
					linearVel(Eigen::VectorXf(3)),
					linearAcc(Eigen::VectorXf(3))
{
	// Check the inputs are sound
	if(poses.size() != times.size())
	{
		std::cout << "ERROR: CartesianTrajectory::CartesianTrajectory() : Input vectors are not of equal length!" << std::endl;
		std::cout << "Pose vector has " << poses.size() << " elements and the time vector has " << times.size() << " elements.";
	}
	
	// Create minimum jerk trajectory
	if(this->n == 2)
	{
		this->quinticPosition = Quintic(poses[0].translation(),
						poses[1].translation(),
						times[0],
						times[1]);
						
		this->quinticOrientation = Quintic(Eigen::Quaternionf(poses[0].linear()),
						    Eigen::Quaternionf(poses[1].linear()),
						    times[0],
						    times[1]);
	}
	
	// Create minimum acceleration trajectory
	else if(this->n > 2)
	{
		std::vector<Eigen::VectorXf> position;
		std::vector<Eigen::Quaternionf> orientation;
		
		for(int i = 0; i < this->n; i++)
		{
			position.push_back(poses[i].translation());
			orientation.push_back(Eigen::Quaternionf(poses[i].linear()));
		}
		
		this->cubicPosition = CubicSpline(position, times);
		this->cubicOrientation = CubicSpline(orientation, times);
	}
	else std::cout << "ERROR: CartesianTrajectory::CartesianTrajectory() : At least 2 poses are needed to generate a trajectory!" << std::endl;
}

/******************** Get the desired state for the given time ********************/
void CartesianTrajectory::get_state(Eigen::Isometry3f &pose,
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time)
{
	if(vel.size() != 6 || acc.size() != 6)
	{
		std::cout << "ERROR: CartesianTrajectory::get_state() : Velocity and acceleration vectors must have 6 elements!" << std::endl;
		std::cout << "You input vel with " << vel.size() << " elements and acc with " << acc.size() << " elements.";
	}
	else
	{
		// Get the state based on the type of trajectory
		if(this->n == 2)
		{
			this->quinticPosition.get_state(this->pos, this->linearVel, this->linearAcc, time);
			this->quinticOrientation.get_state(this->quat, this->angularVel, this->angularAcc, time);
		}
		else
		{
			this->cubicPosition.get_state(this->pos, this->linearVel, this->linearAcc, time);
			this->cubicOrientation.get_state(this->quat, this->angularVel, this->angularAcc, time);
		}
		
		// Combine the information
		pose = Eigen::Translation3f(this->pos)*this->quat;		// Transform should be translation x rotation
		
		for(int i = 0; i < 3; i++)
		{
			vel[i] 	= this->linearVel[i];
			acc[i]		= this->linearAcc[i];
			vel[i+3]	= this->angularVel[i];
			acc[i+3]	= this->angularAcc[i];
		}
	}
}

#endif
