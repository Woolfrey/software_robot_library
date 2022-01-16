/*
*	A trajectory generator in Cartesian space.
*/

#ifndef CARTESIAN_TRAJECTORY_H_
#define CARTESIAN_TRAJECTORY_H_

#include <CubicRot.h>
#include <QuinticRot.h>

class CartesianTrajectory
{
	public:
		CartesianTrajectory() {}
		
		CartesianTrajectory(const Eigen::Isometry3f &startPose,
					const Eigen::Isometry3f &endPose,
					const float &startTime,
					const float &endTime);
		
		CartesianTrajectory(const std::vector<Eigen::Isometry3f> &poses, const std::vector<float> &times);
		
		bool get_state(Eigen::Isometry3f &pose, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time);
		
	private:
		bool isNotValid = true;
		CubicSpline translationTrajectory;
		CubicRot rotationTrajectory;
		Eigen::Isometry3f T0;
				
};										// Semicolon needed after a class declaration

/******************** Constructor with 2 points ********************/
CartesianTrajectory::CartesianTrajectory(const Eigen::Isometry3f &startPose,
					const Eigen::Isometry3f &endPose,
					const float &startTime,
					const float &endTime)
					: T0(startPose)
{
	//Check the times are in correct order
	double t1 = startTime;
	double t2 = endTime;
	if(t1 > t2)
	{
		std::cerr 	<< "[WARNING][CARTESIANTRAJECTORY] Constructor : Start time of "
				<< t1 << " is greater than end time of " << t2 << ". Swapping values..." << std::endl;
		double temp = t1;
		t1 = t2;
		t2 = temp;
	}
	
	std::vector<float> times;
	times.push_back(t1);
	times.push_back(t2);
	
	std::vector<Eigen::VectorXf> pos;
	pos.push_back(startPose.translation());
	pos.push_back(endPose.translation());
	this->translationTrajectory = CubicSpline(pos, times);
	
	std::vector<Eigen::Quaternionf> rot;
	rot.push_back(Eigen::Quaternionf(startPose.rotation()));
	rot.push_back(Eigen::Quaternionf(endPose.rotation()));
	this->rotationTrajectory = CubicRot(rot, times);
}

/******************** Constructor with multiple points ********************/
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Isometry3f> &poses,
					const std::vector<float> &times)
					: T0(poses[0])
{
	// Check the input arguments are of the same length
	if(poses.size() != times.size())
	{
		std::cerr << "[ERROR][CARTESIANTRAJECTORY] Constructor : Input arguments are not of equal length." << std::endl;
		std::cerr << " poses: " << poses.size() << " times: " << times.size() << std::endl;
	}
	else
	{
		this->isNotValid = false;
		
		// Check that the times are in ascending order
		for(int i = 0; i < times.size() - 1; i++)
		{
			if(times[i] > times[i+1])
			{
				std::cerr << "[ERROR][CARTESIANTRAJECTORY] Constructor : Times are not in ascending order." << std::endl;
				this->isNotValid = true;
				break;
			}
		}
		
		if(!this->isNotValid) // this is valid
		{		
			std::vector<Eigen::VectorXf> pos;
			std::vector<Eigen::Quaternionf> rot;
			for(int i = 0; i < poses.size(); i++)
			{
				pos.push_back(poses[i].translation());
				rot.push_back(Eigen::Quaternionf(poses[i].rotation()));
			}
			
			this->translationTrajectory = CubicSpline(pos, times);
			this->rotationTrajectory = CubicRot(rot, times);
		}
	}
}

/******************** Get the desired state for the given time ********************/
bool CartesianTrajectory::get_state(Eigen::Isometry3f &pose, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	// Variables used in this scope
	Eigen::VectorXf pos(3), linearVel(3), linearAcc(3);
	Eigen::Vector3f angularVel(3), angularAcc(3);
	Eigen::Quaternionf rot;
	
	if(vel.size() != 6 || acc.size() != 6)
	{
		std::cerr << "[ERROR][CARTESIANTRAJECTORY] get_state() : Velocity and acceleration vectors must have 6 elements." << std::endl;
		std::cerr << " vel: " << vel.size() << " acc: " << acc.size() << std::endl;
			
		pose = this->T0;						// Remain at the start
		vel.setZero(); acc.setZero();					// Don't move
		
		return false;
	}
	else if(this->translationTrajectory.get_state(pos, linearVel, linearAcc, time)
	     && this->rotationTrajectory.get_state(rot, angularVel, angularAcc, time))
	{
		pose = Eigen::Translation3f(pos)*rot;				// Transform should be translation x rotation
		
		for(int i = 0; i < 3; i++)
		{
			vel[i]   = linearVel[i];
			vel[i+3] = angularVel[i];
			acc[i]   = linearAcc[i];
			acc[i+3] = angularAcc[i];
		}
		return true;
	}
	else
	{
		std::cerr << "[ERROR][CARTESIANTRAJECTORY] get_state() : Could not obtain the state." << std::endl;
		
		pose = this->T0;						// Remain at the start
		vel.setZero(); acc.setZero();					// Don't move
		
		return false;
	}
}

#endif
