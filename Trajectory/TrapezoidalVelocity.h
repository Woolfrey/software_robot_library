#ifndef TRAPEZOIDAL_VELOCITY_H_
#define TRAPEZOIDAL_VELOCITY_H_

#include <Eigen/Geometry>						// Eigen::Vector, Eigen::Matrix, Eigen::Quaternion

class TrapezoidalVelocity
{
	public:
		// Empty constructor
		TrapezoidalVelocity() {}
		
		// Constructor for trajectory over real numbers
		TrapezoidalVelocity(const Eigen::VectorXf &startPoint,
				const Eigen::VectorXf &endPoint,
				const float &velocity,
				const float &acceleration);
		
		// Constructor for orientation		
		TrapezoidalVelocity(const Eigen::Quaternionf &startPoint,
				const Eigen::Quaternionf &endPoint,
				const float &velocity,
				const float &acceleration);
		
		// Get state for trajectory over real numbers
		void get_state(Eigen::VectorXf &pos,
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
		
		// Get state for trajectory over orientation
		void get_state(Eigen::Quaternionf &quat,
				Eigen::Vector3f &vel,
				Eigen::Vector3f &acc,
				const float &time);				
				
	private:
		int n;							// Number of dimensions
		
		float vMax, aMax, startTime, endTime;
};									// Semicolon needed after class declaration

#endif
