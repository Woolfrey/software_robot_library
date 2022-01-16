/*
*	A minimum jerk trajectory between two orientations.
*/

#ifndef QUINTICROT_H_
#define QUINTICROT_H_

#include <Eigen/Geometry>								// Eigen::Quaternionf
#include <Quintic.h>									// Base class

class QuinticRot : public Quintic
{
	public:
		QuinticRot() : Quintic() {}						// Empty constructor
		
		QuinticRot(const Eigen::Quaternionf &startPoint,
				const Eigen::Quaternionf &endPoint,
				const float &startTime,
				const float &endTime);
		
		bool get_state(Eigen::Quaternionf &rot,
				Eigen::Vector3f &vel,
				Eigen::Vector3f &acc,
				const float &time);
	
	private:
		Eigen::Quaternionf Q0;							// Initial rotation
	
};											// Semicolon needed after a class declaration

/******************** Constructor from Quaternions *******************/
QuinticRot::QuinticRot(const Eigen::Quaternionf &startPoint,
			const Eigen::Quaternionf &endPoint,
			const float &startTime,
			const float &endTime)
			: Quintic()							// Empty for now
			, Q0(startPoint)						// Starting orientation
{
	// Variables in this scope
	Eigen::Quaternionf dQ;
	Eigen::AngleAxisf angleAxis;
	float angle;
	
	// We need to interpolate over the DIFFERENCE in orientation
	dQ = startPoint.conjugate()*endPoint;						// dQ = conj(Q0)*conj(Qf)
	dQ.normalize();								// Ensure unit norm
	
	// Get the angle and axis
	angleAxis = Eigen::AngleAxisf(dQ);						// Convert quaternion to angle and axis 
	angle = angleAxis.angle();							// Get the angle between orientations
	if(angle > M_PI) angle = 2*M_PI - angle;					// If greater than 180 degrees, take the shorter route
	
	// Set the inherited object starting from zero
	Quintic(Eigen::VectorXf::Zero(3), angle*angleAxis.axis(), startTime, endTime);
}

/******************** Get the desired state for the given time ********************/
bool QuinticRot::get_state(Eigen::Quaternionf &rot, Eigen::Vector3f &vel, Eigen::Vector3f &acc, const float &time)
{
	Eigen::VectorXf p(3), v(3), a(3);
	
	if(Quintic::get_state(p, v, a, time))
	{
		Eigen::AngleAxisf temp(p.norm(), p.normalized());			// Extract the angle and axis
		Eigen::Quaternionf dQ(temp);						// Convert to a quaternion
		
		rot = this->Q0*dQ;							// Q(t) = Q(0)*dQ(t)
		vel = v;
		acc = a;
		
		return true;
	}
	else
	{
		std::cerr << "[ERROR][QUINTICROT] get_state() : Could not obtain the desired state." << std::endl;
		
		rot = this->Q0;							// Remain at the start
		vel.setZero(); acc.setZero();						// Don't move
		
		return false;
	}
}

#endif
