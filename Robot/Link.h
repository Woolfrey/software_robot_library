/*

	TO DO:
		- Test in combination with SerialLink class to see if forward
		  kinematics if working properly
		- Add some sort of error or feedback in constructor (Line 102)
	
	NOTES, THOUGHTS:
		- J: Do we need the offset property? It should be possible to
		     assign the staticTF so that it aligns with the physical
		     joint encoder.
*/

#ifndef LINK_H_								// If not yet defined...
#define LINK_H_								// ... include this header file, otherwise ignore

#include "Eigen/Core"								// Vectors and matrices
#include "Eigen/Geometry"							// Transforms and quaternions

class Link
{
	public:
		Link();							// Empty constructor

		// Constructor with basic kinematics
		Link(const Eigen::Affine3f &transform,			// Static transform from origin to endpoint
		     const bool &jointType,					// 1 = Revolute, 0 = Prismatic
		     const Eigen::Vector3f &axisOfActuation,			// Unit vector in local origin frame
		     const float position_limits[2],				// Min. and max. values on joint position
		     const float velocity_limits[2]);				// Min. and max. values on joint velocity

		// Constructor with kinematics and mass properties
		Link(const Eigen::Affine3f &transform,			// Static transform from origin to endpoint
		     const bool &jointType,					// 1 = Revolute, 0 = Prismatic
		     const Eigen::Vector3f &axisOfActuation,			// Unit vector in local origin frame
		     const float &linkMass,
		     const Eigen::Vector3f &linkCom,
		     const Eigen::VectorXf &linkInertia,
		     const float position_limits[2],				// Min. and max. values on joint position
		     const float velocity_limits[2]);				// Min. and max. values on joint velocity

		// Functions
		
		bool is_revolute() const {return this->isRevolute;}		// Returns 1 for Revolute joint,  0 for Prismatic joint

		Eigen::Vector3f get_axis() const {return this->axis;}	// Return the axis of actuation
		Eigen::Vector3f get_com() const {return this->com;}		// Return the location of the centre of mass
		float get_mass() const {return this->mass;}				// Return the mass
		Eigen::Matrix3f get_inertia() const {return this->inertia;}	// Return the link inertia
		
		Eigen::Affine3f get_pose(const float &pos);			// Get the link-to-link transform for given joint position
		
	private:
		
		// Kinematic properties
		Eigen::Affine3f staticTF;					// Transform from origin to end point
		
		// Dynamic properties
		// N.B. these should be declared w.r.t. the *previous*frame for cogency
		
		float mass;									// (kg)
		Eigen::Vector3f com;						// Center of mass [3x1] (m)
		Eigen::Matrix3f inertia;					// Inertia matrix [3x3] (kg*m^2)
		
		// Joint properties
		Eigen::Vector3f axis;						// Axis of actuation (unit vector)
		bool isRevolute;						// 1 = Revolute, 0 = Prismatic
		float offset;							// DO WE NEED THIS?
		float pLim[2];							// Min. and max. values on the joint position
		float vLim[2];							// Min. and max. values on the joint velocity
		float tLim[2];							// Min. and max. values on the joint torque/force
		
};										// Need a semicolon after a class declaration

// Empty constructor
Link::Link() :
	staticTF(Eigen::Translation3f(1,0,0)),				// Default values
	mass(1.0),
	com(Eigen::Vector3f(0.5,0,0)),
	inertia(Eigen::Matrix3f::Identity()),
	isRevolute(1),
	axis(Eigen::Vector3f(0,0,1)),
	offset(0),								// DO WE NEED THIS?
	pLim{-3.140, 3.140},
	vLim{-10.0, 10.0},
	tLim{-10.0, 10.0}
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The queen is their slave.
}

// Constructor with basic kinematics
Link::Link(	const Eigen::Affine3f &transform,
		const bool &jointType,
		const Eigen::Vector3f &axisOfActuation,
		const float position_limits[2],
		const float velocity_limits[2])
		:
		staticTF(transform),
		mass(1),							// Default value
		com(Eigen::Vector3f(0.5,0,0)),
		inertia(Eigen::Matrix3f::Identity()),				// Default value
		isRevolute(jointType),
		axis(axisOfActuation),
		offset(0),							// DO WE NEED THIS?
		pLim{position_limits[0], position_limits[1]},
		vLim{velocity_limits[0], velocity_limits[1]},
		tLim{-10.0, 10.0}						// Default value
{
	this->axis.normalize();						// Ensure unit norm

	if(this->pLim[0] >= this->pLim[1])
	{
			// Spit out an error of some sort?
	}
}

// Constructor with kinematics and mass properties
Link::Link(	const Eigen::Affine3f &transform,
		const bool &jointType,
		const Eigen::Vector3f &axisOfActuation,
		const float &linkMass,
		const Eigen::Vector3f &linkCom,
		const Eigen::VectorXf &linkInertia,
		const float position_limits[2], 
		const float velocity_limits[2])
		:
		staticTF(transform),
		mass(linkMass),							// Default value
		com(linkCom),
		isRevolute(jointType),
		axis(axisOfActuation),
		offset(0),							// DO WE NEED THIS?
		pLim{position_limits[0], position_limits[1]},
		vLim{velocity_limits[0], velocity_limits[1]},
		tLim{-10.0, 10.0}						// Default value
{
	this->axis.normalize();						// Ensure unit norm
    this->inertia << linkInertia(0), linkInertia(1), linkInertia(2),
					 linkInertia(1), linkInertia(3), linkInertia(4),
					 linkInertia(2), linkInertia(4), linkInertia(5); // Default value

	if(this->pLim[0] >= this->pLim[1])
	{
			// Spit out an error of some sort?
	}
}

// Get the transform from origin to endpoint for a given joint position
Eigen::Affine3f Link::get_pose(const float &pos)
{
	Eigen::Affine3f jointTF;						// Transform due to change in joint position
	
	if(this->isRevolute)	jointTF = Eigen::AngleAxisf(pos + this->offset, this->axis);		// Pure rotation about the axis
	else			jointTF = Eigen::Translation3f((pos + this->offset)*this->axis);	// Translate along the axis
	
	return this->staticTF*jointTF;
}
	
#endif
