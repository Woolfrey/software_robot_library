#ifndef LINK_H_								// If not yet defined...
#define LINK_H_								// ... include this header file, otherwise ignore

#include "Eigen/Core"								// Vectors and matrices
#include "Eigen/Geometry"							// Transforms and quaternions

class Link
{
	public:
		Link();							// Empty constructor

		// Constructor with basic kinematics
		Link(const Eigen::Isometry3f &transform,			// Static transform from origin to endpoint
		     const bool &jointType,					// 1 = Revolute, 0 = Prismatic
		     const Eigen::Vector3f &jointAxis,			// Unit vector in local origin frame
		     const float positionLimits[2],				// Min. and max. values on joint position
		     const float speedLimit);					// Max value for a joint speed

		// Constructor with kinematics and mass properties
		Link(const Eigen::Isometry3f &transform,			// Static transform from origin to endpoint
		     const bool &jointType,					// 1 = Revolute, 0 = Prismatic
		     const Eigen::Vector3f &jointAxis,			// Unit vector in local origin frame
		     const float &linkMass,
		     const Eigen::Vector3f &linkCom,
		     const Eigen::VectorXf &linkInertia,
		     const float positionLimits[2],				// Min. and max. values on joint position
		     const float velocityLimit);				// Min. and max. values on joint velocity

		// Functions
		bool is_revolute() const {return this->isRevolute;}		// Returns 1 for Revolute joint, 0 for Prismatic joint
		bool is_prismatic() const {return !this->isRevolute;}	// Returns 0 for Revolute joint, 1 for Prismatic joint
		bool get_position_limits(float &lower, float &upper);	// Get the position limits on the attached joint
		Eigen::Isometry3f get_pose(const float &pos);			// Get the link-to-link transform for given joint position
		Eigen::Matrix3f get_inertia() const {return this->inertia;}	// Return the moment of inertia
		Eigen::Vector3f get_axis() const {return this->axis;}	// Return the axis of actuation
		Eigen::Vector3f get_com() const {return this->com;}		// Return the location of the centre of mass
		float get_mass() const {return this->mass;}			// Return the mass
		float get_torque_limit() const {return this->tLim;}		// Return the torque / force limit for the attached joint
		float get_velocity_limit() const {return this->vLim;}	// Return the speed limit for the attached joint
		
	private:
		
		// Kinematic properties
		Eigen::Isometry3f staticTF;					// Transform from origin to end point
		
		// Dynamic properties
		float mass = 1.0;						// (kg)
		Eigen::Vector3f com {0.5, 0.0, 0.0};				// Center of mass [3x1] (m)
		Eigen::Matrix3f inertia;					// Moment of inertia [3x3] (kg*m^2)
		
		// Joint properties
		bool isRevolute = true;					// 1 = Revolute, 0 = Prismatic
		Eigen::Vector3f axis {0.0, 0.0, 1.0};				// Axis of actuation (unit vector)
		float pLim[2] = {-3.14, 3.14};				// Min. and max. values on the joint position
		float vLim = 10.0;						// Absolute joint speed
		float tLim = 10.0;						// Absolute joint torque/force
				
};										// Need a semicolon after a class declaration

/******************** Default (i.e. empty) constructor ********************/
Link::Link()	: staticTF(Eigen::Translation3f(1,0,0))			// Default translation of 1m along x-axis
		, inertia(Eigen::Matrix3f::Identity())			// Not accurate for dynamic control
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Constructor with only kinematics ********************/
Link::Link(const Eigen::Isometry3f &transform,
	const bool &jointType,
	const Eigen::Vector3f &jointAxis,
	const float positionLimits[2],
	const float velocityLimit)
	: staticTF(transform)							// Assign the base transform
	, inertia(Eigen::Matrix3f::Identity())				// Default value (not accurate for dynamic control)
	, isRevolute(jointType)						// Assign the joint type
	, axis(jointAxis)							// Assign the joint axis
	, pLim{positionLimits[0], positionLimits[1]}				// Assign the joint position limits
	, vLim{velocityLimit}							// Assign the joint velocity limit
{
	this->axis.normalize();						// Ensure unit norm for the axis
}

/******************** Constructor with dynamic properties ********************/
Link::Link(const Eigen::Isometry3f &transform,
	const bool &jointType,
	const Eigen::Vector3f &jointAxis,
	const float &linkMass,
	const Eigen::Vector3f &linkCOM,
	const Eigen::VectorXf &linkInertia,
	const float positionLimits[2],
	const float speedLimit)
	: staticTF(transform)
	, mass(linkMass)							// Default value
	, com(linkCOM)
	, isRevolute(jointType)
	, axis(jointAxis)
	, pLim{positionLimits[0], positionLimits[1]}
	, vLim{speedLimit}
{
	this->axis.normalize();						// Ensure unit norm
	
// 	Inerta << 	xx, xy, xz,
//			xy, yy, yz,
//			xz, yz, zz
	this->inertia <<	linkInertia(0), linkInertia(1), linkInertia(2),
				linkInertia(1), linkInertia(3), linkInertia(4),
				linkInertia(2), linkInertia(4), linkInertia(5);
				
	// Lower position limit is greater than upper position limit
	if(this->pLim[0] > this->pLim[1])
	{
		std::cerr << "[WARNING][LINK] Constructor: Lower position limit of " << this->pLim[0]
			<< " is greater than the upper limit of " << this->pLim[1] << "."
			<< " Swapping their values to avoid problems..." << std::endl;
		
		float temp = this->pLim[1];
		this->pLim[1] = this->pLim[0];
		this->pLim[0] = temp;
	}
	// Lower position limit the same as upper position limit
	else if(this->pLim[0] == this->pLim[1])
	{
		std::cerr << "[WARNING][LINK] Constructor : Lower position limit of " << this->pLim[0]
			<< " is the same as the upper position limit " << this->pLim[1] << "." << std::endl;
	}
	
	// Speed limit negative
	if(this->vLim < 0)
	{
		std::cerr << "[WARNING][LINK] Constructor : Speed limit of " << this->vLim << " is negative."
			<< " It will be made positive to avoid problems..." << std::endl;
		this->vLim *= -1;
	}
	// Speed limit zero
	else if(this->vLim == 0)
	{
		std::cerr << "[ERROR][LINK] Constructor : Speed limit of attached joint is zero!" << std::endl;
	}
}

/******************** Get the pose of the link for a given joint position ********************/
Eigen::Isometry3f Link::get_pose(const float &pos)
{
	Eigen::Isometry3f jointTF;							// Transform due to change in joint position
	
	if(this->isRevolute)	jointTF = Eigen::AngleAxisf(pos, this->axis);		// Pure rotation about the axis
	else			jointTF = Eigen::Translation3f(pos*this->axis);	// Translate along the axis
	
	return this->staticTF*jointTF;
}

/******************** Get the position limits for the attached joint ********************/
bool Link::get_position_limits(float &lower, float &upper)
{
	// Can't think of a good error check / message here ¯\_(ツ)_/¯
	
	lower = this->pLim[0];
	upper = this->pLim[1];
	
	return true;
}

	
#endif
