#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Eigen/Geometry>								// Eigen::Affine3f

class RigidBody
{
	public:
		RigidBody() {}								// Empty constructory
		
		RigidBody(const Eigen::Isometry3f &origin,				// Origin relative to some other frame
			const Eigen::Vector3f &centreOfMass,				// Center of mass rleative to origin
			const float &_mass,						// Mass (kg)
			const Eigen::VectorXf &momentOfInertia);			// Inertia (Nm^2)
			
		// Set Functions
		bool set_state(const Eigen::Isometry3f &origin,
				const Eigen::Vector3f &linearVel,
				const Eigen::Vector3f &angularVel);
		
		// Get Functions
		Eigen::Vector3f get_com() const {return this->com;}
		Eigen::Isometry3f get_pose() const {return this->pose;}
		Eigen::Matrix3f get_inertia() const {return this->inertia;}
		float get_mass() const {return this->mass;}
		
	private:
		// Kinematic Properties
		Eigen::Isometry3f pose;						// Pose of the origin to the local frame
		Eigen::Vector3f linearVel = {0.0 , 0.0 , 0.0};
		Eigen::Vector3f angularVel = {0.0 , 0.0 , 0.0};
				
		// Dynamic Properties
		Eigen::Matrix3f inertia;						// Moment of inertia at the center of mass
		Eigen::Matrix3f Idot;							// Time-derivative of inertia
		Eigen::Vector3f com = {0.5, 0.0, 0.0};				// Location of center of mass relative to local frame
		float mass = 1.0;							// Mass of the rigid body
		
};											// Semicolon needed after a class declaration

/******************** Full constructor with dynamic properties ********************/
RigidBody::RigidBody(const Eigen::Isometry3f &origin,
			const Eigen::Vector3f &centreOfMass,
			const float &_mass,
			const Eigen::VectorXf &momentOfInertia)
			: pose(origin)
			, com(centreOfMass)
			, mass(_mass)
{
	if(momentOfInertia.size() != 6)
	{
		std::cerr << "[ERROR] [RIGIDBODY] Constructor : Expected 6 elements for the momentOfInertia argument. "
			<< " Your input had " << momentOfInertia.size() << " elements." << std::endl;
	}
	else
	{
		this->inertia << momentOfInertia(0), momentOfInertia(1), momentOfInertia(2),
				  momentOfInertia(1), momentOfInertia(3), momentOfInertia(4),
				  momentOfInertia(2), momentOfInertia(4), momentOfInertia(5);
	}
				
	if(this->mass < 0)
	{
		std::cerr << "[WARNING] [RIGIDBODY] Constructor : Mass cannot negative. Your input was " << this->mass << "."
			<< " Ensuring the value is positive to avoid problems..." << std::endl;
		this->mass *= -1;
	}
}

#endif
