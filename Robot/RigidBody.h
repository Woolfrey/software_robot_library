
#include <Eigen/Geometry>								// Eigen::Affine3f

class RigidBody
{
	public:
		RigidBody() {}								// Empty constructory
		
		RigidBody(const Eigen::Isometry3f &_pose) : pose(_pose) {}		// Constructor with just kinematics
		
		RigidBody(const Eigen::Isometry3f &_pose,				// Constructor with kinematics & dynamics
			const float &_mass,
			const Eigen::Vector3f &_com,
			const Eigen::Matrix3f &_inertia);
			
		// Set Functions
		bool set_state(const Eigen::Isometry3f &_pose,
				const Eigen::Vector3f &linearVel,
				const Eigen::Vector3f &angularVel);
		
		// Get Functions
		Eigen::Vector3f get_com() const {return this->com;}
		Eigen::Isometry3f get_pose() const {return this->pose;}
		Eigen::Matrix3f get_inertia() const {return this->inertia;}
		float get_mass() const {return this->mass;}
		
	private:
		Eigen::Isometry3f pose;						// Pose of the origin to the local frame
		Eigen::Matrix3f inertia;						// Moment of inertia at the center of mass
		Eigen::Matrix3f Idot;							// Time-derivative of inertia
		Eigen::Vector3f com = {0.5, 0.0, 0.0};				// Location of center of mass relative to local frame
		Eigen::Vector3f linearVel = {0.0 , 0.0 , 0.0};
		Eigen::Vector3f angularVel = {0.0 , 0.0 , 0.0};
		float mass = 1.0;							// Mass of the rigid body
		
};											// Semicolon needed after a class declaration

/******************** Full constructor with dynamic properties ********************/
RigidBody::RigidBody(const Eigen::Isometry3f &_pose,
			const float &_mass,
			const Eigen::Vector3f &_com,
			const Eigen::Matrix3f &_inertia)
			: pose(_pose)
			, mass(_mass)
			, com(_com)
			, inertia(_inertia)
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
