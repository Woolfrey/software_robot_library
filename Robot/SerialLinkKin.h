#ifndef SERIAL_LINK_KIN_H_
#define SERIAL_LINK_KIN_H_

#include <Eigen/Geometry>
#include <Link.h>								// Custom Link class
#include <vector>								// std::vector

class SerialLinkKin
{
	public:
		// Constructor
		SerialLinkKin(const std::vector<Link> &links,
				const Eigen::Affine3f &baseTransform,
				const Eigen::Affine3f &endpointTransform);
		
		// Set Functions
		bool update_state(const Eigen::VectorXf &_q, const Eigen::VectorXf &_qdot);	// Update internal kinematic properties	
		void set_endpoint(const Eigen::Affine3f &transform);				// Set a new offset for the endpoint
		
		// Get Functions
		int get_number_of_joints() const{return this->n;}				// Returns the number of joints
		Eigen::Affine3f get_endpoint() const {return this->fkChain[this->n];}	// Get the pose of the endpoint
		Eigen::MatrixXf get_jacobian();						// Get the Jacobian for the current joint state
		void get_joint_state(Eigen::VectorXf &_q, Eigen::VectorXf &_qdot);		// Get joint positions and velocities
		
	
	protected: 								// SerialLinkDyn has access to these
		// Fixed kinematic properties
		int n;								// Number of joints
		Eigen::VectorXf q, qdot;					// Joint position and velocity
		Eigen::VectorXf baseTwist;					// Linear and angular velocity of the base
		Eigen::Affine3f baseTF;					// Pose of base relative to global frame
		Eigen::Affine3f finalTF;					// Transform from final joint to endpoint
		Eigen::Affine3f endpointTF;					// Offset from end of the kinematic chain
		std::vector<Link> link;					// Vector of Link objects
		
		// Variable kinematic properties
		std::vector<Eigen::Affine3f> fkChain;				// Transform chain for each link/joint
		std::vector<Eigen::Vector3f> a;				// Axis of actuation for each joint
		std::vector<Eigen::Vector3f> r;				// Translation vector from joint to endpoint
		
		// Internal functions
		void update_forward_kinematics();
		void update_axis_and_distance();				// Compute kinematic properties of mechanism for current state
		
	private:
		
};										// Semicolon needed after class declaration

/******************** Basic constructor ********************/
SerialLinkKin::SerialLinkKin(const std::vector<Link> &links,
			const Eigen::Affine3f &baseTransform,
			const Eigen::Affine3f &endpointTransform)
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

#endif
