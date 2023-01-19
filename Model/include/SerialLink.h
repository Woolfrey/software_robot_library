    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //           A class for computing kinematics and dynamics of a serial link mechanism             //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIAL_LINK_H_                                                                              // If not yet defined...
#define SERIAL_LINK_H_                                                                              // ... include this header file, otherwise ignore

#include <array>                                                                                    // std::array
#include <Eigen/Geometry>                                                                           // Transforms (Isometry3f)
#include <iostream>
#include <vector>                                                                                   // std::vector

#include <Joint.h>                                                                                  // Custom class representing moveable joints
#include <RigidBody.h>                                                                              // Custom class

class SerialLink
{
	public:
		// Constructors(s)
		SerialLink() {}                                                                     // Empty constructor
		
		SerialLink(const std::vector<RigidBody> &links,                                     // Proper constructor
		           const std::vector<Joint> &joints);

		// Set Functions
		void set_base_transform(const Eigen::Isometry3f &transform)                         // Define a new origin for the base
					{this->baseTF = transform;}
		
		void set_endpoint_offset(const Eigen::Isometry3f &transform)                        // Define a new endpoint offset
					 {this->endpointTF = transform;}
		
		void set_gravity_vector(const Eigen::Vector3f &gravity)                             // Set a new gravitational vector
				 	{this->gravityVector = gravity;}
		
		bool set_joint_state(const Eigen::VectorXf &pos,                                    // Update internal kinematics & dynamics
			             const Eigen::VectorXf &vel);                   
		
		// Get Functions
		bool is_valid() const {return this->isValid;}
		
		bool get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel);                   // Get the current internal joint state
		
		Eigen::Isometry3f get_endpoint_pose() const {return this->fkChain[this->n];}        // Get the pose of the endpoint
		
		Eigen::MatrixXf get_coriolis() const {return this->C;}                              // Get the Coriolis matrix
		
		Eigen::MatrixXf get_inertia() const {return this->M;}                               // Get inerita matrix in joint space
		
		Eigen::MatrixXf get_jacobian() {return this->J;}                                    // Get Jacobian to the endpoint
		
		Eigen::MatrixXf get_partial_derivative(const Eigen::MatrixXf &J,                    // Get partial derivative w.r.t a single joint
		                                       const int &jointNum);
		
		Eigen::MatrixXf get_time_derivative(const Eigen::MatrixXf &J);                      // Time derivative of a given Jacobian
		
		Eigen::VectorXf get_gravity_torque() const {return this->g;}                        // Get torque needed to oppose gravity
		
		Eigen::VectorXf get_joint_positions() const {return this->q;}                       // As it says on the label
		
		Eigen::VectorXf get_joint_velocities() const {return this->qdot;}                   // Return vector of n joint velocities
		
		float get_joint_position(const int &i) const {return this->q[i];}                   // Query a single joint position
		
		float get_joint_velocity(const int &i) const {return this->qdot[i];}                // Query a single joint velocity
		
		int get_number_of_joints() const {return this->n;}                                  // Returns the number of joints
		
		std::vector<std::array<float,2>> get_position_limits();                             // Get all joint limits as a single object
		
		std::vector<float> get_velocity_limits();                                           // Get all velocity limits as a single object
	
	protected:
		bool isValid = true;                                                                // Won't do calcs if this is false
		
		// Kinematic properties
		int n;                                                                              // Number of joints
		Eigen::Vector3f gravityVector = {0.0, 0.0, -9.81};                                  // Gravitational acceleration
		Eigen::VectorXf                q, qdot;                                             // Joint positions, velocities
		Eigen::Isometry3f              baseTF;                                              // Transform from global frame to origin of robot
		Eigen::Isometry3f              endpointTF;                                          // New endpoint offset (default identity)
		RigidBody                      base;                                                // The base "link"
		std::vector<Eigen::Isometry3f> fkChain;                                             // Transforms for each link
		std::vector<Eigen::Vector3f>   axis;                                                // Axis of actuation for each joint in base frame
		std::vector<Joint>             joint;                                               // Vector of joints connecting the links
		std::vector<RigidBody>         link;                                                // Vector of links on the robot which move
		Eigen::MatrixXf                J;                                                   // Jacobian to the endpoint

		// Dynamic properties
		Eigen::MatrixXf C;                                                                  // Coriolis matrix (nxn)
		Eigen::MatrixXf D;                                                                  // Damping matrix (nxn)
		Eigen::MatrixXf M;                                                                  // Inertia matrix (nxn)
		Eigen::VectorXf g;                                                                  // Gravitational torque (nx1)

		// Get Functions
		Eigen::MatrixXf get_jacobian(const Eigen::Vector3f &point, const int &numJoints);   // Get a Jacobian to a point on the robot
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
