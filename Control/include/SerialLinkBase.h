#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <KinematicTree.h>                                                                          // Computes the kinematics and dynamics
#include <Pose.h>                                                                                   // Position and orientation in 3D
#include <QPSolver.h>                                                                               // Control optimisation

class SerialLinkBase : public KinematicTree,
                       public QPSolver
{
	public:
		SerialLinkBase(const std::string &pathToURDF) : KinematicTree(pathToURDF);
		
		// This virtual functions must be defined in a derived class
		virtual Eigen::MatrixXf resolve_endpoint_motion(const Eigen::Matrix<float,6,1> &endpointMotion) = 0; // Solve the joint motion for given endpoint motion
		
		virtual Eigen::MatrixXf track_endpoint_trajectory(const Pose &pose,
							          const Eigen::Matrix<float,6,1> &velocity,
							          const Eigen::Matrix<float,6,1> &acceleration) = 0;
							  
		virtual Eigen::MatrixXf track_joint_trajectory(const Eigen::MatrixXf &position,
		                                               const Eigen::MatrixXf &velocity,
		                                               const Eigen::MatrixXf &acceleration) = 0;
		                                               
		// Methods
		bool set_cartesian_gains(const float &stiffness, const float &damping);
		                         
		bool set_cartesian_gain_format(const Eigen::Matrix<float,6,6> &format);
		                      
		bool set_joint_gains(const float &proportional, const float &derivative);
		                     
		bool set_max_joint_acceleration(const float &acceleration);
		
		bool set_redundant_task(const Eigen::VectorXf &task);
		                                       
	protected:
	
		// General properties
		
		bool redundantTaskSet = false;
		
		float frequency = 100;                                                              // Control frequency
		float maxAccel  = 10;                                                               // Maximum joint acceleration
		
		// Properties for Cartesian control
		
		Eigen::Matrix<float,6,6> gainFormat =(Eigen::MatrixXf(6,6) << 1, 0, 0,   0,   0,   0,
		                                                              0, 1, 0,   0,   0,   0,
		                                                              0, 0, 1,   0,   0,   0,
		                                                              0, 0, 0, 0.1,   0,   0,
		                                                              0, 0, 0,   0, 0.1,   0,
		                                                              0, 0, 0,   0,   0, 0.1).finished();
		
		Eigen::Matrix<float,6,6> K = 1.0*this->gainFormat;
		Eigen::Matrix<float,6,6> D = 0.1*this->gainFormat;
		
		// Properties for joint control
		
		float kp = 1.0;                                                                     // Proportional gain for joint control
		float kd = 0.1;                                                                     // Derivative gain for proportional control

		// Methods
		
		virtual bool compute_joint_limits(float &lower,
		                                  float &upper,
		                                  const unsigned int &jointNumber) = 0;
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
