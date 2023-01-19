#include <QPSolver.h>
#include <SerialLink.h>

class SerialControlBase : public SerialLink,
                          public QPSolver
{
	public:
		SerialControlBase(const std::vector<RigidBody> &links,
		                  const std::vector<Joint> &joints,
                                  const float &controlFrequency);
						  
		// These virtual functions must be written in the derived class
		virtual Eigen::VectorXf move_endpoint(const Eigen::Matrix<float,6,1> &speed) = 0;   // Move endpoint at given velocity/acceleration
		
		virtual Eigen::VectorXf move_joints(const Eigen::VectorXf &speed) = 0;              // Move joints at given velocity/acceleration
				
		virtual Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,
		                                                   const Eigen::Matrix<float,6,1> &vel,
								   const Eigen::Matrix<float,6,1> &acc) = 0; // Solve Cartesian feedback control
														   
		virtual Eigen::VectorXf track_joint_trajectory(const Eigen::VectorXf &pos,
		                                               const Eigen::VectorXf &vel,
							       const Eigen::VectorXf &acc) = 0;     // Solve joint feedback control
													   
		// Set functions
		bool set_cartesian_gains(const float &stiffness,                                    // Used in Cartesian feedback control
		                         const float &damping);
					
		bool set_joint_gains(const float &proportional,                                     // As it says on the label
		                     const float &derivative);
							 
		bool set_max_acceleration(const float &accel);
							 
		bool set_redundant_task(const Eigen::VectorXf &task);                               // Used during Cartesian control
		
		void set_cartesian_gain_format(const Eigen::Matrix<float,6,6> &format)              // As it says on the label
		                              {this->gainFormat = format;}
									  
		
	private:
		bool redundantTaskSet = true;
	
		float hertz;                                                                        // Control frequency
		float dt;                                                                           // Inverse of frequency
		float maxAccel = 10.0;                                                              // Joint acceleration limit
		
		// Variables for joint control
		float kp = 1.0;                                                                     // Proportional gain
		float kd = 0.1;                                                                     // Derivative gain
		
		// Variables for Cartesian control
		Eigen::Matrix<float,6,6> gainFormat;                                                // Defines layout of matrix
		Eigen::Matrix<float,6,6> Kc;                                                        // Stiffness
		Eigen::Matrix<float,6,6> Dc;                                                        // Damping
		Eigen::VectorXf          redundantTask;                                             // Obvious
		
		// Internal functions
		float compute_joint_penalty_function(const unsigned int &jointNumber);
		float compute_joint_penalty_derivative(const unsigned int &jointNumber);
		
		Eigen::Matrix<double,6,1> get_pose_error(const Eigen::Isometry3f &desired,
		                                         const Eigen::Isometry3f &actual);          // As it says on the label
		
};                                                                                                  // Semicolon needed after class definition
