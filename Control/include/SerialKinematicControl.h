
#include <SerialControlBase.h>

class SerialKinematicControl : public SerialControlBase
{
	public:
		SerialKinematicControl(const std::vector<RigidBody> &links,
		                       const std::vector<Joint>     &joints,
		                       const float &controlFrequency);
		
		// Functions derived from the base class
		Eigen::VectorXf move_endpoint(const Eigen::Matrix<float,6,1> &speed);               // Move endpoint at given acceleration
		
		Eigen::VectorXf move_joints(const Eigen::VectorXf &speed);                          // Move joints at given acceleration
				
		Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,           // Solve Cartesian feedback control
		                                           const Eigen::Matrix<float,6,1> &vel,
												   const Eigen::Matrix<float,6,1> &acc);
														   
		Eigen::VectorXf track_joint_trajectory(const Eigen::VectorXf &pos,                  // Solve joint feedback control
		                                       const Eigen::VectorXf &vel,
											   const Eigen::VectorXf &acc);
													   		
	private:
		bool get_velocity_bounds(float &lower, float &upper, const int &jointNumber);		
};                                                                                                  // Semicolon needed after a class declaration
