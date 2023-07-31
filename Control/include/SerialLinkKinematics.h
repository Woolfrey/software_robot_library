
#include <SerialLinkBase.h>

class SerialLinkKinematics : public SerialLinkBase
{
	public:
		SerialKinematicControl(const std::string &pathToURDF) : SerialLinkBase(pathToURDF);
		
		// Functions derived from the base class
		
		Eigen::MatrixXf resolve_endpoint_motion(const Eigen::Matrix<float,6,1> &endPointMotion); // Solve the joint motion to execute a given endpoint motion
		
		Eigen::MatrixXf track_endpoint_trajectory(const Pose &desiredPose,
							  const Eigen::Matrix<float,6,1> &desiredVel,
							  const Eigen::Matrix<float,6,1> &desiredAcc);
							  
		Eigen::MatrixXf track_joint_trajectory(const Eigen::MatrixXf &desiredPos,
		                                       const Eigen::MatrixXf &desiredVel,
		                                       const Eigen::MatrixXf &desiredAcc);
													   		
	protected:
	
		bool compute_joint_limits(float &lower, float &upper, const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration
