#include <CartesianTrajectory.h>
#include <MultiPointTrajectory.h>

class SerialKinCtrl
{
	public:
		SerialKinCtrl() {}
		
		// Get Functions
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired,
						const Eigen::Isometry3f &actual);
		
		// Control Functions
		void move_to_position(const Eigen::VectorXf &position);	// Move joints to a desired position
		void move_to_pose(const Eigen::Isometry3f &pose);		// Move endpoint to a desired position, orientation
	
	protected:								// SerialDynControl has access to these
	
	private:
		CartesianTrajectory cartesianTrajectory;
		MultiPointTrajectory jointTrajectory;
	
};										// Semicolon needed after class declaration

/******************** Get the error between two poses for feedback control purposes ********************/
Eigen::VectorXf SerialKinCtrl::get_pose_error(const Eigen::Isometry3f &desired,
						const Eigen::Isometry3f &actual)
{
	Eigen::VectorXf error(6);							// Value to be returned
	
	error.head(3) = desired.translation() - actual.translation();		// Get the translation error
	
	// Is there are smarter/faster way to do this???
	Eigen::Isometry3f Re = desired*actual.inverse();				// Get the rotation error
	
	error.tail(3) = Eigen::Quaternionf(Re.rotation()).vec();			// Quaternion feedback
	
	return error;
}
