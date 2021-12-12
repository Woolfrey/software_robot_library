
class SerialKinCtrl
{
	public:
		SerialKinCtrl();
		
		// Get Functions
		Eigen::VectorXf get_pose_error(const Eigen::Affine3f &desired);
		
		// Control Functions
		void move_to_position(const Eigen::VectorXf &position);	// Move joints to a desired position
		void move_to_pose(const Eigen::Affine3f &pose);		// Move endpoint to a desired position, orientation
	
	protected:								// SerialDynControl has access to these
	
	private:
	
};										// Semicolon needed after class declaration

SerialKinCtrl::SerialKinCtrl()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
