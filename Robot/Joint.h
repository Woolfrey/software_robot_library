
#include <Eigen/Geometry>							// Eigen::Isometry3f, Eigen::Vector3f

class Joint
{
	public:
		Joint() {}							// Empty constructor
		
		bool is_revolute() {return this->isRevolute;}			// Returns true if revolute
		bool is_prismatic() {return !this->isRevolute;}		// Returns true if NOT revolute
		
		Eigen::Isometry3f get_pose(const float &pos);			// Get the pose from the displacement of this joint
		
	private:
		bool isRevolute = true;
		Eigen::Vector3f axis = {0,0,1};				// Axis of actuation
		float damping = 1.0;						// Viscous friction for the joint
		float pLim[2] = {-M_PI, M_PI};				// Position limits
		float vLim = 10;						// Speed limits
		float tLim = 10;						// Torque / force limits

};										// Semicolon needed after a class declaration

Eigen::Isometry3f Joint::get_pose(const float &pos)
{
	Eigen::Isometry3f T;							// Need to pre-declare object here to convert from AngleAxis below
	if(this->isRevolute) 	T = Eigen::AngleAxisf(pos, this->axis);
	else			T = Eigen::Translation3f(pos*this->axis);
	return T;
}

