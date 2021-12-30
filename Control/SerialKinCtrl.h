#include <CartesianTrajectory.h>
#include <MultiPointTrajectory.h>
#include <SerialLink.h>

class SerialKinCtrl
{
	public:
		SerialKinCtrl(SerialLink *serial);
		
		// Set Functions
		bool set_joint_target(Eigen::VectorXf &target);
		bool set_joint_targets(const std::vector<Eigen::VectorXf> &targets, const std::vector<float> &times);
		bool set_proportional_gain(const float &gain);
		
		// Get Functions
		Eigen::VectorXf get_joint_control(const float &time);		// Control to track joint trajectory
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual);
	
	private:

		Eigen::VectorXf q_d, qdot_d, qddot_d;					// Desired joint position, velocity
	
		float kp = 1.0;							// Proportional gain
		
		int n;									// Number of joints in the robot

		SerialLink *robot;							// Use pointer to memory address?
		
		std::vector<std::array<float,2>> pLim;				// Array of position limits
		std::vector<std::array<float,2>> vLim;				// Array of velocity limits

		MultiPointTrajectory jointTrajectory;
	
};											// Semicolon needed after class declaration

/******************** Create a controller for the given SerialLink object ********************/
SerialKinCtrl::SerialKinCtrl(SerialLink *serial)
	:
	robot(serial),
	n(serial->get_number_of_joints()),
	q_d(Eigen::VectorXf(this->n)),
	qdot_d(Eigen::VectorXf(this->n)),
	qddot_d(Eigen::VectorXf(this->n))
{
	// Not sure if this is the best way to do this?
	this->pLim = this->robot->get_position_limits();				// Get the joint position limits
	this->vLim = this->robot->get_velocity_limits();				// Get the joint velocity limits
}

/******************** Set a desired joint configuration to move to ********************/
bool SerialKinCtrl::set_joint_target(Eigen::VectorXf &target)
{
	int n = this->robot->get_number_of_joints();						// Number of joints
	if(target.size() != n)
	{
		std::cout << "ERROR: SerialKinCtrl::set_joint_target() : Input vector has "
			<< target.size() << " elements, but the robot has " << n << " joints!"
			<< " Joint target has not been set." << std::endl;
			
		return false;
	}
	else
	{
		float dt;									// Change in time
		float dq;									// Change in position
		float startTime = 0.0;								// Have to declare this here or it won't compile???
		float endTime = 1.0;								// Default trajectory time
		
		for(int i = 0; i < n; i++)
		{
			// Check that the target is within joint limits
			if(target[i] <= this->pLim[i][0])	target[i] = 0.99*this->pLim[i][0];
			if(target[i] >= this->pLim[i][1])	target[i] = 0.99*this->pLim[i][1];
	
			// Compute the optimal time scaling for quintic polynomial. See:
			// Angeles, J. (2002). Fundamentals of robotic mechanical systems (Vol. 2).
			// New York: Springer-Verlag.
			dq = target[i] - this->robot->get_joint_position(i);			// Distance travelled
			if(dq < 0) dt = (15*dq)/(8*this->vLim[i][0]);				// Time to reach end at min. speed
			if(dq > 0) dt = (15*dq)/(8*this->vLim[i][1]);				// Time to reach end at max. speed
			if(dq != 0 && dt > endTime) endTime = dt;				// Set new end time for trajectory
		}
		
		// Set new trajectory object
		this->jointTrajectory = MultiPointTrajectory(this->robot->get_joint_positions(), target, startTime, endTime);
		
		return true;
	}
}

bool SerialKinCtrl::set_joint_targets(const std::vector<Eigen::VectorXf> &targets, const std::vector<float> &times)
{
	std::cout << "Worker bees can leave." << std::endl;
	std::cout << "Even drones can fly away." << std::endl;
	std::cout << "The Queen is their slave." << std::endl;
	
	return true;
}

/******************** Set proportional gains for the feedback control ********************/
bool SerialKinCtrl::set_proportional_gain(const float &gain)
{
	if(gain < 0)
	{
		std::cout << "ERROR: SerialKinCtrl::set_proportional_gain : Proportional gain cannot be negative!" << std::endl;
		return false;
	}
	else
	{
		this->kp = gain;
		return true;
	}
}

/******************** Compute the control to track the internal joint trajectory ********************/
Eigen::VectorXf SerialKinCtrl::get_joint_control(const float &time)
{
	this->jointTrajectory.get_state(this->q_d, this->qdot_d, this->qddot_d, time); 	// Get the desired state for the given time
	return this->qdot_d + this->kp*(this->q_d - this->robot->get_joint_positions());	// Feedforward + feedback
}

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
