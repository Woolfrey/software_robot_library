#include <SerialKinCtrl.h>

class SerialDynCtrl : public SerialKinCtrl
{
	public:
		SerialDynCtrl(const SerialLink &serial);				// Constructor
		
		// Set Functions
		bool set_gains(const float &proportional, const float &derivative);
		
		// Get Functions
//virtual	Eigen::VectorXf get_joint_control(const float &time);		// Get the control to track the joint trajectory
		
	private:
		float kp = 100;							// Proportional gain
		float kd = 20;								// Derivative gain >= 2*sqrt(kp)
		
		Eigen::MatrixXf K, D;							// Cartesian stiffness and damping
		
};											// Semicolon needed after class declaration

/******************** Create a dynamic-level controller for a SerialLink object ********************
SerialDynCtrl::SerialDynCtrl(const SerialLink &serial) : SerialKinCtrl(serial)
{
	this->K.resize(6,6); this->K.setIdentity()*25;				// Set default values for Cartesian stiffness
	this->D.resize(6,6); this->D.setIdentity()*10;				// Cartesian damping >= 2*sqrt(K)
}

/******************** Set the gains for joint feedback control ********************
bool SerialDynCtrl::set_gains(const float &proportional, const float &derivative)
{
	// Check that the inputs are sound
	if(proportional < 0 || derivative < 0)
	{
		std::cout << "ERROR SerialDynCtrl::set_gains() : Feedback gains cannot be negative!"
			<< " Input values were " << proportional << " and " << derivative << "." << std::endl;
			
		return false;
	}
	else
	{
		// Warn the user if underdamped
		if(derivative < 2*sqrt(proportional))
		{
			std::cout << "WARNING SerialDynCtrl::set_gains() : The derviative gain is too low and the system will be underdamped."
				<< " Your input: " << derivative << " Ideal input: > " << 2*sqrt(proportional) "." << std::endl;
		}
		
		this->kp = proportional;
		this->kd = derivative;
		return true;
	}	
}

/******************** Get the joint torque to track a trajectory ********************
Eigen::VectorXf SerialDynCtrl::get_joint_control(const float &time)
{
	Eigen::VectorXf q_d(this->n), qdot_d(this->n), qddot_d(this->n);			// Desired joint state
	this->jointTrajectory.get_state(q_d, qdot_d, qddot_d);				// Get the desired state from the trajectory object
	
	Eigen::VectorXf q	= this->robot.get_joint_position();				// Actual joint positions
	Eigen::VectorXf qdot	= this->robot.get_joint_velocities();				// Actual joint velocities
	Eigen::VectorXf qddot	= qddot_d + this->kd*(qdot_d - qdot) + this->kp*(q_d - q);	// Feedforward + feedback control
	
	// tau = M*qddot + C*qdot + g
	return this->robot.get_inertia()*qddot + this->robot.get_coriolis*qdot + this->robot.get_gravity_torque();				
}
*/
