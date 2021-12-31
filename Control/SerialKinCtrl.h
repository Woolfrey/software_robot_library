#include <CartesianTrajectory.h>							// Custom trajectory class
#include <MultiPointTrajectory.h>							// Custom trajectory class
#include <SerialLink.h>								// Serial link robot class

class SerialKinCtrl
{
	public:
		SerialKinCtrl(const SerialLink &serial);				// Constructor
		
		// Set Functions
		bool set_joint_target(Eigen::VectorXf &target);
		bool set_joint_targets(const std::vector<Eigen::VectorXf> &targets, const std::vector<float> &times);
		bool set_proportional_gain(const float &gain);
		
		// Get Functions
		Eigen::VectorXf get_joint_control(const float &time);		// Control to track joint trajectory
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual);

		// Get functions
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);		// Get the inverse of the given matrix
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W);
	
	private:

		Eigen::VectorXf q_d, qdot_d, qddot_d;					// Desired joint position, velocity
	
		float kp = 1.0;							// Proportional gain
		
		int n;									// Number of joints in the robot

		SerialLink robot;							// Use pointer to memory address?
		
		std::vector<std::array<float,2>> pLim;				// Array of position limits
		std::vector<std::array<float,2>> vLim;				// Array of velocity limits

		MultiPointTrajectory jointTrajectory;					// Joint trajectory object
	
};											// Semicolon needed after class declaration

/******************** Create a controller for the given SerialLink object ********************/
SerialKinCtrl::SerialKinCtrl(const SerialLink &serial)
	:
	robot(serial),									// Assign the serial link object
	n(robot.get_number_of_joints()),						// Number of joints
	pLim(robot.get_position_limits()),						// Not sure if this is the best way to
	vLim(robot.get_velocity_limits())						// work with the joint limits...
{
	this->q_d.resize(this->n);
	this->qdot_d.resize(this->n);
	this->qddot_d.resize(this->n);
}

/******************** Set a desired joint configuration to move to ********************/
bool SerialKinCtrl::set_joint_target(Eigen::VectorXf &target)
{
	if(target.size() != this->n)
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
			dq = target[i] - this->robot.get_joint_position(i);			// Distance travelled
			if(dq < 0) dt = (15*dq)/(8*this->vLim[i][0]);				// Time to reach end at min. speed
			if(dq > 0) dt = (15*dq)/(8*this->vLim[i][1]);				// Time to reach end at max. speed
			if(dq != 0 && dt > endTime) endTime = dt;				// Set new end time for trajectory
		}
		
		// Set new trajectory object
		this->jointTrajectory = MultiPointTrajectory(this->robot.get_joint_positions(), target, startTime, endTime);
		
		return true;
	}
}

/******************** Set several waypoints for the joints to move through ********************/
bool SerialKinCtrl::set_joint_targets(const std::vector<Eigen::VectorXf> &targets, const std::vector<float> &times)
{
	std::cout << "\nWorker bees can leave." << std::endl;
	std::cout << "Even drones can fly away." << std::endl;
	std::cout << "The Queen is their slave.\n" << std::endl;
	std::cout << "\n (I need to program in optimal time scaling for this still) \n" << std::endl;
	
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
	return this->qdot_d + this->kp*(this->q_d - this->robot.get_joint_positions());	// Feedforward + feedback
}

/******************** Get the error between two poses for feedback control purposes ********************/
Eigen::VectorXf SerialKinCtrl::get_pose_error(const Eigen::Isometry3f &desired,
						const Eigen::Isometry3f &actual)
{
	Eigen::VectorXf error(6);								// Value to be returned
	
	error.head(3) = desired.translation() - actual.translation();			// Get the translation error
	
	// Is there are smarter/faster way to do this???
	Eigen::Isometry3f Re = desired*actual.inverse();					// Get the rotation error
	
	error.tail(3) = Eigen::Quaternionf(Re.rotation()).vec();				// Quaternion feedback
	
	return error;
}

/******************** Get the inverse of a matrix, add damping as necessary ********************/
Eigen::MatrixXf SerialKinCtrl::get_inverse(const Eigen::MatrixXf &A)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // Get the SVD decomposition
	Eigen::MatrixXf V = SVD.matrixV();							// V-matrix
	Eigen::VectorXf s = SVD.singularValues();						// Get the singular values
	Eigen::MatrixXf invA(A.cols(), A.rows()); invA.setZero();				// Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < A.rows(); j++)
		{
			if(s(j) > 1e-10)	invA(i,j) += V(i,j)/s(j);			// Left half of the inverse
//			else			invA(i,j) += V(i,j)*0				// Ignore singular directions
		}
	}
	
	return invA*SVD.matrixU().transpose();
}

/******************** Get the weighted inverse of a matrix ********************/
Eigen::MatrixXf SerialKinCtrl::get_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W)
{
	// Matrix isn't square, return zero
	if(W.cols() != W.rows())
	{
		std::cout << "ERROR: SerialKinCtrl::get_inverse() : Cannot compute the weighted inverse!"
			<< " Weighting matrix is " << W.rows() << "x" << W.cols() << ", but it must be square." << std::endl;
			
		return Eigen::MatrixXf::Zero(A.cols(), A.rows());
	}
	// Columns don't match rows, return zero
	else if(A.cols() != W.rows())
	{
		std::cout << "ERROR: SerialKinCtrl::get_inverse() :: Cannot compute the weighted inverse!"
			<< " Input matrix has " << A.cols() << " columns and weighting matrix has " << W.rows()
			<< " rows, but they must be the same." << std::endl;
		return Eigen::MatrixXf::Zero(A.cols(), A.rows());
	}
	else
	{	Eigen::MatrixXf invW = get_inverse(W);					// Get the inverse of the weighting matrix
		Eigen::MatrixXf invWAt = invW*A.transpose();					// This makes calcs a little faster
		return invWAt*get_inverse(A*invWAt);						// Return the inverse
	}
}
