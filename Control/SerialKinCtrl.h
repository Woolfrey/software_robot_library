/*
*		A class for velocity control of a SerialLink object.
*/

#ifndef SERIALKINCTRL_H_
#define SERIALKINCTRL_H_

#include <array>										// std::array
#include <SerialLink.h>									// Custom robot class
#include <vector>										// std::vector

class SerialKinCtrl : public SerialLink
{
	public:
		SerialKinCtrl(const std::vector<RigidBody> links,
			const std::vector<Joint> joints,
			const float &controlFrequency);					// Constructor
		
		// Set Functions
		bool set_proportional_gain(const float &gain);				// Set the gain used in feedback control
		
		// Get Functions
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);
		Eigen::MatrixXf get_weighted_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W);
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual);				
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel);
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel, const Eigen::VectorXf &secondaryTask);
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel);
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel, const Eigen::VectorXf &secondary);
		Eigen::VectorXf get_joint_control(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel);
		
	private:
		double k = 1.0;								// Proportional gain
		float dt = 1/100;								// Discrete time step, for control purposes
		std::vector<std::array<float,2>> pLim;					// Position limits for all the joints
		std::vector<float> vLim;							// Velocity limits for all the joints
		std::vector<float> aLim;							// Acceleration limits for all the joints
				
		// Functions
//		bool scale_velocity_vector(Eigen::VectorXf &vector, const Eigen::VectorXf &reference);
		Eigen::MatrixXf get_joint_weighting();					// Used for joint limit avoidance
		Eigen::VectorXf singularity_avoidance(const float &scalar, const Eigen::MatrixXf &J);
		
};												// Semicolon needed after a class declaration

/////////////////////////////////////////////////////////////////////////////////////////////////////
//					Constructer						      //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialKinCtrl::SerialKinCtrl(const std::vector<RigidBody> links,
			const std::vector<Joint> joints,
			const float &controlFrequency)
			: SerialLink(links, joints)
			, dt(1/controlFrequency)
{
	// Not sure if I should "duplicate" the joint limits here, or just grab them from the Joint class... 
	// It makes the code a little neater, and the limits in the Control class can be altered
	
	// Resize vectors
	this->pLim.resize(this->n);
	this->vLim.resize(this->n);
	this->aLim.resize(this->n);
	
	// Transfer the values from the Joint classes
	for(int i = 0; i < this->n; i++)
	{
		this->joint[i].get_position_limits(this->pLim[i][0], this->pLim[i][1]);
		this->vLim[i] = this->joint[i].get_velocity_limit();
		this->aLim[i] = 2.0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			Set the gain used for proportional feedback control			      //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinCtrl::set_proportional_gain(const float &gain)
{
	if(gain == 0)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] set_proportional_gain() : Value cannot be zero. Gain not set." << std::endl;
		return false;
	}
	else if(gain < 0)
	{
		std::cerr << "[WARNING] [SERIALKINCTRL] set_proportional_gain() : Gain of " << gain << " cannot be negative! "
			<< "Setting the value positive to avoid problems." << std::endl;
		this->k = -1*gain;
		return true;
	}
	else
	{
		this->k = gain;
		return true;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Get the inverse of a matrix					      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinCtrl::get_inverse(const Eigen::MatrixXf &A)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // Get the SVD decomposition
	Eigen::MatrixXf V = SVD.matrixV();							// V matrix
	Eigen::MatrixXf U = SVD.matrixU();							// U matrix
	Eigen::VectorXf s = SVD.singularValues();						// Get the singular values
	Eigen::MatrixXf invA(A.cols(), A.rows()); invA.setZero();				// Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-04)	invA(i,k) += (V(i,j)*U(k,j))/s(j);	// Fast inverse
//				else			invA(i,k) += 0;			// Ignore singular directions
			}
		}
	}
	return invA;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			Get the weighted pseudoinverse of a matrix				      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinCtrl::get_weighted_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W)
{
	if(W.rows() != W.cols())
	{
		std::cerr << "[WARNING] [SERIALKINCTRL] get_weighted_inverse() : Weighting matrix must be square. "
			<< "Your input had " << W.rows() << " rows and " << W.cols() << " columns. "
			<< "Ignoring the weighting matrix..." << std::endl;
		return get_inverse(A);								// Return a null matrix
	}
	else if(A.cols() == W.rows())								// Overdetermined system
	{
		Eigen::MatrixXf invWAt = get_inverse(W)*A.transpose();
		return invWAt*get_inverse(A*invWAt);						// W^-1*A'*(A*W^-1*A')^-1
	}
	else if(W.cols() == A.rows())								// Underdetermined system
	{
		Eigen::MatrixXf AtW = A.transpose()*W;
		return get_inverse(AtW*A)*AtW;						// (A'*W*A)^-1*A'*W
	}
	else
	{
		std::cerr << "[WARNING] [SERIALKINCTRL] get_weighted_inverse() : Input matrices do not have compatible dimensions. "
			<< "The matrix A has " << A.rows() << " rows and " << A.cols() << " columns, and the matrix W has "
			<< W.rows() << " rows and " << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
		return get_inverse(A);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Get the error between two poses for feedback control				      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual)
{
	Eigen::VectorXf error(6);								// Value to be returned
	error.head(3) = desired.translation() - actual.translation();			// Get the translation error
	Eigen::Isometry3f Re = desired*actual.inverse();					// Get the rotation error (is there a better way?)
	error.tail(3) = Eigen::Quaternionf(Re.rotation()).vec();				// Quaternion feedback
	return error;	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Get the joint velocities to achieve the given Cartesian velocity		      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_cartesian_control(const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_cartesian_control() : Expected a 6x1 vector for the input argument "
			<< "but it had " << vel.size() << " elements." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		Eigen::MatrixXf J = get_jacobian();	
		if(this->n == 6)
		{	
			return get_inverse(J)*vel;
			// NOTE: NEED TO ADD IN JOINT LIMIT AVOIDANCE
		}
		else
		{
			Eigen::MatrixXf W = this->M + get_joint_weighting();
			Eigen::MatrixXf invJ = get_weighted_inverse(J, W);
			Eigen::VectorXf qdot_R = invJ*vel;
			
			// NEED TO SCALE qdot_R FOR FEASIBILITY HERE
			
			Eigen::VectorXf secondaryTask = singularity_avoidance(5.0, J);
			Eigen::VectorXf qdot_N = (Eigen::MatrixXf::Identity(this->n, this->n) - invJ*J)*get_inverse(W)*secondaryTask;
			
			// NEED TO SCALE qdot_N FOR FEASIBILITY HERE
			
			return qdot_R + qdot_N;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Solve the Cartesian velocity control with added redundant task		      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_cartesian_control(const Eigen::VectorXf &vel, const Eigen::VectorXf &secondaryTask)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_cartesian_control() : Expected a 6x1 vector for the input argument, "
			<< "but it had " << vel.size() << " elements." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else if(this->n < 7)
	{
		std::cerr << "[WARNING] [SERIALKINCTRL] get_cartesian_control() : This robot has " << this->n 
			<< " joints and is not redundant. The secondary task cannot be performed." << std::endl;
		return get_cartesian_control(vel);
	}
	else if(secondaryTask.size() != this->n)
	{
		std::cerr << "[WARNING] [SERIALKINCTRL] get_cartesian_control() : Expected a " << this->n << "x1 vector "
			<< " for the secondary task, but it had " << secondaryTask.size() << " elememnts." << std::endl;
		return get_cartesian_control(vel);
	}
	else
	{
		Eigen::MatrixXf J = get_jacobian();
		Eigen::MatrixXf W = this->M + get_joint_weighting();
		Eigen::MatrixXf invJ = get_weighted_inverse(J, W);
		Eigen::VectorXf qdot_R = invJ*vel;
		
		// NEED TO SCALE qdot_R FOR FEASIBILITY HERE
		
		Eigen::VectorXf qdot_N = (Eigen::MatrixXf::Identity(this->n, this->n) - invJ*J)*get_inverse(W)*secondaryTask;
		
		// NEED TO SCALE qdot_N FOR FEASIBILITY HERE
		
		return qdot_R + qdot_N;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			Compute Cartesian feedforward + feedback control			      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_cartesian_control() : Expected a 6x1 vector for the velocity argument, "
			<< "but it had " << vel.size() << " elements." << std::endl;
		
		return Eigen::VectorXf::Zero(this->n);					// Don't move
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());	// Pose error as a 6x1 vector
		return get_cartesian_control(vel + this->k*e);				// Feedforward + feedback control
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Compute Cartesian feedforward + feedback control with redundant task		      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel, const Eigen::VectorXf &secondary)
{
	if(vel.size()!= 6)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_cartesian_control() : Expected a 6x1 vector for the velocity argument, "
			<< "but it had " << vel.size() << " elements." << std::endl;
			
		return Eigen::VectorXf::Zero(this->n);
	}
	else if(secondary.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_cartesian_control() : Expected a " << this->n << "x1 vector for the "
			<< "secondary task argument, but it had " << secondary.size() << " elements." << std::endl;
		return Eigen::VectorXf::Zero(this->n);	
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());	
		return get_cartesian_control(vel + this->k*e, secondary);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Compute the feedfoward + feedback control to track a joint trajectoryW	      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::get_joint_control(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() != this->n || vel.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] get_joint_control() : This robot has " << this->n
			<< " joints, but the position argument had " << pos.size() << " elements "
			<< "and the velocity argument had " << vel.size() << " elements." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		return vel + this->k*(pos - this->q);
		// NOTE: NEED TO CHECK JOINT LIMITS
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			Compute a weighting matrix to avoid joint limits			      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinCtrl::get_joint_weighting()
{
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based scheme
	// for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	Eigen::MatrixXf W(this->n, this->n); W.setIdentity();			// Value to be returned
	float q, dpdq, range, upper, lower;
	for(int i = 0; i < this->n; i++)
	{
		upper = this->pLim[i][1] - this->q[i];				// Distance to upper limit
		lower = this->q[i] - this->pLim[i][0];				// Distance to lower limit
		range = this->pLim[i][1] - this->pLim[i][0];				// Difference between upper and lower
		dpdq = (range*range*(2*q - this->pLim[i][1] - this->pLim[i][0]))	// Partial derivative of penalty function
		      /(4*upper*upper*lower*lower);
			
		if(dpdq*this->qdot[i] > 0)						// If moving toward a limit...
		{
			W(i,i) = range*range/(4*upper*lower);				// Penalize joint motion
			
			if(W(i,i) < 1)
			{
				std::cout << "[ERROR] [SERIALKINCTRL] get_joint_limit_weighting() : "
				<< "Penalty function is less than 1! How did that happen???" << std::endl;
				std::cout << "qMin: " << this->pLim[i][0] << " q: " << this->q[i] << " qMax: " << this->pLim[i][1] << std::endl;
				W(i,i) = 1.0;
			}
		}
	}
	return W;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		Compute the gradient of manipulability to avoid singularities		      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinCtrl::singularity_avoidance(const float &scalar, const Eigen::MatrixXf &J)
{
	// Variables used in this scope
	Eigen::MatrixXf JJt = J*J.transpose();					// This makes things a little easier
	Eigen::MatrixXf invJ = J.transpose()*get_inverse(JJt);			// Pseudoinverse of Jacobian
	Eigen::MatrixXf dJ;								// Partial derivative of the Jacobian
	Eigen::VectorXf grad(this->n);						// Value to be returned
	float mu = sqrt(JJt.determinant());						// Actual measure of manipulability
	
	if(scalar <= 0)
	{
		std::cerr << "[ERROR] [SERIALKINCTRL] singularity_avoidance() : Scalar " << scalar << " must be positive." << std::endl;
		grad.setZero();							// Don't do anything!
	}
	else
	{
		grad[0] = 0.0;								// First joint doesn't affect manipulability
		
		for(int i = 1; i < this->n; i++)
		{
			dJ = get_partial_derivative(J, i);				// Get partial derivative w.r.t. ith joint
			grad[i] = scalar*mu*(dJ*invJ).trace();			// Gradient of manipulability
		}
	}
	return grad;
}

#endif
