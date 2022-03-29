    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                        A class for velocity control of a serial link robot                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIALKINCONTROL_H_
#define SERIALKINCONTROL_H_

#include <array>                                                                                   // std::array
#include <SerialLink.h>                                                                            // Custom robot class
#include <vector>                                                                                  // std::vector

class SerialKinControl : public SerialLink
{
	public:
		SerialKinControl(const std::vector<RigidBody> links,
                                const std::vector<Joint> joints,
                                const float &controlFrequency);                                    // Constructor
		
		// Set Functions
		bool set_proportional_gain(const float &gain);                                     // Set the gain used in feedback control
		
		// Get Functions
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);                             // Get the inverse of a matrix
		Eigen::MatrixXf get_weighted_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W);
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel);
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel);
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose,
		                                      const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		Eigen::VectorXf get_joint_control(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel);
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual);				
		
	private:
		double k = 1.0;                                                                    // Proportional gain
		float dt = 1/100;                                                                  // Discrete time step, for control purposes
		std::vector<std::array<float,2>> pLim;                                             // Position limits for all the joints
		std::vector<float> vLim;                                                           // Velocity limits for all the joints
		std::vector<float> aLim;                                                           // Acceleration limits for all the joints
		Eigen::VectorXf maxSpeed, minSpeed;                                                // Variable speed limits
				
		// Functions
		bool cap_joint_velocities(Eigen::VectorXf &vel,
					   const Eigen::VectorXf &upper,
					   const Eigen::VectorXf &lower);
		Eigen::MatrixXf get_joint_weighting();                                             // Used for joint limit avoidance
		Eigen::VectorXf singularity_avoidance(const float &scalar);                        // Returns gradient of manipulability
		void update_speed_limits();		
		
};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructer                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialKinControl::SerialKinControl(const std::vector<RigidBody> links,
                                   const std::vector<Joint> joints,
                                   const float &controlFrequency):
                                   SerialLink(links, joints),
                                   dt(1/controlFrequency),
                                   maxSpeed(Eigen::VectorXf::Zero(this->n)),
                                   minSpeed(Eigen::VectorXf::Zero(this->n))
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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gain used for proportional feedback control                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::set_proportional_gain(const float &gain)
{
	if(gain == 0)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] set_proportional_gain(): "
		          << "Value cannot be zero. Gain not set." << std::endl;
		return false;
	}
	else if(gain < 0)
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] set_proportional_gain(): "
                         << "Gain of " << gain << " cannot be negative. "
                         << "It has been automatically made positive..." << std::endl;
		this->k = -1*gain;
		return true;
	}
	else
	{
		this->k = gain;
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Get the inverse of a matrix                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinControl::get_inverse(const Eigen::MatrixXf &A)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);       // Get the SVD decomposition
	Eigen::MatrixXf V = SVD.matrixV();                                                         // V matrix
	Eigen::MatrixXf U = SVD.matrixU();                                                         // U matrix
	Eigen::VectorXf s = SVD.singularValues();                                                  // Get the singular values
	Eigen::MatrixXf invA(A.cols(), A.rows()); invA.setZero();                                  // Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-04)	invA(i,k) += (V(i,j)*U(k,j))/s(j);         // Fast inverse
//				else			invA(i,k) += 0;                            // Ignore singular directions
			}
		}
	}
	return invA;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the weighted pseudoinverse of a matrix                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinControl::get_weighted_inverse(const Eigen::MatrixXf &A, const Eigen::MatrixXf &W)
{
	if(W.rows() != W.cols())
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] get_weighted_inverse() : "
                         << "Weighting matrix must be square. Your input had " << W.rows() << " rows and "
                         << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
		return get_inverse(A);                                                             // Return a null matrix
	}
	else if(A.cols() == W.rows())                                                              // Overdetermined system
	{
		Eigen::MatrixXf invWAt = get_inverse(W)*A.transpose();
		return invWAt*get_inverse(A*invWAt);                                               // W^-1*A'*(A*W^-1*A')^-1
	}
	else if(W.cols() == A.rows())                                                              // Underdetermined system
	{
		Eigen::MatrixXf AtW = A.transpose()*W;
		return get_inverse(AtW*A)*AtW;                                                     // (A'*W*A)^-1*A'*W
	}
	else
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] get_weighted_inverse() : "
                         << "Input matrices do not have compatible dimensions. Matrix A has " << A.rows()
                         << " rows and " << A.cols() << " columns, and matrix W has " << W.rows()
                         << " rows and " << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
		return get_inverse(A);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Get the error between two poses for feedback control                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual)
{
	Eigen::VectorXf error(6);                                                                  // Value to be returned
	error.head(3) = desired.translation() - actual.translation();                              // Get the translation error
	Eigen::AngleAxisf temp(desired.rotation()*actual.rotation().inverse());
	double angle = temp.angle();
	if(angle > M_PI) angle = 2*M_PI - angle;                                                   // Take the shortest path
	error.tail(3) = angle*temp.axis();

	return error;	
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Get the joint velocities to achieve the given Cartesian velocity               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
                         << "Expected a 6x1 vector for the input argument "
                         << "but it was " << vel.size() << "x1." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		Eigen::VectorXf qdot;                                                              // Velocity vector to be solved
		
		if(this->n < 6)                                                                    // Underactuated robot
		{
			std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
			          << "Control for underactuated robots hasn't been programmed yet." << std::endl;
			qdot.Zero(this->n);                                                        // Don't move
		}
		if(this->n == 6)                                                                   // Fully actuated robot
		{	
			qdot = get_inverse(get_jacobian())*vel;                                    // Inverse kinematics
			update_speed_limits();                                                     // Update the variable joint speed limits
			cap_joint_velocities(qdot, this->minSpeed, this->maxSpeed);                // Ensure feasibility
		}
		else qdot = get_cartesian_control(vel, singularity_avoidance(5.0));                // Automatically perform singularity avoidance
		
		return qdot;                                                                       // Return the optimised velocity vector
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Solve the Cartesian velocity control with added redundant task               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::VectorXf &vel, const Eigen::VectorXf &secondaryTask)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
                         << "Expected a 6x1 vector for the input argument, "
                         << "but it was " << vel.size() << "x1." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else if(secondaryTask.size() < 7)
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] get_cartesian_control(): "
		          << "This robot has " << this->n << " joints and is not redundant. "
		          << "The secondary task cannot be performed." << std::endl;
		return get_cartesian_control(vel);
	}
	else if(secondaryTask.size() != this->n)
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] get_cartesian_control(): "
                         << "Expected a " << this->n << "x1 vector for the secondary task, but it was "
                         << secondaryTask.size() << "x1." << std::endl;
		return get_cartesian_control(vel);
	}
	else
	{
		update_speed_limits();                                                             // Update the variable joint speed limits
		
		Eigen::MatrixXf J = get_jacobian();                                                // As it says on the label
		Eigen::MatrixXf W = this->M + get_joint_weighting();                               // Weight by inertia, joint limit avoidance
		Eigen::MatrixXf invJ = get_weighted_inverse(J, W);                                 // Pseudoinverse Jacobian
		
		Eigen::VectorXf qdot_R = invJ*vel;                                                 // Range space task	
		cap_joint_velocities(qdot_R, this->minSpeed, this->maxSpeed);                      // Ensure kinematic feasibility

		Eigen::VectorXf qdot_N = (Eigen::MatrixXf::Identity(this->n,this->n) - invJ*J)*secondaryTask; // Null space task
		cap_joint_velocities(qdot_N, this->minSpeed - qdot_R, this->maxSpeed - qdot_R);    // Ensure kinematic feasibility
		
		// NOTE: qdot_R.transpose()*W*qdot_N = 0
		
		return qdot_R + qdot_N;                                                            // Return feasible solutions
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Compute Cartesian feedforward + feedback control                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::Isometry3f &pose, const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control() : "
                         << "Expected a 6x1 vector for the velocity argument, but it was " << vel.size() << "x1." << std::endl;
		
		return Eigen::VectorXf::Zero(this->n);                                             // Don't move
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                     // Pose error as a 6x1 vector
		return get_cartesian_control(vel + this->k*e);                                     // Feedforward + feedback control
	}
}

 /////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Compute Cartesian feedforward + feedback control with redundant task              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::Isometry3f &pose,
                                                        const Eigen::VectorXf &vel,
                                                        const Eigen::VectorXf &secondaryTask)
{
	if(vel.size()!= 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control() : "
                         << "Expected a 6x1 vector for the velocity argument, but it was " << vel.size() << "x1." << std::endl;
			
		return Eigen::VectorXf::Zero(this->n);
	}
	else if(secondaryTask.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control() : "
                         << "Expected a " << this->n << "x1 vector for the secondary task argument, but it was "
                         << secondaryTask.size() << "x1." << std::endl;
		return Eigen::VectorXf::Zero(this->n);	
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                     // As it says on the label	
		return get_cartesian_control(vel + this->k*e, secondaryTask);                      // Feedforward + feedback control
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //             Compute the feedfoward + feedback control to track a joint trajectory              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_joint_control(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() != this->n || vel.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_joint_control() : "
                         << "This robot has " << this->n << " joints, but the position argument had "
                         << pos.size() << " elements and the velocity argument had " << vel.size() << " elements." << std::endl;
		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		Eigen::VectorXf qdot = vel + this->k*(pos - this->q);                              // Feedforward + feedback control
		
		// Ensure speed limits are obeyed
		update_speed_limits();                                                             // Get new speed limits
		cap_joint_velocities(qdot, this->minSpeed, this->maxSpeed);
		return qdot;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Compute a weighting matrix to avoid joint limits                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinControl::get_joint_weighting()
{
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based scheme
	// for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	Eigen::MatrixXf W(this->n, this->n); W.setIdentity();                                      // Value to be returned
	float q, dpdq, range, upper, lower;
	for(int i = 0; i < this->n; i++)
	{
		upper = this->pLim[i][1] - this->q[i];                                             // Distance to upper limit
		lower = this->q[i] - this->pLim[i][0];                                             // Distance to lower limit
		range = this->pLim[i][1] - this->pLim[i][0];                                       // Difference between upper and lower
		dpdq = (range*range*(2*q - this->pLim[i][1] - this->pLim[i][0]))                   // Partial derivative of penalty function
		      /(4*upper*upper*lower*lower);
			
		if(dpdq*this->qdot[i] > 0)                                                         // If moving toward a limit...
		{
			W(i,i) = range*range/(4*upper*lower);                                      // Penalize joint motion
			
			if(W(i,i) < 1)
			{
				std::cout << "[ERROR] [SERIALKINCONTROL] get_joint_limit_weighting() : "
				<< "Penalty function is less than 1! How did that happen???" << std::endl;
				std::cout << "qMin: " << this->pLim[i][0] << " q: " << this->q[i] << " qMax: " << this->pLim[i][1] << std::endl;
				W(i,i) = 1.0;
			}
		}
	}
	return W;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the gradient of manipulability to avoid singularities                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::singularity_avoidance(const float &scalar)
{
	// Variables used in this scope
	Eigen::MatrixXf J = get_jacobian();                                                        // As it says on the label
	Eigen::MatrixXf JJt = J*J.transpose();                                                     // This makes things a little easier
	Eigen::MatrixXf invJ = J.transpose()*get_inverse(JJt);                                     // Pseudoinverse of Jacobian
	Eigen::MatrixXf dJ;                                                                        // Partial derivative of the Jacobian
	Eigen::VectorXf grad(this->n);                                                             // Value to be returned
	float mu = sqrt(JJt.determinant());                                                        // Actual measure of manipulability
	
	if(scalar <= 0)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] singularity_avoidance() : "
                         << "Scalar  of " << scalar << " must be positive." << std::endl;
		grad.setZero();                                                                    // Don't do anything!
	}
	else
	{
		grad[0] = 0.0;                                                                     // First joint doesn't affect manipulability
		
		for(int i = 1; i < this->n; i++)
		{
			dJ = get_partial_derivative(J, i);                                         // Get partial derivative w.r.t. ith joint
			grad[i] = scalar*mu*(dJ*invJ).trace();                                     // Gradient of manipulability
		}
	}
	return grad;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Compute the variable speed limits                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
void SerialKinControl::update_speed_limits()
{
	// Variables used in this scope
	float v, a, lower, upper;
	
	for(int i = 0; i < this->n; i++)
	{
		// Compute lower limit
		v = -this->vLim[i];
		a = -sqrt(2*this->aLim[i]*(this->q[i] - this->pLim[i][0]));
		
		lower = (this->pLim[i][0] - this->q[i])/this->dt;                                  // Assume position limit is largest
		if(lower < v) lower = v;                                                           // If velocity limit larger, override
		if(lower < a) lower = a;                                                           // If acceleration limit larger, override
		
		// Compute upper limit
		v = this->vLim[i];
		a = sqrt(2*this->aLim[i]*(this->pLim[i][1] - this->q[i]));
		
		upper = (this->pLim[i][1] - this->q[i])/this->dt;                                  // Assume position limit is smallest
		if(upper > v) upper = v;                                                           // If velocity limit is smaller, override
		if(upper > a) upper = a;                                                           // If acceleration limit is smaller, override
		
		this->minSpeed[i] = lower;                                                         // Update variable minimum speed
		this->maxSpeed[i] = upper;                                                         // Update variable maximum speed
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Cap any joint velocities that exceed the given speed limits                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::cap_joint_velocities(Eigen::VectorXf &vel,
					     const Eigen::VectorXf &upper,
					     const Eigen::VectorXf &lower)
{
	bool capped = false;
	for(int i = 0; i < this->n; i++)
	{
		if(vel[i] < lower[i])
		{
			vel[i] = 0.99*lower[i];
			capped = true;
		}
		else if(vel[i] > upper[i])
		{
			vel[i] = 0.99*upper[i];
			capped = true;
		}
	}
	return capped;
}

#endif
