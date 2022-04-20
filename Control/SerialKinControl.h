    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                        A class for velocity control of a serial link robot                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIALKINCONTROL_H_
#define SERIALKINCONTROL_H_

#include <algorithm>                                                                                // std::min, std::max
#include <array>                                                                                    // std::array
#include <SerialLink.h>                                                                             // Custom robot class
#include <vector>                                                                                   // std::vector

class SerialKinControl : public SerialLink
{
	public:
		SerialKinControl(const std::vector<RigidBody> links,
                                const std::vector<Joint> joints,
                                const float &controlFrequency);                                     // Constructor
		
		// Set Functions
		bool set_proportional_gain(const float &gain);                                      // Set the gain used in feedback control
		
		// Get Functions
		
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);                              // Get the inverse of a matrix
		
		Eigen::MatrixXf get_weighted_inverse(const Eigen::MatrixXf &A,                      // Get the weighted inverse of a matrix
						      const Eigen::MatrixXf &W);
		
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel);                 
		
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		                                      
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose,
                                                     const Eigen::VectorXf &vel);
		
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose,
		                                      const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		                                      
		Eigen::VectorXf get_joint_control(const Eigen::VectorXf &pos,
						   const Eigen::VectorXf &vel);
		
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired,
						const Eigen::Isometry3f &actual);				
		
	private:
		// Functions
		bool limit_joint_velocity(float &qdot, const int &i);                               // Ensure kinematic feasiblity of joint control
		
		bool primary_task_is_ok(const Eigen::VectorXf &task);                               // Ensure input vector is 6x1
		
		bool secondary_task_is_ok(const Eigen::VectorXf &task);                             // Ensure input vector is nx1
		
		Eigen::MatrixXf get_joint_weighting();                                              // Used for joint limit avoidance
		
		Eigen::VectorXf rmrc(const Eigen::VectorXf &xdot,                                   // Cartesian velocity control algorithm
                                    const Eigen::VectorXf &qdot_d);
		
		Eigen::VectorXf singularity_avoidance(const float &scalar);                         // Returns gradient of manipulability
		
		// Variables
		double k = 1.0;                                                                     // Proportional gain
		float dt = 1/100;                                                                   // Discrete time step, for control purposes
		std::vector<std::array<float,2>> pLim;                                              // Position limits for all the joints
		std::vector<float> vLim;                                                            // Velocity limits for all the joints
		std::vector<float> aLim;                                                            // Acceleration limits for all the joints
		
};                                                                                                  // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructer                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialKinControl::SerialKinControl(const std::vector<RigidBody> links,
                                   const std::vector<Joint> joints,
                                   const float &controlFrequency):
                                   SerialLink(links, joints),
                                   dt(1/controlFrequency)
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
	Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);        // Get the SVD decomposition
	Eigen::MatrixXf V = SVD.matrixV();                                                          // V matrix
	Eigen::MatrixXf U = SVD.matrixU();                                                          // U matrix
	Eigen::VectorXf s = SVD.singularValues();                                                   // Get the singular values
	Eigen::MatrixXf invA(A.cols(), A.rows()); invA.setZero();                                   // Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-04) invA(i,k) += (V(i,j)*U(k,j))/s(j);                // Fast inverse
				else                     
				{
//                                      invA(i,k) += 0;                                             // Ignore singular directions

					std::cout << "[WARNING] [SERIALKINCONTROL] get_inverse(): "
						  << "Matrix is near-singular!" << std::endl;
				}
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
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_weighted_inverse() : "
                          << "Weighting matrix must be square. Your input had " << W.rows() << " rows and "
                          << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
                         
		return get_inverse(A);                                                              // Ignore the weighting matrix
	}
	else if(A.cols() == W.rows())                                                               // Overdetermined system
	{
		Eigen::MatrixXf invWAt = get_inverse(W)*A.transpose();
		
		return invWAt*get_inverse(A*invWAt);                                                // W^-1*A'*(A*W^-1*A')^-1
	}
	else if(W.cols() == A.rows())                                                               // Underdetermined system
	{
		Eigen::MatrixXf AtW = A.transpose()*W;
		
		return get_inverse(AtW*A)*AtW;                                                      // (A'*W*A)^-1*A'*W
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
 //                  Solve the joint velocities to achieve the endpoint velocity                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::VectorXf &vel)
{
	if(primary_task_is_ok(vel))
	{
		if(this->n <= 6) return rmrc(vel, Eigen::VectorXf::Zero(this->n));                  // Not redundant, secondary task doesn't matter
		else             return rmrc(vel, singularity_avoidance(5.0));                      // Apply automatic singularity avoidance
	}
	else	return Eigen::VectorXf::Zero(this->n);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //     Solve the joint velocities to achieve the endpoint velocity with added redundant task      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::VectorXf &vel,
							const Eigen::VectorXf &secondaryTask)
{
	if(primary_task_is_ok(vel) and secondary_task_is_ok(secondaryTask))
	{
		return rmrc(vel, secondaryTask);
	}
	else	return Eigen::VectorXf::Zero(this->n);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Follow the endpoint pose at the given speed                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::Isometry3f &pose,
                                                        const Eigen::VectorXf &vel)
{
	if(primary_task_is_ok(vel))
	{
		Eigen::VectorXf xdot = vel + this->k*get_pose_error(pose, this->fkChain.back());
		
		if(this->n <= 6) return rmrc(xdot, Eigen::VectorXf::Zero(this->n));                 // Not redundant, secondary task doesn't matter
		else             return rmrc(xdot, singularity_avoidance(5.0));                     // Apply automatic singularity avoidance
	}
	else	return Eigen::VectorXf::Zero(this->n);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //            Follow the endpoint pose at the given speed, with added redundant task              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::Isometry3f &pose,
                                                        const Eigen::VectorXf &vel,
                                                        const Eigen::VectorXf &secondaryTask)
{
	if(primary_task_is_ok(vel) and secondary_task_is_ok(secondaryTask))
	{
		return rmrc(vel + this->k*get_pose_error(pose, this->fkChain.back()), secondaryTask);
	}
	else	return Eigen::VectorXf::Zero(this->n);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //             Compute the feedfoward + feedback control to track a joint trajectory              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_joint_control(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() != this->n || vel.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_joint_control() : "
                         << "This robot has " << this->n << " joints "
                         << "but the position argument had " << pos.size() << " elements "
                         << "and the velocity argument had " << vel.size() << " elements." << std::endl;

		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		Eigen::VectorXf qdot = Eigen::VectorXf::Zero(this->n);                              // Value to be returned
		
		for(int i = 0; i < this->n; i++)
		{
			qdot(i) = vel(i) + this->k*(pos(i) - this->q(i));                           // Feedforward + feedback control	
			limit_joint_velocity(qdot(i), i);                                           // Ensure kinematic feasiblity
		}
		
		return qdot;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Get the error between two poses for feedback control                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_pose_error(const Eigen::Isometry3f &desired, const Eigen::Isometry3f &actual)
{
	// Yuan, J. S. (1988). Closed-loop manipulator control using quaternion feedback.
	// IEEE Journal on Robotics and Automation, 4(4), 434-440.
	
	Eigen::VectorXf error(6);                                                                   // Value to be returned
	
	error.head(3) = desired.translation() - actual.translation();                               // Position error
	
	Eigen::Quaternionf temp(desired.rotation()*actual.rotation().inverse());                    // Orientation error as quaternion
	
	float angle = 2*asin(temp.vec().norm());                                                    // Get the angle error
	
	if(angle < M_PI) error.tail(3) =  temp.vec();
	else             error.tail(3) = -temp.vec();                                               // Angle > 180 degrees, so reverse direction

	return error;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //         Limit the joint velocities to obey position, speed, acceleration constraints           //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::limit_joint_velocity(float &qdot, const int &i)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2012, May). Motion control of redundant robots
	// under joint constraints: Saturation in the null space.
	// In 2012 IEEE International Conference on Robotics and Automation (pp. 285-292). IEEE.
	
	float p = (this->pLim[i][0] - this->q[i])/this->dt;                                         // Velocity constraint to avoid position limit
	float v = -this->vLim[i];                                                                   // Minimum velocity
	float a = -sqrt(2*this->aLim[i]*(this->q[i] - this->pLim[i][0]));                           // Velocity constraint based on maximum braking
	
	float lower = std::max(std::max(p,v), std::max(v,a));                                       // Get the maximum of the 3 values
	
        p = (this->pLim[i][1] - this->q[i])/this->dt;
        v = this->vLim[i];
        a = sqrt(2*this->aLim[i]*(this->pLim[i][1] - this->q[i]));
        
        float upper = std::min(std::min(p,v), std::min(v,a));                                       // Get the minimum of the 3 values

	if(qdot < lower)
	{
		qdot = lower + 0.001;                                                               // Just above the lower limit
		
		std::cout << "[WARNING] [SERIALKINCONTROL] limit_joint_velocity(): "
			  << "Joint " << i+1 << " hit its lower limit!" << std::endl;
			  
		return true;
	}
	else if(qdot > upper)
	{
		qdot = upper - 0.001;                                                               // Just below the upper limit
		
		std::cout << "[WARNING] [SERIALKINCONTROL] limit_joint_velocity(): "
			  << "Joint " << i+1 << " hit its upper limit!" << std::endl;
			  
		return true;
	}
	else	return false;                                                                       // Joint was not limited
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Check that the endpoint velocity vector has 6 dimensions                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::primary_task_is_ok(const Eigen::VectorXf &task)
{
	if(task.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
			  << "Expected a 6x1 vector for the velocity argument "
			  << "but it had " << task.size() << " elements." << std::endl;
		
		return false;
	}
	else	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Check that the secondary task is feasible                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::secondary_task_is_ok(const Eigen::VectorXf &task)
{
	if(this->n <= 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
			  << "This robot has no redundancy so a secondary task cannot be applied. "
			  << "Use the function get_cartesian_control(const std::vector &vel) instead." << std::endl;
			  
		return false;
	}
	else if(task.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
			  << "Expected a " << this->n << "x1 vector for the secondary task "
			  << "but it had " << task.size() << " elements." << std::endl;
		
		return false;
	}
	else	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Compute a weighting matrix to avoid joint limits                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialKinControl::get_joint_weighting()
{
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based scheme
	// for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	Eigen::MatrixXf W(this->n, this->n); W.setIdentity();                                       // Value to be returned
	float q, dpdq, range, upper, lower;
	for(int i = 0; i < this->n; i++)
	{
		upper = this->pLim[i][1] - this->q[i];                                              // Distance to upper limit
		lower = this->q[i] - this->pLim[i][0];                                              // Distance to lower limit
		range = this->pLim[i][1] - this->pLim[i][0];                                        // Difference between upper and lower
		dpdq = (range*range*(2*q - this->pLim[i][1] - this->pLim[i][0]))                    // Partial derivative of penalty function
		      /(4*upper*upper*lower*lower);
			
		if(dpdq*this->qdot[i] > 0)                                                          // If moving toward a limit...
		{
			W(i,i) = range*range/(4*upper*lower);                                       // Penalize joint motion
			
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
 //                    The full resolved motion rate control (rmrc) algorithm                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::rmrc(const Eigen::VectorXf &xdot, const Eigen::VectorXf &qdot_d)
{
	// Whitney, D. E. (1969). Resolved motion rate control of manipulators and human prostheses.
	// IEEE Transactions on man-machine systems, 10(2), 47-53.
	
	// Solve the equation: qdot = invJ*xdot + (I - invJ*J)*qdot_d
	// If not redundant this reduces to qdot = invJ*xdot

	// Variables used in this scope
	Eigen::VectorXf qdot = Eigen::VectorXf::Zero(this->n);                                      // Value to be returned
	Eigen::MatrixXf J = get_jacobian();                                                         // xdot = J*qdot
	Eigen::MatrixXf invJ;                                                                       // J*invJ = I
	
	if(this->n <= 6) invJ = get_inverse(J);
	else		 invJ = get_weighted_inverse(J, this->M + get_joint_weighting());           // Weighted for energy, joint limit avoidance
	
	Eigen::MatrixXf N = Eigen::MatrixXf::Identity(this->n, this->n) - invJ*J;                   // For redundant robots, J*N = 0
	
	// Solve each individual joint velocity
	for(int i = 0; i < this->n; i++)
	{
		for(int j = 0; j < this->n; j++)
		{
			if(j < 6)       qdot(i) += invJ(i,j)*xdot(j);                               // Range space velocities
			if(this->n > 6) qdot(i) += N(i,j)*qdot_d(j);                                // Null space velocities
		}
		
		limit_joint_velocity(qdot(i), i);                                                   // Ensure kinematic feasibility
	}
	
	return qdot;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the gradient of manipulability to avoid singularities                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::singularity_avoidance(const float &scalar)
{
	// Yoshikawa, T. (1985). Manipulability of robotic mechanisms.
	// The international journal of Robotics Research, 4(2), 3-9.
	
	// Proof of the partial derivative can be found in this book:
	// Marani, G., & Yuh, J. (2014). Introduction to autonomous manipulation:
	// Case Study with an Underwater Robot, SAUVIM
	// Volume 102 of Springer Tracts in Advanced Robotics. Springer.
	
	if(scalar <= 0)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] singularity_avoidance(): "
			  << "Input argument was " << scalar << " but it must be positive!" << std::endl;
	
		return Eigen::VectorXf::Zero(this->n);
	}
	else
	{
		Eigen::MatrixXf J = get_jacobian();                                                 // As it says on the label
		Eigen::MatrixXf JJt = J*J.transpose();                                              // Makes calcs a little simpler
		Eigen::MatrixXf invJ = J.transpose()*get_inverse(JJt);                              // Pseudoinverse Jacobian
		
		float mu = sqrt(JJt.determinant());                                                 // Actual measure of manipulability
		
		Eigen::MatrixXf dJ;                                                                 // Placeholder for partial derivative
		
		Eigen::VectorXf grad(this->n);                                                      // Value to be returned
		
		grad(0) = 0.0;                                                                      // First joint does not affect manipulability
		
		for(int i = 1; i < this->n; i++)
		{
			dJ = get_partial_derivative(J,i);                                           // Get partial derivative w.r.t ith joint
			grad(i) = scalar*mu*(dJ*invJ).trace();                                      // Gradient of manipulability
		}
		
		return grad;
	}
}

#endif
