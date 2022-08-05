#include <SerialKinControl.h>
 
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
		this->aLim[i] = 5.0;
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

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Move the endpoint at a given speed                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::move_at_speed(const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] move_at_speed(): "
		          << "Expected a 6x1 vector for the input, "
		          << "but it was " << vel.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();                                                              // Slow down to avoid problems
	}
	else
	{
		if(this->n <= 6) return move_at_speed(vel, Eigen::VectorXf::Zero(this->n));         // No redundancy
		else             return move_at_speed(vel, singularity_avoidance(0.5));             // Automatic singularity avoidance
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move the endpoint at a given speed                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::move_at_speed(const Eigen::VectorXf &vel,
                                                const Eigen::VectorXf &redundant)
{
	// Whitney, D. E. (1969). Resolved motion rate control of manipulators and human prostheses.
	// IEEE Transactions on man-machine systems, 10(2), 47-53.
	//
	// Given xdot = J*qdot, we can solve for:
	//
	// - qdot = J^-1 *xdot               if n <= 6
	// - qdot = J^-1 &xdot + N*redundant otherwise
	//
	// Matrix inversion is prone to numerical instability when ill-conditioned,
	// so we can use QR decomposition instead
	
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] move_at_speed(): "
		          << "Expected a 6x1 vector for the input, but it was " << vel.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else if(redundant.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] move_at_speed(): "
		          << "Expected a " << this->n << "x1 vector for the redundant task, "
		          << "but it was " << redundant.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else
	{
		Eigen::VectorXf ctrl;
		Eigen::MatrixXf J  = get_jacobian();                                                // As it says on the label
		Eigen::MatrixXf Jt = J.transpose();                                                 // Makes calcs a little easier
		Eigen::MatrixXf A, Q, R;
		Eigen::MatrixXf W = get_joint_weighting();                                          // Penalise joint motion toward limits
		Eigen::VectorXf y;
	
		if(this->n <= 6) 
		{
			// Solve J'*J*qdot = J'*xdot
			// Then do QR decomposition on J'*J such that:
			//      Q*R*qdot = J'*xdot
			//        R*qdot = Q'*J'*xdot
			A = Jt*J;
			ctrl.resize(this->n);
		}
		else
		{
			// Minimize 0.5*(redundant - qdot)'*W*(redundant - qdot) subject to xdot = J*qdot
			// Lagrangian:
			//
			//     L = 0.5*qdot'*W*qdot - qdot'*W*redundant + (J*qdot - xdot)'*lambda
			//
			// Solution exists where derivative is zero:
			//
			//    [ dL/dlambda ] = [ 0  J ][ lambda ] - [    xdot     ]  = [ 0 ]
			//    [  dL/dqdot  ]   [ J' W ][  qdot  ]   [ W*redundant ]    [ 0 ]
			//                    `-------'            `---------------'
			//                        A                        y
			// Then do QR decomp on matrix A. To speed up calcs, we can avoid computing lambda.
			 
			A.resize(6+this->n,this->n);
			A.block(0,0,6,6).setZero();
			A.block(6,0,this->n,      6) = Jt;
			A.block(0,6,      6,this->n) = J;
			A.block(6,6,this->n,this->n) = get_inertia() + W;
			
			ctrl.resize(6+this->n);
		}
		
		if(get_qr_decomposition(A,Q,R))
		{
			// Since R is upper-triangular, we can solve it with back-substitution
			
			if(this->n <= 6) y = Q.transpose()*Jt*vel;
			else             y = Q.block(0,6,this->n,      6).transpose()*y
			                   + Q.block(6,6,this->n,this->n).transpose()*W*redundant;
			                   
			for(int i = this->n-1; i >= 0; i--)                                         // Use back substitution to solve control
			{
				float sum = 0.0;
				for(int j = i; j < this->n; j < this->n)
				{
					sum += R(i,j)*ctrl(j);                                      // Sum up recursive values
				}
				
				if(abs(R(i,i)) < 1E-6) ctrl(i) = 0.9*get_joint_velocity(i);         // Joint causing singularity, slow down
				else                   ctrl(i) = (y(i) - sum)/R(i,i);               // Solve as normal
				
				// Limit the joint velocities
				float lower, upper;
				get_speed_limit(lower, upper, i);
				
				if     (ctrl(i) < lower) ctrl(i) = lower;
				else if(ctrl(i) > upper) ctrl(i) = upper;
			}
				
			if(this->n <= 6) return ctrl;
			else             return ctrl.block(6,0,this->n,1);                          // Only need the last n values
		}
		else	return 0.9*get_joint_velocities();
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //            Get the joint velocity to move to a given joint position at max. speed              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::move_to_position(const Eigen::VectorXf &pos)
{
	if(pos.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] move_to_position(): "
		          << "Expected a " << this->n << "x1 vector for the input, "
		          << "but it was " << pos.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();                                                  // Slow down joints to avoid problems
	}
	else
	{
		Eigen::VectorXf ctrl(this->n);                                                      // Value to be returned
		
		for(int i = 0; i < this->n; i++)
		{
			ctrl(i) = this->k*(pos(i) - this->q[i]);                                    // Use proportional feedback
			
			// Limit the speed
			float lower, upper;
			get_speed_limit(lower, upper, i);
			if     (ctrl(i) < lower) ctrl(i) = lower;
			else if(ctrl(i) > upper) ctrl(i) = upper;
		}
		
		return ctrl;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Move the robot to a given endpoint pose                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::move_to_pose(const Eigen::Isometry3f &pose)
{
	Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                              // As it says on the label
	
	return move_at_speed(this->k*e);                                                            // Proportional feedback
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Move the robot to a given endpoint pose                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::move_to_pose(const Eigen::Isometry3f &pose,
                                               const Eigen::VectorXf &redundant)
{
	if(redundant.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALLINK] move_to_pose(): "
		          << "Expected a " << this->n << "x1 vector for the redundant task, "
		          << "but it was " << redundant.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();                                                  // Slow down to avoid problems
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                      // As it says on the label
		
		return move_at_speed(this->k*e, redundant);                                         // Use proportional feedback
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Track Cartesian trajectory                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::track_cartesian_trajectory(const Eigen::Isometry3f &pose,
                                                             const Eigen::VectorXf &vel)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] track_cartesian_trajectory(): "
		          << "Expected a 6x1 vector for the velocity argument, "
		          << "but it was " << vel.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                      // As it says on the label
		
		return move_at_speed(vel + this->k*e);                                              // Feedforward + feedback control
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Track Cartesian trajectory                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::track_cartesian_trajectory(const Eigen::Isometry3f &pose,
                                                             const Eigen::VectorXf &vel,
                                                             const Eigen::VectorXf &redundant)
{
	if(vel.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] track_cartesian_trajectory(): "
		          << "Expected a 6x1 vector for the velocity argument, "
		          << "but it was " << vel.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else if(redundant.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] track_cartesian_trajectory(): "
		          << "Expected a " << this->n << "x1 vector for the redundant task, "
		          << "but it was " << redundant.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else
	{
		Eigen::VectorXf e = get_pose_error(pose, get_endpoint_pose());                      // As it says on the label
		
		return move_at_speed(vel + this->k*e, redundant);                                   // Feedforward + feedback control
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Track a joint trajectory                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::track_joint_trajectory(const Eigen::VectorXf &pos,
                                                         const Eigen::VectorXf &vel)
{
	if(pos.size() != this->n or vel.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] track_joint_trajectory(): "
		          << "This robot has " << this->n << " joints, but "
		          << "position vector was " << pos.size() << "x1 and "
		          << "velocity vector was " << vel.size() << "x1." << std::endl;
		
		return 0.9*get_joint_velocities();
	}
	else
	{
		Eigen::VectorXf ctrl(this->n);                                                      // Value to be returned
		
		for(int i = 0; i < this->n; i++)
		{
			ctrl(i) = vel(i) + this->k*(pos(i) - get_joint_position(i));                // Feedforward + feedback control
			
			// Limit the joint speeds
			float lower, upper;
			get_speed_limit(lower, upper, i);
			
			if     (ctrl(i) < lower) ctrl(i) = lower;
			else if(ctrl(i) > upper) ctrl(i) = upper;
		}
		
		return ctrl;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Get the speed limits on a particular joint for the current state               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::get_speed_limit(float &lower, float &upper, const int &i)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2012, May). Motion control of redundant robots
	// under joint constraints: Saturation in the null space.
	// In 2012 IEEE International Conference on Robotics and Automation (pp. 285-292). IEEE.
	
	// Compute lowest possible speed
	float p = (this->pLim[i][0] - this->q[i])/this->dt;                                         // Minimum speed before hitting joint limit
	float v = -this->vLim[i];                                                                   // Negative of maximum motor speed
	float a = -2*sqrt(this->aLim[i]*(this->q[i] - this->pLim[i][0]));                           // Minimum speed at maximum braking
	
	lower = std::max(p,std::max(v,a));                                                          // Return largest of the 3
	
	// Compute largest possible speed
	p = (this->pLim[i][1] - this->q[i])/this->dt;                                               // Maximum speed before hitting joint limit
	v = this->vLim[i];                                                                          // Maximum motor speed
	a = 2*sqrt(this->aLim[i]*(this->pLim[i][1] - this->q[i]));                                  // Maximum speed at maximum braking
	
	upper = std::min(p,std::min(v,a));                                                          // Return smallest
	
	return true;
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
			
		if(dpdq*get_joint_velocities()[i] > 0)                                                          // If moving toward a limit...
		{
			W(i,i) = range*range/(4*upper*lower) - 1;                                   // Penalize joint motion (minimum 0)
			
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
		Eigen::MatrixXf J    = get_jacobian();                                              // As it says on the label
		Eigen::MatrixXf JJt  = J*J.transpose();                                             // Makes calcs a little simpler
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
