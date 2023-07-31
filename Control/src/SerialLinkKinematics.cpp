#include <SerialLinkKinematics.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialLinkKinematics::resolve_endpoint_motion(const Eigen::Matrix<float,6,1> &endpointMotion)
{
	Eigen::MatrixXf J = endpoint_jacobian();                                                    // As it says on the label
	
	Eigen::MatrixXf JJt = J*J.transpose();                                                      // Makes calcs a little easier
	
	float manipulability = sqrt((JJt).determinant());                                           // Proximity to a singularity
	
	if(manipulability <= this->threshold)
	{
		std::cout << "[WARNING] [SERIAL LINK CONTROL] resolve_endpoint_motion(): "
		          << "Robot is in a singular configuration! "
		          << "(Manipulability " << manipulability << " < " <<
		          << "threshold " << this->threshold << ").\n";
		
		return 0.9*this->jointVelocity;                                                     // Slow down
	}

	Eigen::VectorXf gradient(this->numJoints); gradient(0) = 0;                                 // Gradient of manipulability
	
	Eigen::VectorXf startPoint = this->jointVelocity;                                           // Needed for the QP solver

	Eigen::VectorXf upperBound(this->numJoints),
                        lowerBound(this->numJoints);                                                // Instantaneous limits on solution

	// Compute joint control limits and gradient of manipulability
	for(int i = 0; i < this->numJoints; i++)
	{
		// Maximum speed permissable
		if(not compute_joint_limits(lowerBound(i), upperBound(i), i))
		{
			throw std::runtime_error("[FLAGRANT SYSTEM ERROR] resolve_endpoint_motion(): "
				                 "Unable to compute the joint limits.");
		}
		
		// Ensure start point is inside bounds or QP solver will fail
		     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 0.001;
		else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 0.001;
		
		// Get the gradient of manipulability
		if(i > 0)
		{
			Eigen::MatrixXf dJ = partial_derivative(J,i);                               // Partial derivative w.r.t. ith joint
			
			gradient(i) = manipulability*(JJt.partialPivLu().solve(dJ*J.transpose()));  // Gradient of manipulability
		}
	}
	
	// Set up the inequality constraints for the QP solver B*qdot > z
	
	// B = [    -I     ]   >   z = [ -upperBound ]
	//     [     I     ]           [  lowerBound ]
	//     [ (dmu/dq)' ]           [ -gamma*mu   ]
	
	unsigned int n = this->numJoints;                                                           // I'm too lazy to type 'this->numJoints' every time
	
	Eigen::MatrixXf B(2*n+1,n);
	
	B.block(n,0,n,n).setIdentity();
	B.block(0,0,n,n) = -B.block(n,0,n,n);
	B.row(2*n+1) = gradient.transpose();
	
	Eigen::VectorXf z(2*n+1);
	
	z.block(0,0,n,1) = -upperBound;
	z.block(n,0,n,1) =  lowerBound;
	z(2*n+1) = -this->barrierScalar*manipulability;
	
	// Solve optimisation problem based on number of joints
	
	if(this->numJoints <= 6)                                                                    // No redundancy
	{
		// min || xdot - J*qdot ||^2
		// subject to:   qdot_min < qdot < qdot_max
		//               (dmu/dq)'*qdot > -gamma*mu 
		
		return QPSolver::solve(J.transpose()*J,-endpointMotion,B,z,startPoint);             // Too easy lol				
	}
	else
	{
		if(not this->redundantTaskSet)
		{
			this->redundantTask = gradient;                                             // Optimise manipulability by default
			this->redundantTaskSet = false;                                             // Reset for next control loop
		}
		
		// Make sure the redundant task is feasible after projection
		// on to the null space or the QP solver will fail
		
		float alpha = 1.0;
		
		Eigen::MatrixXf invMJt = this->jointInertiaMatrix.ldlt().solve(J.transpose());      // Makes calcs a little easier
		
		EigenVectorXf xn = redundantTask - invMJt*(J*invMJt).ldlt().solve(J*redundantTask); // Null space projection
		
		for(int i = 0; i < this->numJoints; i++)
		{
			     if(xn(i) <= lowerBound(i)) alpha = std::min((float)0.9*lowerBound(i)/xn(i), alpha);
			else if(xn(i) >= upperBound(i)) alpha = std::min((float)0.9*upperBound(i)/xn(i), alpha);
		}
		
		redundantTask *= alpha;                                                             // Scale accordingly
		
		return QPSolver::redundant_least_squares(redundantTask, M, endpointMotion, J, z, B, startPoint);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,1> SerialLinkKinematics::track_endpoint_trajectory(const Pose &desiredPose,
                                                                          const Eigen::Matrix<double,6,1> &desiredVel,
                                                                          const Eigen::Matrix<double,6,1> &desiredAcc)
{
	return resolve_endpoint_motion(desiredVel + this->K*this->endpoint_pose().error(desiredPose)); // Feedforward + feedback
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint velocities needs to track a given trajectory               //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialLinkKinematics::track_joint_trajectory(const Eigen::VectorXf &desiredPos,
                                                             const Eigen::VectorXf &desiredVel,
                                                             const Eigen::VectorXf &desiredAcc)
{
	if(desiredPos.size() != this->numJoints	or desiredVel.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] track_joint_trajectory(): "
		          << "This robot has " this->numJoints << " joints but "
		          << "the position argument had " << desiredPos.size() << " elements and "
		          << "the velocity argument had " << desiredVel.size() << " elements.\n";
		
		return 0.9*this->jointVelocity;
	}
	else
	{
		Eigen::VectorXf vel = desiredVel + this->kp*(desiredPos + this->jointPosition);    // Feedforward + feedback
	
		// Ensure kinematic feasibility	
		for(int i = 0; i < this->numJoints; i++)
		{
			float lowerBound, upperBound;
			
			if(not compute_joint_limits(lowerBound,upperBound,i))
			{
				std::cerr << "[ERROR] [SERIAL LINK CONTROL] track_joint_trajectory(): "
				          << "Could not compute joint limits for the '"
				          << this->joint[i].name() << "' joint.\n";
			}
			else
			{
				     if(vel(i) <= lowerBound) vel(i) = lowerBound + 0.001;
				else if(vel(i) >= upperBound) vel(i) = upperBound - 0.001;
			}
		}
		
		return vel;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLinkKinematics::compute_joint_limits(float &lower, float &upper, const unsigned int &jointNumber)
{
	float dq = this->jointPosition[jointNumber] - this->positionLimit[jointNumber][0];
	
	lower = std::max( -this->hertz*dq,
	        std::max( -this->velocityLimit[jointNumber],
	                  -sqrt(2*this->accelLimit[jointNumber]*dq) ));
	                  
	dq = this->positionLimit[jointNumber][1] - this->jointPosition[jointNumber];
	
	upper = std::min( this->hertz*dq,
	        std::min( this->velocityLimit[jointNumber],
	                  sqrt(2*this->accelLimit[jointNumber]*dq) ));
	                  
	if(lower >= upper)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] compute_joint_limits(): "
		          << "Lower bound is greater than upper bound for the '"
		          << this->joint[i].name() << "' joint. "
		          << "(" << lower << " >= " << upper << "). How did that happen?\n";
	
		return false;
	}
	else 	return true;        
}
