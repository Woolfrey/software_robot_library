#include <SerialDynamicControl.h>


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //     Track a Cartesian trajectory defined by pose, instantaneous velocity and acceleration     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,
                                           const Eigen::VectorXf   &vel,
					   const Eigen::VectorXf   &acc)
{
	if(vel.size() != 6 or acc.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALDYNAMICCONTROL] track_cartesian_trajectory(): "
		          << "Expected 6x1 vectors for the input arguments, but "
			  << "the velocity vector was " << vel.size() << "x1 and "
			  << "the acceleration vector was " << acc.size() << "x1." << std::endl;
		
		return get_feedback_linearization() - this->kd*this->get_joint_velocities();
	}
	else
	{
		
	}
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //       Track a Cartesian trajectory defined by pose, instantaneous velocity and acceleration    //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,
                                           const Eigen::Vector3f   &vel,
					   const Eigen::Vector3f   &acc,
					   const Eigen::Vector3f   &redundancy)
{
	if(vel.size() != 6 or acc.size() != 6)
	{
		std::cerr << "[ERROR] [SERIALDYNAMICCONTROL] track_cartesian_trajectory(): "
			  << "Expected 6x1 vectors for the inputs, but "
			  << "the velocity vector was " << vel.size() << "x1 and "
			  << "the acceleration vector was " << acc.size() << "x1." << std::endl;
		
		return get_feedback_linearization() - this->kd*this->get_joint_velocities();                // Slow down the robot
	}
	else if(redundancy.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALDYNAMICCONTROL] track_cartesian_trajectory(): "
		          << "Expected a " << this->n << "x1 vector for the redundant task "
		          << "but it was " << redundancy.size() << "x1." << std::endl;
				  
		return get_feedback_linearization() - this->kd*this->get_joint_velocities();
	}
	else
	{
			Eigen::MatrixXf J = get_jacobian();
			Eigen::MatrixXf M = get_inertia();
			
			// Feedforward + feedback control: xddot = xddot_d + J*M^-1*J'*( D*edot + K*e )
			// Convert to acceleration to 1) Avoid inverting singular Jacobian
			//                            2) Synthesize kinematic & dynamic constraints
			
			Eigen::VectorXf xddot = acc + J*M.llt().solve( J.transpose()*( this->D*(vel - get_joint_velocities() )
			                                                             + this->K*get_pose_error(pose,get_endpoint_pose()) );
																	 
			return accelerate_endpoint(xddot, redundancy);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      This is the fundamental endpoint control function                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf accelerate_endpoint(const Eigen::VectorXf &acc,
                                          Eigen::VectorXf &redundancy)
{
	// Variables used in this scope
	Eigen::MatrixXf J    = get_jacobian();
	Eigen::MatrixXf Jdot = get_time_derivative(J);
	Eigen::MatrixXf M    = get_inertia();
	Eigen::VectorXf h    = get_feedback_linearization();                                            // Coriolis and gravity terms
	Eigen::VectorXf qdot = get_joint_velocities();
	Eigen::VectorXf x0(6+this->n);                                                                  // Initial guess for QP solver
	
	// Solve QP problem  min 0.5*x'*H*x + f subject to:   B*x >= z
	// where the decision variable is:
	//     x = [ lambda ]
	//         [  qddot ]
	// lambda is the Lagrange multiplier that enforces endpoint motion
	
	// H = [ 0   J ]
	//     [ J'  M ]
	Eigen::MatrixXf H(6+this->n);
	H.block(0,0,      6,      6) = Eigen::MatrixXf::Zero(this->n,this->n);
	H.block(0,6,      6,this->n) = J;
	H.block(6,0,this->n,      6) = J.transpose();
	H.block(6,6,this->n,this->n) = M;
	
	// B = [ 0 -I ]
	//     [ 0  I ]
	//     [ 0 -M ]
	//     [ 0  M ]
	Eigen::MatrixXf B(4*this->n,6+this->n);
	B.block(        0,0,4*this->n,      6) =  Eigen::MatrixXf::Zero(4*this->n, 6);
	B.block(        0,6,  this->n,this->n) = -Eigen::MatrixXf::Identity(this->n,this->n);
	B.block(1*this->n,6,  this->n,this->n) =  Eigen::MatrixXf::Identity(this->n,this->n);
	B.block(2*this->n,6,  this->n,this->n) = -M;
	B.block(3*this->n,6,  this->n,this->n) =  M;
	
	// z = [ -maxAcc ]
	//     [  minAcc ]
	//     [ -maxTau ]
	//     [  minTau ]
	Eigen::VectorXf z(4*this->n);
	for(int i = 0; i < this->n; i++)
	{
		float lower, upper;
		get_acceleration_limits(lower, upper, i);                                                     // Instantaneous limits
		
		z(i)           = -upper;
		z(i+1*this->n) =  lower;
		z(i+2*this->n) =  h(i) - this->tMax[i];                                                     // -M*qddot >= -tauMax + h
		z(i+3*this->n) = -(this->tMax[i] + h(i))                                                    //  M*qddot >= -tauMax - h
		
		x0(i) = 0.5*(lower + upper);
		
		// QP solver won't return a good solution if redundant task is too large
		if     (tau0(i) >  this->tMax[i]) tau0(i) = this->tMax[i] - 0.001;                          // Just below the limit
		else if(tau0(i) < -this->tMax[i]) tau0(i) = 0.001 - this->tMax[i];                          // Just above the limit
	}
	
	// f = [ Jdot*qdot - xddot ]
	//     [      -tau0        ]
	Eigen::VectorXf f(6+this->n);
	f.head(6)       = Jdot*qdot - xddot;
	f.tail(this->n) = -tau0;
	
	Eigen::VectorXf solution = solve(H,f,B,z,x0);                                                   // Solve the QP problem
		
	return M*solution.tail(n) + h;
}
	

/*	
	// Set up QP problem. Decision variable is:
	//
	//      x = [ lambda_1 ] 6
	//          [ lambda_2 ] n  
	//          [  qddot   ] n
	//          [   tau    ] n
	
	// H = [ 0   0  J  0 ]
	//     [ 0   0 -M  I ]
	//     [ J' -M  0  0 ]
	//     [ 0   M  0  I ]
	Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6+3*this->n,6+3*this->n);
	H.block(0          ,6+1*this->n,      6,this->n) = J;
	H.block(6          ,6+1*this->n,this->n,this->n) =-M;
	H.block(6          ,6+2*this->m,this->n,this->n) = Eigen::MatrixXf::Identity(this->n,this->n);
	H.block(6+1*this->n,          0,this->n,      6) = J.transpose();
	H.block(6+1*this->n,          6,this->n,this->n) =-M;
	H.block(6+2*this->n,          6,this->n,this->n) = M;
	H.block(6+2*this->n,6+2*this->n,this->n,this->n) = Eigen::MatrixXf::Identity(this->n,this->n);
	
	// B = [ 0 0 -I  0 ]
	//     [ 0 0  I  0 ]
	//     [ 0 0  0 -I ]
	//     [ 0 0  0  I ]
	Eigen::MatrixXf B = Eigen::MatrixXf::Zero(4*this->n,6+3*this->n);
	B.block(        0,6+1*this->n,this->n,this->n) =-Eigen::MatrixXf::Identity(this->n,this->n);
	B.block(1*this->n,6+1*this->n,this->n,this->n) = Eigen::MatrixXf::Identity(this->n,this->n);
	B.block(2*this->n,6+2*this->n,this->n,this->n) =-Eigen::MatrixXf::Identity(this->n,this->n);
	B.block(3*this->n,6+2*this->n,this->n,this->n) = Eigen::MatrixXf::Identity(this->n,this->n);
	
	// z = [ -maxAcc ]
	//     [  minAcc ]
	//     [ -maxTau ]
	//     [  minTau ]
	Eigen::VectorXf tau0 = redundancy;
	Eigen::VectorXf z = Eigen::VectorXf::Zero(4*this->n);
	for(int i = 0; i < this->n; i++)
	{
		float lower, upper;
		get_acceleration_limits(lower, upper);
		z(i)           = -upper;
		z(i+1*this->n) =  lower;
		z(i+2*this->n) = -this->tMax[i];
		z(i+3*this->n) = -this->tMin[i];
		
		// Ensure that redundant task is within limits to assist the QP solver
		if     (tau0[i] >  this->tMax[i]) tau0[i] =  this->tMax[i] - 0.01;                          // Set just below the limit
		else if(tau0[i] < -this->tMax[i]) tau0[i] = -this->tMax[i] + 0.01;                          // Set just above the limit
	}
	
	// f = [ Jdot*qdot - xddot ]
	//     [        -h        ]
	//     [         0        ]
	//     [    -redundancy   ]
	Eigen::VectorXf f(6+3*this->n);
	f.block(          0,0,      6,1) =  Jdot*qdot - acc;
	f.block(          6,0,this->n,1) = -h;
	f.block(6+1*this->n,0,this->n,1) =  Eigen::VectorXf::Zero(this->n);
	f.block(6+2*this->n,0,this->n,1) = -tau0;                                                       // Scale the redundant task for feasibility
	
	this->previousSolution = solve(H,f,B,z,previousSolution);                                       // Solve the QP problem
	
	return previousSolution.tail(this->n);                                                          // Return the joint torques
	
	*/
