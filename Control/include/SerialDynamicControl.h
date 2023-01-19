

class SerialDynamicControl : public SerialBaseControl
{
	public:
	
		SerialDynamicControl() {}                                                                    // Empty constructor
		
		// Functions derived from the base class
		Eigen::VectorXf move_endpoint(const Eigen::Matrix<float,6,1> &speed);                       // Move endpoint at given acceleration
		
		Eigen::VectorXf move_joints(const Eigen::VectorXf &speed);                                  // Move joints at given acceleration
				
		Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,                   // Solve Cartesian feedback control
		                                           const Eigen::Matrix<float,6,1> &vel,
												   const Eigen::Matrix<float,6,1> &acc);
														   
		Eigen::VectorXf track_joint_trajectory(const Eigen::VectorXf &pos,                          // Solve joint feedback control
		                                       const Eigen::VectorXf &vel,
											   const Eigen::VectorXf &acc);
													   		
	private:
		bool get_acceleration_bounds(float &lower, float &upper, const int &jointNumber);		
};                                                                                                  // Semicolon needed after a class declaration

  //////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Move the endpoint at the given acceleration                            //
//////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf move_endpoint(const Eigen::Matrix<float,6,1> &speed)
{
	Eigen::VectorXf qddot(this->n);                                                                 // Value to be solved
	Eigen::Matrix<double,6,1> offset = get_time_derivative(this->J)*this->qdot;                     // Jdot*qdot
	
	// Compute the instantaneous lower and upper bounds
	Eigen::VectorXf lowerBound(this->n), upperBound(this->n);
	for(int i = 0; i < this->n; i++) get_acceleration_limits(lowerBound[i],upperBound[i],i);
		
	if(this->n <= 6)                                                                                // No redundancy
	{
		qddot = least_squares(speed - offset, J, lowerBound, upperBound, initialGuess);             // Too easy lol
	}
	else
	{
		// Bruyninckx & Khatib (2000) Gauss' Principle and the Dynamics of Redundant
		// and Constrained Manipulators. IEEE International Conference on Robotics and
		// Automation (ICRA) pp. 256 - 2568
		
		// Solve the redundant task
		Eigen::MatrixXf M = get_inertia_matrix();
		Eigen::VectorXf unconstrainedAccel;
		if(this->redundantTaskSet) unconstrainedAccel = M.partialPivLu().solve(-this->redundantTask);
		else                       unconstrainedAccel = M.partialPivLu().solve(-this->D*(avoid_singularities(10) - this->qdot));
		this->redundantTaskSet = false;                                                             // Must be reset by user
		
		return least_squares(unconstrainedAccel, M, speed-offset, lowerBound, upperBound, initialGuess);
	}
	
	return this->M*qddot + get_feedback_linearization();                                            // M*qddot + C*qdot + g
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Track a Cartesian trajectory                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f        &pose,
                                           const Eigen::Matrix<float,6,1> &vel,
										   const Eigen::Matrix<float,6,1> &acc)
{
	return move_endpoint(acc                                                                        // Feedforward term
	                   + this->Dc*(vel - this->J*this->qdot)                                        // Velocity feedback
					   + this->Kc*get_pose_error(pose,get_endpoint_pose()));                        // Pose feedback
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Track a joint trajectory                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialDynamicControl::track_joint_trajectory(const Eigen::VectorXf &pos,
                                                             const Eigen::VectorXf &vel,
															 const Eigen::VectorXf &acc)
{
	if(pos.size() != this->n or vel.size() != this->n or acc.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIAL DYNAMIC CONTROL] track_joint_trajectory(): "
		          << "This robot has " << this->n << " joints, but "
				  << "the position argument had " << pos.size() << " elements, "
				  << "the velocity argument had " << vel.size() << " elements, "
				  << "and the acceleration argument had " << acc.size() << " elements." << std::endl;
	}
	else
	{
		return move_joints(acc                                                                      // Feedforward term
		                 + this->kd*(vel - this->qdot)                                              // Velocity feedback
					     + this->kp*(pos - this->q);                                                // Position feedback
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //            Compute the instantaneous acceleration bounds for joint limit avoidance            //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool compute_acceleration_bounds(float &lower, float &upper, const unsigned int &jointNumber)
{
	if(jointNumber > this->n)
	{
		std::cerr << "[ERROR] [SERIAL DYNAMIC CONTROL] compute_acceleration_bounds(): "
		          << "You called for " << jointNumber << ", but this robot only has "
				  << this->n << " joints." << std::endl;
				  
		return false;
	}
	else
	{
		float hertzSquared = this->hertz*this->hertz; // 1/dt^2
		
		lower = std::max(2*( this->posLimit[i][0] - this->q[i] - this->dt*this->qdot[i])*hertzSquared,
		        std::max(  (-this->velLimit[i] - this->qdot[i] )*this->hertz,
				            -this->maxAcceleration ));
	
		upper = std::min(2*( this->posLimit[i][1] - this->q[i] - this->dt*this->qdot[i])*hertzSquared,
		        std::min(  ( this->velLimit[i] - this->qdot[i] )*this->hertz,
				             this->maxAcceleration));
		
		return true;
	}
}