  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Solve the joint velocities to achieve the endpoint velocity                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::get_cartesian_control(const Eigen::VectorXf &vel)
{
	if(primary_task_is_ok(vel))
	{
		if(this->n <= 6) return rmrc(vel, Eigen::VectorXf::Zero(this->n));                 // Not redundant, secondary task doesn't matter
		else             return rmrc(vel, singularity_avoidance(5.0));                     // Apply automatic singularity avoidance
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
		Eigen::VectorXf xdot = vel + this->k*get_pose_error(pose, this->FK(end));
		
		if(this->n <= 6) return rmrc(xdot, Eigen::VectorXf::Zero(this->n));                // Not redundant, secondary task doesn't matter
		else             return rmrc(xdot, singularity_avoidance(5.0));                    // Apply automatic singularity avoidance
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
		return rmrc(vel + this->k*get_pose_error(pose, this->FK(end)), secondaryTask);
	}
	else	return Eigen::VectorXf::Zero(this->n);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    The full resolved motion rate control (rmrc) algorithm                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf SerialKinControl::rmrc(const Eigen::VectorXf &xdot, const Eigen::VectorXf qdot_d)
{
	// Whitney, D. E. (1969). Resolved motion rate control of manipulators and human prostheses.
	// IEEE Transactions on man-machine systems, 10(2), 47-53.
	
	// Solve the equation: qdot = invJ*xdot + (I - invJ*J)*qdot_d
	// If not redundant this reduces to qdot = invJ*xdot

	// Variables used in this scope
	Eigen::VectorXf qdot = Eigen::VectorXf::Zero(this->n);                                     // Value to be returned
	Eigen::MatrixXf J = get_jacobian();                                                        // xdot = J*qdot
	Eigen::MatrixXf invJ;                                                                      // J*invJ = I
	if(this->n <= 6) invJ = get_inverse(J);
	else		 invJ = get_weighted_inverse(J, this->M + get_joint_weighting());          // Weighted for energy, joint limit avoidance
	Eigen::MatrixXf N = Eigen::Matrix::Identity(this->n, this->n) - invJ*J;                    // For redundant robots, J*N = 0
	
	// Solve each individual joint velocity
	for(int i = 0; i < this->n; i++)
	{
		for(int j = 0; j < this->n; j++)
		{
			if(j < 6) qdot(i) += inv(i,j)*xdot(j);                                     // Range space velocities
			
			if(this->n > 6) qdot(i) += N(i,j)*qdot_d(j);                               // Null space velocities
		}
		
		limit_joint_velocity(qdot(i), i);                                                  // Ensure it is kinematically feasible
	}
	
	return qdot;
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
			  << "but it had " << vel.size() << " elements." << std::endl;
		
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
	else if(secondaryTask.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_cartesian_control(): "
			  << "Expected a " this->n << "x1 vector for the secondary task "
			  << "but it had " << secondaryTask.size() << " elements." << std::endl;
		
		return false;
	}
	else	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //         Limit the joint velocities to obey position, speed, acceleration constraints           //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialKinControl::limit_joint_velocity(double &qdot, const int &i)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2012, May). Motion control of redundant robots
	// under joint constraints: Saturation in the null space.
	// In 2012 IEEE International Conference on Robotics and Automation (pp. 285-292). IEEE.
	
	float lower = std::max({ (this->pLim[i][0] - this->q[i])/this->dt,                         // Position constraint
                                 -this->vLim[i],                                                   // Speed constraint
                                 -sqrt(2*this->aLim[i]*(this->q[i] - this->pLim[i][0])) });        // Acceleration constraint
        
        float upper = std::min({ (this->pLim[i][1] - this->q[i])/this->dt,                         // Position constraint
                                  this->vLim[i],                                                   // Speed constraint
                                  sqrt(2*this->aLim[i]*(this->pLim[i][1] - this->q[i])) });        // Acceleration constraint

	if(qdot < lower)
	{
		qdot = lower + 0.001                                                               // Just above the lower limit
		return true;
	}
	else if(qdot > upper)
	{
		qdot = upper - 0.001;                                                              // Just below the upper limit
		return true;
	}
	else	return false;
}
