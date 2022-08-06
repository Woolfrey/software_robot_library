#include <SerialLink.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialLink::SerialLink(const std::vector<RigidBody> &links,
                       const std::vector<Joint> &joints):
                       n(joints.size()),                                                            // Number of joints
                       joint(joints),                                                               // Vector of Joint objects
                       q(Eigen::VectorXf::Zero(this->n)),                                           // Joint position vector
                       qdot(Eigen::VectorXf::Zero(this->n)),                                        // Joint velocity vector
                       base(links[0]),                                                              // Robot base as a RigidBody object
                       C(Eigen::MatrixXf::Zero(this->n, this->n)),                                  // Coriolis matrix
                       D(Eigen::MatrixXf::Identity(this->n, this->n)),                              // Damping matrix
                       M(Eigen::MatrixXf::Identity(this->n, this->n)),                              // Inertia matrix
                       g(Eigen::VectorXf::Zero(this->n)),                                           // Gravity torque vectors
                       baseTF(Eigen::Isometry3f::Identity()),                                       // Transform of base w.r.t world frame
                       endpointTF(Eigen::Isometry3f::Identity())                                    // Additional offset for the endpoint
{
	if(links.size() != joints.size() + 1)
	{
		std::cerr << "[ERROR] [SERIALLINK] Constructor : Object requires n+1 links for n joints. "
			<< "No. of links: " << links.size() << " no. of joints: " << joints.size() << std::endl;
		this->isValid = false;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{	
			this->link.push_back(links[i+1]);
			this->fkChain.push_back(Eigen::Isometry3f::Identity());                     // Transforms between links/joints
			this->axis.push_back(Eigen::Vector3f::Zero());                              // Axis of actuation for each joint
		}
		this->fkChain.push_back(Eigen::Isometry3f::Identity());                             // Extra transform to endpoint
		
		set_joint_state(this->q, this->qdot);                                               // Set initial state
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Update all the internal kinematics & dynamics                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLink::set_joint_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() != this->n or vel.size() != this->n)                                          // Dimension of all arguments are incorrect
	{
		std::cerr << "[ERROR] [SERIALLINK] set_joint_state(): "
			  << "Expected " << this->n <<"x1" << " vectors for the input arguments but "
		          << "the position vector had " << pos.size() << " elements and "
		          << "the velocity vector had " << vel.size() << " elements." << std::endl;
		          
		return false;
	}
	else
	{
		this->q = pos;                                                                      // Assign new joint positions
		this->qdot = vel;                                                                   // Assign new joint velocities		
//		update_forward_kinematics();                                                        // Compute new transform chain
//		update_inverse_dynamics();                                                          // Compute inertia, coriolis, gravity

		// Variables used in this scope
		Eigen::Matrix3f A     = Eigen::Matrix3f::Identity();                                // Cartesian mass-inertia matrix
		Eigen::Matrix3f Adot  = Eigen::Matrix3f::Zero();                                    // Time-derivative of inertia
		Eigen::MatrixXf Jc, Jcdot, Jv, Jw;                                                  // Jacobian to center of mass
		Eigen::Vector3f com   = Eigen::Vector3f::Zero();                                    // Location of center of mass for a link
		Eigen::Vector3f omega = Eigen::Vector3f::Zero();                                    // Angular velocity of a link
		float m = 0.0;                                                                      // Mass of an individual link
		
		// Reset dynamic properties
		this->M.setZero();                                                                  // Inertia matrix
		this->C.setZero();                                                                  // Coriolis matrix
		this->g.setZero();                                                                  // Gravity vector
			
		// Compute the kinematic & dynamic properties for the given joint state		
		for(int i = 0; i < this->n; i++)
		{
			////////////////////////////// KINEMATICS //////////////////////////////////
			if(i == 0) this->fkChain[i] = this->base.get_pose()*this->joint[i].get_pose(this->q[i]);
			else       this->fkChain[i] = this->fkChain[i-1]*this->joint[i].get_pose(this->q[i]);
	
			this->axis[i] = this->fkChain[i].rotation()*this->joint[i].get_axis();
			
			////////////////////////////// DYNAMICS ////////////////////////////////////
		 	// A (convoluted) derivation can be found here:
        		// Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        		// theory, methods, and algorithms (4th Edition),
        		// New York, NY: Springer New York. pages 306 - 315
        		
			com = this->fkChain[i]*this->link[i].get_com();                             // Transform the com to the global frame
			  
			Jc = get_jacobian(com,i+1);                                                 // Get the Jacobian to the ith center of mass
			Jv = Jc.block(0,0,3,i+1);                                                   // Linear velocity component
			Jw = Jc.block(3,0,3,i+1);                                                   // Angular velocity component
			
			m = this->link[i].get_mass();                                               // Mass of the ith link
			
			this->g.head(i+1) -= m*Jv.transpose()*this->gravityVector;                  // We need to *negate* the effect of gravity
			
			A = this->fkChain[i].rotation()
			  * this->link[i].get_inertia()
			  * this->fkChain[i].rotation().inverse();                                  // Rotate inertia matrix to global frame
			
			this->M.block(0,0,i+1,i+1) += m*Jv.transpose()*Jv + Jw.transpose()*A*Jw;    // Accrue inertia
		
			Jcdot = get_time_derivative(Jc);                                            // As it says on the label
			
			if(this->joint[i].is_revolute()) omega += this->qdot[i]*this->axis[i];      // Update cumulative angular velocity
			
			if(i > 0)
			{
				// Adot = skew(omega)*A, but we can skip all the zeros by doing it manually:
				Adot << omega(1)*A(2,0)-omega(2)*A(1,0), omega(1)*A(2,1)-omega(2)*A(1,1), omega(1)*A(2,2)-omega(2)*A(1,2),
                                       omega(2)*A(0,0)-omega(0)*A(2,0), omega(2)*A(0,1)-omega(0)*A(2,1), omega(2)*A(0,2)-omega(0)*A(2,2),
                                       omega(0)*A(1,0)-omega(1)*A(0,0), omega(0)*A(1,1)-omega(1)*A(0,1), omega(0)*A(1,2)-omega(1)*A(0,2);
                        }
			
			this->C.block(0,0,i+1,i+1) += m*Jv.transpose()*Jcdot.block(0,0,3,i+1)
                                                   + Jw.transpose()*(A*Jcdot.block(3,0,3,i+1) + Adot*Jw);
		}
		
		// Update transform from last joint to endpoint
		this->fkChain[this->n] = this->fkChain[this->n-1]
                                       * this->link[this->n-1].get_pose()
                                       * this->endpointTF;
	
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the joint state information                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLink::get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel)
{
	if(pos.size() != this->n or vel.size() != this->n)
	{
		std::cerr << "[ERROR] [SERIALLINK] get_joint_state(): "
			  << "Expected " << this->n << "x1" << " vectors for the input arguments but "
			  << "the position vector had " << pos.size() << " elements and "
			  << "the velocity vector had " << vel.size() << " elements." << std::endl;
		
		return false;
	}
	else
	{
		pos = this->q;
		vel = this->qdot;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Compute the forward kinematics chain                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
void SerialLink::update_forward_kinematics()
{	
	this->fkChain[0] = this->base.get_pose()*this->joint[0].get_pose(this->q[0]);               // Dynamic transform from joint position
	this->axis[0] = this->fkChain[0].rotation()*this->joint[0].get_axis();                      // Rotate axis to global frame
			
	for(int i = 1; i < this->n; i++)
	{
		this->fkChain[i] = this->fkChain[i-1]*this->joint[i].get_pose(this->q[i]);          // Dynamic joint transform at end of link
		this->axis[i] = this->fkChain[i].rotation()*this->joint[i].get_axis();
	}
		
	this->fkChain[this->n] = this->fkChain[this->n-1]                                           // Transform for final joint
				*this->link[this->n-1].get_pose()                                   // Transform for final link
				*this->endpointTF;                                                  // Additional offset of the endpoint
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Compute the dynamics properties                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
void SerialLink::update_inverse_dynamics()
{
	// A (convoluted) derivation can be found here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (4th Edition),
        // New York, NY: Springer New York. pages 306 - 315
        
	// Variables used in this scope
	Eigen::Matrix3f R, I, Idot, sdot_temp;                                                      // Rotation matrix and moment of inertia
	Eigen::MatrixXf Jc, Jcdot, Jv, Jw;                                                          // Jacobian to c.o.m.
	Eigen::Vector3f com, omega, sdot;                                                           // c.o.m. and angular velocity vector
	float m;                                                                                    // Link mass
	
	// (Re)set values
	this->M.setZero();
	this->C.setZero();
	this->g.setZero();
	omega.setZero();                                                                            // Initial angular velocity
	
	// Compute dynamics
	for(int i = 0; i < this->n; i++)
	{
		com = this->fkChain[i]*this->link[i].get_com();                                     // Transform c.o.m. of ith link to global frame
		
		I = this->fkChain[i].rotation()                                                     // Rotation inertia to global frame
		   *this->link[i].get_inertia()
		   *this->fkChain[i].rotation().transpose();

		Jc = get_jacobian(com, i+1);                                                        // Get the Jacobian to the ith c.o.m.
		Jv = Jc.block(0,0,3,i+1);                                                           // Makes calcs a little easier
		Jw = Jc.block(3,0,3,i+1);                                                           // Makes calcs a little easier
		m = this->link[i].get_mass();                                                       // Get the mass of the ith link
		
		this->g.head(i+1) -= m*Jv.transpose()*this->gravityVector;                          // tau = Jc'*(m*a) NOTE: Need to NEGATE gravity
		
		this->M.block(0,0,i+1,i+1) += m*Jv.transpose()*Jv + Jw.transpose()*I*Jw;            // M = Jc'*I*Jc

		Jcdot = get_time_derivative(Jc);                                                    // Get time derivative of Jacobian
		if(this->joint[i].is_revolute()) omega += this->qdot[i]*this->axis[i];              // Compute angular velocity up the chain
		
		// Idot =  skew(omega)*I
		if (i == 0)
			Idot.setZero();
		else
			Idot << omega(1)*I(2,0)-omega(2)*I(1,0), omega(1)*I(2,1)-omega(2)*I(1,1), omega(1)*I(2,2)-omega(2)*I(1,2),
                               omega(2)*I(0,0)-omega(0)*I(2,0), omega(2)*I(0,1)-omega(0)*I(2,1), omega(2)*I(0,2)-omega(0)*I(2,2),
                               omega(0)*I(1,0)-omega(1)*I(0,0), omega(0)*I(1,1)-omega(1)*I(0,1), omega(0)*I(1,2)-omega(1)*I(0,2);
		
		this->C.block(0,0,i+1,i+1) += m*Jv.transpose()*Jcdot.block(0,0,3,i+1)               // C = Jc'*(I*Jcdot + Idot*Jc);
                                           + Jw.transpose()*(I*Jcdot.block(3,0,3,i+1) + Idot*Jw);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialLink::get_jacobian(const Eigen::Vector3f &point, const int &numJoints)
{
	// Whitney, D. E. (1972). The mathematics of coordinated control of prosthetic arms and manipulators.
	// Journal of Dynamic  Systems, Measurement,  and  Control, pp. 303-309
	
	Eigen::MatrixXf J = Eigen::MatrixXf::Zero(6,numJoints);                                     // Value to be returned
	
	if(numJoints <= 0)
	{
		std::cerr << "[ERROR] [SERIALLINK] get_jacobian(): "
			  << "Cannot compute the Jacobian for " << numJoints << " joints!" << std::endl;
	}
	else if(numJoints > this->n)
	{
		std::cerr << "[ERROR] [SERIALLINK] get_jacobian(): "
			  << "Cannot compute the Jacobian for " << numJoints << "joints "
			  << "because this model only has " << this->n << "!" << std::endl;
	}
	else
	{
		for(int i = 0; i < numJoints; i++)
		{
			if(this->joint[i].is_revolute())                                            // Revolute joint
			{
				J.block(0,i,3,1) = this->axis[i].cross(point - this->fkChain[i].translation()); // a_i x r_i
				J.block(3,i,3,1) = this->axis[i];                                   // a_i
			} 
			else                                                                        // Prismatic joint
			{
				J.block(0,i,3,1) = this->axis[i];                                   // a_i
//				J.block(3,i,3,1) = zeros                                            // 0
			}
		}
	}
	
	return J;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Compute the time derivative of a given Jacobian matrix                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialLink::get_time_derivative(const Eigen::MatrixXf &J)
{
	// See the following for the partial derivative:
	// E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
	// Proceedings. 1991 IEEE International Conference on Robotics and Automation,
	// 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
	
	if(J.rows() != 6)
	{
		std::cerr << "[ERROR] [SERIALLINK] get_time_derivative() : Expected a 6xn Jacobian, "
			<< "but your input only had " << J.rows() << " rows." << std::endl;
			
		return Eigen::MatrixXf::Zero(J.rows(), J.cols());
	}
	else
	{
		Eigen::MatrixXf Jdot;                                                               // Value to be returned
		Jdot.setZero(6,J.cols());

		for(int i = 0; i < J.cols(); i++)
		{
			for(int j = 0; j <= i; j++)
			{
				// Compute dJ(i)/dq(j)
				if(this->joint[j].is_revolute())                                    // J_j = [a_j x r_j; a_j]
				{
					// qdot_j * ( a_j x (a_i x r_i) )
					Jdot(0,i) += this->qdot(j)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
					Jdot(1,i) += this->qdot(j)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
					Jdot(2,i) += this->qdot(j)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
					
					if(this->joint[i].is_revolute())                            // J_i = [a_i x r_i; a_i]
					{
						// qdot_j * ( a_j x a_i )
						Jdot(3,i) += this->qdot(j)*(J(4,j)*J(5,i) - J(5,j)*J(4,i));
						Jdot(4,i) += this->qdot(j)*(J(5,j)*J(3,i) - J(3,j)*J(5,i));
						Jdot(5,i) += this->qdot(j)*(J(3,j)*J(4,i) - J(4,j)*J(3,i));
					}	
				}
				
				// Compute dJ(j)/dq(i)
				if(i != j && this->joint[j].is_revolute())                          // J_j = [a_j x r_j; a_j]
				{
					if(this->joint[i].is_revolute())                            // J_i = [a_i x r_i; a_i]
					{
						// qdot_i * ( a_i x (a_j x r_j) )
						Jdot(0,j) += this->qdot(i)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
						Jdot(1,j) += this->qdot(i)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
						Jdot(2,j) += this->qdot(i)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
					}
					else // this->link[i].is_prismatic()                        // J_i = [a_i ; 0]
					{
						// qdot_i * ( a_i x (a_j x r_j) )
						Jdot(0,j) += this->qdot(i)*(J(1,i)*J(2,j) - J(2,i)*J(1,j));
						Jdot(1,j) += this->qdot(i)*(J(2,i)*J(0,j) - J(0,i)*J(2,j));
						Jdot(2,j) += this->qdot(i)*(J(0,i)*J(1,j) - J(1,i)*J(0,j));
					}
				}
			}
		}
		
		return Jdot;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //    Compute the partial derivative of a given Jacobian with respect to the given joint number   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf SerialLink::get_partial_derivative(const Eigen::MatrixXf &J, const int &jointNum)
{
	// E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
	// Proceedings. 1991 IEEE International Conference on Robotics and Automation,
	// 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
	
	if(J.rows() != 6)
	{
		std::cerr << "[ERROR] [SERIALLINK] get_partial_derivative() : Expected a 6xn Jacobian, "
			<< "but your input only had " << J.rows() << " rows." << std::endl;
		return Eigen::MatrixXf::Zero(J.rows(), J.cols());
	}
	else if(jointNum > J.cols())
	{
		std::cerr << "[ERROR] [SERIALLINK] get_partial_derivative() : Cannot compute the partial derivative with respect to "
			<< "joint " << jointNum << " because the Jacobian only has " << J.cols() << " columns." << std::endl;
		
		return Eigen::MatrixXf::Zero(J.rows(), J.cols());
	}
	else
	{ 
		Eigen::MatrixXf dJ(6,J.cols());                                                     // Value to be returned
		dJ.setZero();
		
		for(int i = 0; i < J.cols(); i++)
		{
			if(this->joint[i].is_revolute())                                            // J_i = [a_i x r_i ; a_i]
			{
				if(this->joint[jointNum].is_revolute())                             // J_i = [a_j x r_j; a_j]
				{
					if (jointNum < i)
					{
						// a_j x (a_i x a_i)
						dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
						dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
						dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);

						// a_j x a_i
						dJ(3,i) = J(4,jointNum)*J(5,i) - J(5,jointNum)*J(4,i);
						dJ(4,i) = J(5,jointNum)*J(3,i) - J(3,jointNum)*J(5,i);
						dJ(5,i) = J(3,jointNum)*J(4,i) - J(4,jointNum)*J(3,i);
					}
					else
					{
						// a_i x (a_j x a_j)
						dJ(0,i) = J(4,i)*J(2,jointNum) - J(5,i)*J(1,jointNum);
						dJ(1,i) = J(5,i)*J(0,jointNum) - J(3,i)*J(2,jointNum);
						dJ(2,i) = J(3,i)*J(1,jointNum) - J(4,i)*J(0,jointNum);
					}
				}
				else if(this->joint[jointNum].is_prismatic() && jointNum > i)       // J_j = [a_j ; 0]
				{
					// a_j x a_i
					dJ(0,i) = J(1,jointNum)*J(2,i) - J(2,jointNum)*J(1,i);
					dJ(1,i) = J(2,jointNum)*J(0,i) - J(0,jointNum)*J(2,i);
					dJ(2,i) = J(0,jointNum)*J(1,i) - J(1,jointNum)*J(0,i);
				}
			}
			else if(this->joint[i].is_prismatic()                                       // J_i = [a_i ; 0]
                               and this->joint[jointNum].is_revolute()                             // J_j = [a_j x r_j; a_j]
                               and jointNum < i)
			{
				// a_j x a_i
				dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
				dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
				dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);
			}
		}
		return dJ;
	}
}
