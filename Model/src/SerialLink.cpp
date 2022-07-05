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
		Eigen::Matrix3f A = Eigen::Matrix3f::Identity();                                    // Cartesian mass-inertia matrix
		Eigen::Matrix3f Adot = Eigen::Matrix3f::Zero();                                     // Time-derivative of inertia
		Eigen::MatrixXf Jc, Jcdot, Jv, Jw;                                                  // Jacobian to center of mass
		Eigen::Vector3f com = Eigen::Vector3f::Zero();                                      // Location of center of mass for a link
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
	
	// Set initial values
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
						// Jdot(0,j) += this->qdot(i)*(J(4,i)*J(2,j) - J(5,i)*J(1,j));
						// Jdot(1,j) += this->qdot(i)*(J(5,i)*J(0,j) - J(3,i)*J(2,j));
						// Jdot(2,j) += this->qdot(i)*(J(3,i)*J(1,j) - J(4,i)*J(0,j));
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
				&& this->joint[jointNum].is_revolute()                              // J_j = [a_j x r_j; a_j]
				&& jointNum < i)
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

/********************************************************************************************
*					LEGACY CODE 						*
*********************************************************************************************/

/******************** Compute location for the c.o.m. for each link ********************
void SerialLink::update_com()
{
	for(int i = 0; i < this->n; i++)
	{
		this->com[i] = this->fkChain[i] * this->link[i+1].get_com();
	}
}

/******************** Rotate moment of inertia for each link to global frame ********************
void SerialLink::update_link_inertia()
{
	Eigen::AngleAxisf R;
	R = this->baseTF.rotation() * this->link[0].get_pose(this->q(0)).rotation(); 	// THIS IS OBSOLETE?
	R = this->fkChain[0].rotation();

	I[0] = R*this->link[1].get_inertia()*R.toRotationMatrix().transpose();

	for(int i = 0; i < this->n; ++i)
	{
		R = this->fkChain[i].rotation();
		I[i] = R*this->link[i+1].get_inertia()*R.toRotationMatrix().transpose();
	}
}



/******************** Returns the Jacobian matrix for the centre of mass for every link ********************
void SerialLink::update_com_jacobian()
{
	for(int i = 0; i < this->n; i++) this->Jc[i] = get_jacobian(this->com[i], i+1);	// Super easy now!
}

/******************** Compute the effect of gravity on the joints ********************
void SerialLink::update_gravity_torque()
{
	this->g.setZero(this->n);								// Set all the elements to zero
	
	for(int i = 0; i < this->n; i++)
	{
		// Since gravity points down, we need to negate its effects here
		this->g.head(i+1) -= this->link[i+1].get_mass()*this->Jc[i].block(0,0,3,i+1).transpose()*this->gravityVector;
	}
}
	

/******************** Compute inertia matrix in joint space ********************
void SerialLink::update_inertia()
{
	this->M.setZero(this->n, this->n);							// Zero all the elements

	for(int i = 0; i < this->n; i++)
	{	
		this->M.block(0,0,i+1,i+1) += this->link[i].get_mass()*this->Jc[i].block(0,0,3,i+1).transpose()*this->Jc[i].block(0,0,3,i+1)
						+ this->Jc[i].block(3,0,3,i+1).transpose()*this->I[i]*this->Jc[i].block(3,0,3,i+1);
	}
}

/******************** Compute kinematic properties of mechanism based on current joint position ********************
// Note: std::vector<Eigen::Vector3f> a is now updated inside the function update_forward_kinematics()
void SerialLink::update_axis_and_distance()
{
	for(int i = 0; i < this->n; i++)
	{
		this->axis[i] = this->fkChain[i].rotation()*this->link[i].get_axis();			// Rotate axis from local frame to global frame
		this->r[i] = this->fkChain[this->n].translation() - this->fkChain[i].translation();	// Distance from joint to end-effector
	}		
}

/******************** Compute cumulative angular velocity up the kinematic chain ********************
void SerialLink::update_omega()
{
	if(this->link[0].is_revolute())
	{
		w[0] = this->baseTwist.tail(3) + this->qdot(0)*a[0];
	}

	for(int i = 1; i < this->n; ++i)
	{
		w[i] = w[i-1];

		if(this->link[i].is_revolute())
			w[i] += this->qdot(i)*a[i];
	}
}

/******************** Returns the Jacobian matrix for the centre of mass for every link ********************
std::vector<Eigen::MatrixXf> SerialLink::get_mass_jacobian()
{
	std::vector<Eigen::MatrixXf> Jm;
	Eigen::Vector3f d(Eigen::Vector3f::Zero());
	Eigen::MatrixXf Jm_i;
	Jm_i.setZero(6,this->n);

	for(int i = 0; i < this->n; ++i) // 1->number joints
	{
		Jm.push_back(Jm_i);

		for(int j = 0; j <= i; ++j)
		{
			d = this->com[i] - this->fkChain[j].translation();

			if (link[i].is_revolute())
			{
				Jm[i].block(0,j,3,1) = this->axis[j].cross(d);
				Jm[i].block(3,j,3,1) = this->axis[j];
				Jm[i].col(j) << this->axis[j].cross(d), this->axis[j];
			}
			else
				Jm[i].block(0,j,3,1) = this->axis[j];
		}
	}
	return Jm;
}

/******************** Returns the gravity torque ********************
Eigen::VectorXf SerialLink::get_gravity_torque2()
{
	Eigen::VectorXf tau(Eigen::VectorXf::Zero(this->n));
	Eigen::Vector3f grav(0,0,-9.81);
	std::vector<Eigen::MatrixXf> Jm = get_mass_jacobian();

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < this->n; ++j)
		{
			for(int k = 0; k < this->n; ++k)
			{
				tau(j) = tau(j) - Jm[k](i,j)*grav(i)*this->link[k+1].get_mass();
			}
		}
	}

	return tau;
}

/******************** Returns the inertia matrix of the manipulator in the joint space ********************
Eigen::MatrixXf SerialLink::get_inertia2()
{
	Eigen::MatrixXf M = Eigen::MatrixXf::Zero(this->n, this->n);
	std::vector<Eigen::MatrixXf> Jm = get_mass_jacobian();
	std::vector<Eigen::MatrixX3f> linkInertia = this->I;				// Inertia matrices for each link in the origin frame


	for(int i = 0; i < this->n; ++i)
	{
		// Why this->link[i+1] ????
		M = M + this->link[i+1].get_mass()*Jm[i].block(0,0,3,this->n).transpose() * Jm[i].block(0,0,3,this->n)
				+ Jm[i].block(3,0,3,this->n).transpose() * linkInertia[i] * Jm[i].block(3,0,3,this->n);
	}

	return M;
}

/******************** Returns the Jacobian matrix ********************

NOTE: New Jacobian method seems to work. This can be moved outside the class in future.

Eigen::MatrixXf SerialLink::get_jacobian()
{
	// No input arguments, so get the Jacobian to the end-effector
	return get_jacobian(this->fkChain[this->n].translation(), this->n);

	Eigen::MatrixXf J;
	J.setZero(6,n);

	for(int i = 0; i < this->n; i++)
	{
		if(this->link[i].is_revolute())					// Revolute joint
		{
			J.block(0,i,3,1) = this->axis[i].cross(this->r[i]);		// a_i x r_i
			J.block(3,i,3,1) = this->axis[i];				// a_i
		}
		else									// Prismatic joint
		{
			J.block(0,i,3,1) = this->axis[i];				// a_i
			// J.block(3,i,3,1) = zeros
		}
	}

	return J;
} */