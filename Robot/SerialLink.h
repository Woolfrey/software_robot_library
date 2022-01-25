#ifndef SERIAL_LINK_H_									// If not yet defined...
#define SERIAL_LINK_H_									// ... include this header file, otherwise ignore

#include <array>									// std::array
#include <Eigen/Geometry>								// Transforms (Isometry3f)
#include <Link.h>									// Custom link class
#include <vector>									// std::vector

class SerialLink
{
	public:
		// Constructors(s)
		SerialLink() {}							// Empty constructor
		
		SerialLink(const std::vector<Link> &links,				// Proper constructor
			const Eigen::Isometry3f &baseTransform,
			const Eigen::Isometry3f &finalTransform);
			
		// Set Functions
		void set_endpoint_offset(const Eigen::Isometry3f &transform) {this->endpointTF = transform;} // Define a new endpoint
		void set_gravity_vector(const Eigen::Vector3f &gravity) {this->gravityVector = gravity;}	// Set a new gravitational vector
		bool update_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel);			// Update internal kinematics & dynamics
		
		// Get Functions
		bool get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel);			// Get the current internal joint state
		Eigen::Isometry3f get_endpoint_pose() const {return this->fkChain[this->n];}		// Get the pose of the endpoint
		Eigen::MatrixXf get_coriolis() const {return this->C;}				// Get the Coriolis matrix
		Eigen::MatrixXf get_inertia() const {return this->M;}				// Get inerita matrix in joint space
		Eigen::MatrixXf get_jacobian() {return get_jacobian(this->fkChain[this->n].translation(), this->n);}
		Eigen::MatrixXf get_partial_derivative(const Eigen::MatrixXf &J, const int &jointNum); // Partial derivative w.r.t a single joint
		Eigen::MatrixXf get_time_derivative(const Eigen::MatrixXf &J);			// Time derivative of a given Jacobian
		Eigen::VectorXf get_gravity_torque() const {return this->g;}				// Get torque needed to oppose gravity
		Eigen::VectorXf get_joint_positions() const {return this->q;}			// As it says on the label
		float get_joint_position(const int &i) const {return this->q[i];}			// Query a single joint position
		float get_joint_velocity(const int &i) const {return this->qdot[i];}			// Query a single joint velocity
		int get_number_of_joints() const {return this->n;}					// Returns the number of joints
		std::vector<std::array<float,2>> get_position_limits();				// Get all joint limits as a single object
		std::vector<float> get_velocity_limits();						// Get all velocity limits as a single object
			
	private:
		// Kinematic properties
		int n;									// Number of joints
		Eigen::Vector3f gravityVector = {0.0, 0.0, -9.81};			// Gravitational acceleration
		Eigen::VectorXf q, qdot;						// Joint positions, velocities
		Eigen::Isometry3f baseTF;						// Transform to first joint
		Eigen::Isometry3f finalTF;						// Transform from final joint to endpoint
		Eigen::Isometry3f endpointTF;						// New endpoint offset (default identity)
		std::vector<Link> link;						// Array of link objects
		std::vector<Eigen::Isometry3f> fkChain;				// Transforms for each link
		std::vector<Eigen::Vector3f> axis;					// Axis of actuation for each joint, in base frame

		// Dynamic properties
		Eigen::MatrixXf C;							// Coriolis matrix (nxn)
		Eigen::MatrixXf D;							// Damping matrix (nxn)
		Eigen::MatrixXf M;							// Inertia matrix (nxn)
		Eigen::VectorXf g;							// Gravitational torque (nx1)

		// Get Functions
		Eigen::MatrixXf get_jacobian(const Eigen::Vector3f &point, const int &numJoints);
		
		// Update internal state
		// Note: MUST update kinematics first
		void update_forward_kinematics();					// Compute forward kinematic chain based on current state
		void update_inverse_dynamics();					// Compute inverse dynamics properites based on current state
		
};											// Semicolon needed after a class declaration

/******************** Constructor ********************/
SerialLink::SerialLink(const std::vector<Link> &links,
		const Eigen::Isometry3f &baseTransform,
		const Eigen::Isometry3f &finalTransform)
		: n(links.size()-1)							// Number of joints
		, q(Eigen::VectorXf::Zero(this->n))					// Joint position vector
		, qdot(Eigen::VectorXf::Zero(this->n))				// Joint velocity vector
		, baseTF(baseTransform)						// Pose of the base in global frame
		, finalTF(finalTransform)						// Pose of the endpoint from the final link frame
		, endpointTF(Eigen::Isometry3f::Identity())				// Additional endpoint offset
		, link(links)								// Vector of Link objects
		, C(Eigen::MatrixXf::Zero(this->n, this->n))				// Coriolis matrix
		, D(Eigen::MatrixXf::Identity(this->n, this->n))			// Damping matrix
		, M(Eigen::MatrixXf::Identity(this->n, this->n))			// Inertia matrix
		, g(Eigen::VectorXf::Zero(this->n))					// Gravity torque vectors
		
{
	// Initialize all std::vector objects
	this->fkChain.push_back(Eigen::Isometry3f::Identity());
	
	for(int i = 0; i < this->n; i++)
	{
		this->fkChain.push_back(Eigen::Isometry3f::Identity());		// Link / joint transforms
		this->axis.push_back(Eigen::Vector3f::Zero());			// Axis of actuation
	}

	update_state(this->q, this->qdot);						// Set the initial state
}

/******************** Update all the internal kinematic & dynamic properties ********************/
bool SerialLink::update_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() == vel.size() && pos.size() == this->n)			// Dimension of all arguments are correct
	{
		this->q = pos;								// Assign new joint positions
		this->qdot = vel;							// Assign new joint velocities		
		update_forward_kinematics();						// Compute new transform chain
		update_inverse_dynamics();						// Compute inertia, coriolis, gravity
		return true;
	}
	else
	{
		std::cerr << "[ERROR][SERIALLINK] set_joint_state() : Length of input vectors is not correct." << std::endl;
		std::cerr << "Number of joints: " << this->n << " input pos: " << pos.size() << " input vel: " <<  vel.size() << std::endl;
		return false;
	}
}

/******************** Get the current joint position, velocites ********************/
bool SerialLink::get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel)
{
	if(pos.size() != vel.size() || pos.size() != this->n)
	{
		std::cerr << "[ERROR][SERIALLINK] get_joint_state() : Input vectors are not the correct length." << std::endl;
		std::cerr << " No. of joints: " << this->n << " pos: " << pos.size() << " vel: " << vel.size() << std::endl;
		
		pos.setZero();
		vel.setZero();
		
		return false;
	}
	else
	{
		pos = this->q;
		vel = this->qdot;
		
		return true;
	}
}

/******************** Get the joint position limits as a single object ********************/
std::vector<std::array<float,2>> SerialLink::get_position_limits()
{
	std::vector<std::array<float,2>> limits;						// Value to be returned
	limits.resize(this->n);								// Resize array accordingly
	for(int i = 0; i < this->n; i++)
	{
		this->link[i].get_position_limits(limits[i][0], limits[i][1]);
	}
	return limits;
}

/******************** Get the joint velocity limits as a single object ********************/
std::vector<float> SerialLink::get_velocity_limits()
{
	std::vector<float> limits;
	limits.resize(this->n);
	for(int i = 0; i < this->n; i++) limits[i] = this->link[i].get_velocity_limit();
	return limits;

}

/******************** Compute forward kinematics chain at current joint position ********************/
void SerialLink::update_forward_kinematics()
{	
	this->fkChain[0] = this->baseTF*this->link[0].get_pose(this->q[0]); 			// Pre-multiply by base transform
	this->axis[0] = this->fkChain[0].rotation()*this->link[0].get_axis();		// Update axis for 1st joint	
	
	for(int i = 1; i < this->n; i++)
	{
		this->fkChain[i] = this->fkChain[i-1]*this->link[i].get_pose(this->q[i]);	// Compute chain of joint/link transforms
		this->axis[i] = this->fkChain[i].rotation()*this->link[i].get_axis();	// Rotate axis from local frame to global frame
	}
	this->fkChain[this->n] = this->fkChain[this->n-1]*this->link[this->n].get_pose(0)*this->endpointTF; // Offset the endpoints
}

/******************** Compute inverse dynamics for current joint state ********************/
void SerialLink::update_inverse_dynamics()
{
	// Variables used in this scope
	Eigen::Matrix3f R, I, Idot, sdot_temp;						// Rotation matrix and moment of inertia
	Eigen::MatrixXf Jc, Jcdot, Jv, Jw;							// Jacobian to c.o.m.
	Eigen::Vector3f com, omega, sdot;							// c.o.m. and angular velocity vector
	float m;										// Link mass
	
	// Set initial values
	this->M.setZero();
	this->C.setZero();
	this->g.setZero();
	omega.setZero();									// Initial angular velocity
	
	// Compute dynamics
	for(int i = 0; i < this->n; i++)
	{
		// Note to self:
		// link[i+1] is used because link[0] is the static base link
		com = this->fkChain[i]*this->link[i+1].get_com();				// Transform c.o.m. of ith link to global frame
		
		I = this->fkChain[i].rotation()						// Rotation inertia to global frame
		   *this->link[i+1].get_inertia()
		   *this->fkChain[i].rotation().transpose();
		  
		Jc = get_jacobian(com, i+1);							// Get the Jacobian to the ith c.o.m.
		Jv = Jc.block(0,0,3,i+1);							// Makes calcs a little easier
		Jw = Jc.block(3,0,3,i+1);							// Makes calcs a little easier
		m = this->link[i+1].get_mass();							// Get the mass of the ith link
		
		this->g.head(i+1) -= m*Jv.transpose()*this->gravityVector;			// tau = Jc'*(m*a) NOTE: Need to NEGATE gravity
		
		this->M.block(0,0,i+1,i+1) += m*Jv.transpose()*Jv + Jw.transpose()*I*Jw;	// M = Jc'*I*Jc

		Jcdot = get_time_derivative(Jc);						// Get time derivative of Jacobian
		if(this->link[i+1].is_revolute()) omega += this->qdot[i]*this->axis[i]; 	// Compute angular velocity up the chain
		
		// Idot =  skew(omega)*I
		if (i == 0)
			Idot.setZero();
		else
			Idot << omega(1)*I(2,0)-omega(2)*I(1,0), omega(1)*I(2,1)-omega(2)*I(1,1), omega(1)*I(2,2)-omega(2)*I(1,2),
					omega(2)*I(0,0)-omega(0)*I(2,0), omega(2)*I(0,1)-omega(0)*I(2,1), omega(2)*I(0,2)-omega(0)*I(2,2),
					omega(0)*I(1,0)-omega(1)*I(0,0), omega(0)*I(1,1)-omega(1)*I(0,1), omega(0)*I(1,2)-omega(1)*I(0,2);
		
		this->C.block(0,0,i+1,i+1) += m*Jv.transpose()*Jcdot.block(0,0,3,i+1)	// C = Jc'*(I*Jcdot + Idot*Jc);
					     + Jw.transpose()*(I*Jcdot.block(3,0,3,i+1) + Idot*Jw);
	}
}

/******************** Compute a Jacobian matrix to any given point ********************/
Eigen::MatrixXf SerialLink::get_jacobian(const Eigen::Vector3f &point, const int &numJoints)
{
	Eigen::MatrixXf J;									// Value to be returned
	J.setZero(6,numJoints);
	
	if(numJoints < 0)
	{
		std::cout << "ERROR: Can't compute the Jacobian for " << numJoints << " joints!" << std::endl;
	}
	else
	{
		for(int i = 0; i < numJoints; i++)
		{
			if(this->link[i].is_revolute())					// Revolute joint
			{
				J.block(0,i,3,1) = this->axis[i].cross(point - this->fkChain[i].translation()); // a_i x r_i
				J.block(3,i,3,1) = this->axis[i];				// a_i
			}
			else									// Prismatic joint
			{
				J.block(0,i,3,1) = this->axis[i];				// a_i
	//			J.block(3,i,3,1) = zeros					// 0
			}
		}
	}
	return J;
}

/******************** Compute the time derivative of a given Jacobian ********************/
Eigen::MatrixXf SerialLink::get_time_derivative(const Eigen::MatrixXf &J)
{
	Eigen::MatrixXf Jdot;									// Value to be returned
	Jdot.setZero(6,J.cols());

	for(int i = 0; i < J.cols(); i++)
	{
		for(int j = 0; j <= i; j++)
		{
			// Compute dJ(i)/dq(j)
			if(this->link[j+1].is_revolute())					// J_j = [a_j x r_j; a_j]
			{
				// qdot_j * ( a_j x (a_i x r_i) )
				Jdot(0,i) += this->qdot(j)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
				Jdot(1,i) += this->qdot(j)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
				Jdot(2,i) += this->qdot(j)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				
				if(this->link[i+1].is_revolute())				// J_i = [a_i x r_i; a_i]
				{
					// qdot_j * ( a_j x a_i )
					Jdot(3,i) += this->qdot(j)*(J(4,j)*J(5,i) - J(5,j)*J(4,i));
					Jdot(4,i) += this->qdot(j)*(J(5,j)*J(3,i) - J(3,j)*J(5,i));
					Jdot(5,i) += this->qdot(j)*(J(3,j)*J(4,i) - J(4,j)*J(3,i));
				}	
			}
			
			// Compute dJ(j)/dq(i)
			if(i != j && this->link[j+1].is_revolute())				// J_j = [a_j x r_j; a_j]
			{
				if(this->link[i+1].is_revolute())				// J_i = [a_i x r_i; a_i]
				{
					// qdot_i * ( a_i x (a_j x r_j) )
					// Jdot(0,j) += this->qdot(i)*(J(4,i)*J(2,j) - J(5,i)*J(1,j));
					// Jdot(1,j) += this->qdot(i)*(J(5,i)*J(0,j) - J(3,i)*J(2,j));
					// Jdot(2,j) += this->qdot(i)*(J(3,i)*J(1,j) - J(4,i)*J(0,j));
					Jdot(0,j) += this->qdot(i)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
					Jdot(1,j) += this->qdot(i)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
					Jdot(2,j) += this->qdot(i)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				}
				else // this->link[i].is_prismatic()				// J_i = [a_i ; 0]
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

/******************** Get the partial derivative of the Jacobian w.r.t. to the ith joint  ********************/
Eigen::MatrixXf SerialLink::get_partial_derivative(const Eigen::MatrixXf &J, const int &jointNum)
{
	// E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
	// Proceedings. 1991 IEEE International Conference on Robotics and Automation,
	// 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
	
	Eigen::MatrixXf dJ(6,J.cols());							// Value to be returned
	dJ.setZero();

	if(jointNum > J.cols())
	{
		std::cout << "ERROR: SerialLink::get_partial_jacobian() : Cannot compute the partial derivative with respect to joint "
			<< jointNum << " because the Jacobian only has " << J.cols() << " columns!" << std::endl;
		return dJ;
	}
	else
	{
		for(int i = 0; i < J.cols(); i++)
		{
			if(this->link[i].is_revolute())					// J_i = [a_i x r_i ; a_i]
			{
				if(this->link[jointNum].is_revolute()) 			// J_i = [a_j x r_j; a_j]
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
				else if(this->link[jointNum].is_prismatic() && jointNum > i)	// J_j = [a_j ; 0]
				{
					// a_j x a_i
					dJ(0,i) = J(1,jointNum)*J(2,i) - J(2,jointNum)*J(1,i);
					dJ(1,i) = J(2,jointNum)*J(0,i) - J(0,jointNum)*J(2,i);
					dJ(2,i) = J(0,jointNum)*J(1,i) - J(1,jointNum)*J(0,i);
				}
			}
			else if(this->link[i].is_prismatic()					// J_i = [a_i ; 0]
				&& this->link[jointNum].is_revolute()				// J_j = [a_j x r_j; a_j]
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

/********************************************************************************************
*					LEGACY CODE 						*
*********************************************************************************************/

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

#endif
