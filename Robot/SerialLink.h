/*
	TO DO:
		- Test the forward kinematics calculation is correct
		- Test that the Jacobian calculation is correct
		- Add some sort of error message (Line 124)
		
*/

#ifndef SERIAL_LINK_H_									// If not yet defined...
#define SERIAL_LINK_H_									// ... include this header file, otherwise ignore

#include <Eigen/Geometry>								// Transforms (Affine3f)
#include <Link.h>									// Custom link class
#include <vector>									// std::vector

class SerialLink
{
	public:
		// Constructors(s)
		SerialLink() {}							// Empty constructor
		
		SerialLink(const std::vector<Link> &links,				// Proper constructor
			const Eigen::Affine3f &baseTransform,
			const Eigen::Affine3f &finalTransform);
			
		// Set Functions
		bool set_joint_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel); // Update internal kinematics & dynamics
		void set_endpoint_offset(const Eigen::Affine3f &transform);		// Define a new endpoint from the original
		void set_gravity_vector(const Eigen::Vector3f &gravity);		// Set a new gravitational vector
		
		// Get Functions
		Eigen::Affine3f get_endpoint() const {return this->fkchain[this->n];} // Get the pose of the endpoint
		Eigen::MatrixXf get_jacobian() {return get_jacobian(this->fkchain[this->n].translation(), this->n);}
		Eigen::MatrixXf get_inertia();					// Get the inertia matrix of the manipulator
		Eigen::MatrixXf get_inertia2() const {return this->M;}		// TO REPLACE get_inertia()
		Eigen::VectorXf get_gravity_torque();					// As it says on the label	
		Eigen::VectorXf get_gravity_torque2() {return this->g;}		// TO REPLACE get_gravity_torque()	
		int get_number_of_joints() const {return this->n;}			// Returns the number of joints
		std::vector<Eigen::MatrixXf> get_com_jacobian() const {return this->Jc;} // TO REPLACE get_mass_jacobian()
		std::vector<Eigen::MatrixXf> get_mass_jacobian();			// Get the Jacobian to the c.o.m for every link
		void get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel);	// Get the current internal joint state


	private:
		// Variables
		int n;								// Number of joints
		Eigen::Vector3f gravityVector;				// Gravitational acceleration
		Eigen::VectorXf q, qdot;					// Joint positions, velocities
		Eigen::VectorXf baseTwist;					// Base twist
		Eigen::Affine3f baseTF;					// Transform to first joint
		Eigen::Affine3f finalTF;					// Transform from final joint to endpoint
		Eigen::Affine3f endpointTF;					// New endpoint offset (default identity)
		std::vector<Link> link;					// Array of link objects

		// Variable kinematic properties
		std::vector<Eigen::Affine3f> fkchain;				// Transforms for each link
		std::vector<Eigen::Vector3f> a;				// Axis of actuation for each joint, in base frame
		std::vector<Eigen::Vector3f> r;				// Translation from joint origin to endpoint
		std::vector<Eigen::Vector3f> w;				// N.B. This might be obsolete in the future!

		// Dynamic properties
		Eigen::MatrixXf C;						// Coriolis matrix (nxn)
		Eigen::MatrixXf D;						// Damping matrix (nxn)
		Eigen::MatrixXf M;						// Inertia matrix (nxn)
		Eigen::VectorXf g;						// Gravitational torque (nx1)
		std::vector<Eigen::Vector3f> com;				// Center of mass for each link, in base frame
		std::vector<Eigen::MatrixX3f> I;				// Inertia matrices for each link in the origin frame
		std::vector<Eigen::MatrixXf> Jc;				// Jacobian to the center of mass
		std::vector<Eigen::MatrixXf> Jcdot;				// Time derivative of Jacobian

		// Get Functions
		Eigen::MatrixXf get_jacobian(const Eigen::Vector3f &point, const int &numJoints);
		
		// Update Internal State
		// NOTE: Order which they are called is important!
		void update_forward_kinematics();				// Compute forward kinematic chain based on current state
		void update_com();						// Compute location for the c.o.m. for each link
		void update_com_jacobian();					// Compute the Jacobian to each c.o.m.
		void update_gravity_torque();					// Compute the effect of gravity on the joints
		void update_link_inertia();					// Rotate link inertia from local frame to base frame
		void update_inertia();						// Compute new inertia matrix in joint space
		
		// Old functions to be made obsolete:
		void update_axis_and_distance();				// Compute kinematic properties of mechanism for current stat
		void update_omega();						// Compute angular velocity up the kinematic chain
		
		
};										// Semicolon needed after a class declaration

/******************** Constructor ********************/
SerialLink::SerialLink(const std::vector<Link> &links,
		const Eigen::Affine3f &baseTransform,
		const Eigen::Affine3f &finalTransform)
		:
		n(links.size()-1),
		gravityVector(Eigen::Vector3f(0.0, 0.0, -9.81)),
		q(Eigen::VectorXf::Zero(this->n)),
		qdot(Eigen::VectorXf::Zero(this->n)),
		baseTwist(Eigen::VectorXf::Zero(6)),
		baseTF(baseTransform),
		finalTF(finalTransform),
		endpointTF(Eigen::Affine3f::Identity()),
		link(links),
		C(Eigen::MatrixXf::Zero(this->n, this->n)),
		D(Eigen::MatrixXf::Identity(this->n, this->n)),
		M(Eigen::MatrixXf::Identity(this->n, this->n)),
		g(Eigen::VectorXf::Zero(this->n))
		
{
	// Initialize all std::vector objects
	this->fkchain.push_back(Eigen::Affine3f::Identity());
	
	for(int i = 0; i < this->n; i++)
	{
		this->fkchain.push_back(Eigen::Affine3f::Identity());	// Link / joint transforms
		this->a.push_back(Eigen::Vector3f::Zero());			// Axis of actuation
		this->r.push_back(Eigen::Vector3f::Zero());			// Translation from joint to endpoint
		this->w.push_back(Eigen::Vector3f::Zero());			// Angular velocity
		this->com.push_back(Eigen::Vector3f::Zero());			// Center of mass
		this->I.push_back(Eigen::MatrixXf::Zero(3, 3));		// Inertia matrix
		this->Jc.push_back(Eigen::MatrixXf::Zero(6, i+1));		// Jacobian to c.o.m.
		this->Jcdot.push_back(Eigen::MatrixXf::Zero(6,i+1));		// Time-derivative
	}

	set_joint_state(this->q, this->qdot);					// Set the initial state
}

/******************** Update all the internal kinematic & dynamic properties ********************/
bool SerialLink::set_joint_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() == vel.size() && pos.size() == this->n)		// Dimension of all arguments are correct
	{
		this->q = pos;							// Assign new joint positions
		this->qdot = vel;						// Assign new joint velocities
		
		update_forward_kinematics();					// Compute new transform chain
		update_com();							// Transform the com for each link to the base frame
		update_com_jacobian();						// Compute new Jacobian to each c.o.m.
		update_gravity_torque();					// Compute new torques from gravitational acceleration
		update_link_inertia();						// Rotate the inertia for each link to the base frame
		update_inertia();						// Compute new joint space inertia matrix
		
		// Older functions which will be made obsolete:	
		update_axis_and_distance();					// NOTE: TO BE SCHEDULED FOR DELETION
		get_inertia();							// NOTE: Change this to update_inertia in the future!
		update_omega();	
		return 1;
	}
	else
	{
		std::cout << "ERROR: SerialLink::set_joint_state() : Length of input vectors are incorrect." << std::endl;
		std::cout << "This object has " << this->n << " joints and you input a " << pos.size() << "x1" << " vector and a "
			  << vel.size() << "x1" << " vector." << std::endl;
		return 0;
	}
}

/******************** Adds an additional transform to the endpoint of the forward kinematics ********************/
void SerialLink::set_endpoint_offset(const Eigen::Affine3f &transform)
{
	this->endpointTF = transform;
}

/******************** Set a new gravity vector ********************/
void SerialLink::set_gravity_vector(const Eigen::Vector3f &gravity)
{
	this->gravityVector = gravity;
}

/******************** Get the current joint position, velocites ********************/
void SerialLink::get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel)
{
	pos = this->q;
	vel = this->qdot;
}


/******************** Compute forward kinematics chain at current joint position ********************/
void SerialLink::update_forward_kinematics()
{	
	this->fkchain[0] = this->baseTF*this->link[0].get_pose(this->q[0]); 			// Pre-multiply by base transform
	this->a[0] = this->fkchain[0].rotation()*this->link[0].get_axis();			// Update axis for 1st joint	
	
	for(int i = 1; i < this->n; i++)
	{
		this->fkchain[i] = this->fkchain[i-1]*this->link[i].get_pose(this->q[i]);	// Compute chain of joint/link transforms
		this->a[i] = this->fkchain[i].rotation()*this->link[i].get_axis();		// Rotate axis from local frame to global frame
	}
	this->fkchain[this->n] = this->fkchain[this->n-1]*this->link[this->n].get_pose(0)*this->endpointTF; // Offset the endpoints
}


/******************** Compute kinematic properties of mechanism based on current joint position ********************/
// NOTE: SCHEDULED FOR DELETION
//		- std::vector<Eigen::Vector3f> r IS OBSOLETE.
//	 	- std::vector<Eigen::Vector3f> a IS UPDATED INSIDE update_forward_kinematics()
void SerialLink::update_axis_and_distance()
{
	for(int i = 0; i < this->n; i++)
	{
		this->a[i] = this->fkchain[i].rotation()*this->link[i].get_axis();			// Rotate axis from local frame to global frame
		this->r[i] = this->fkchain[this->n].translation() - this->fkchain[i].translation();	// Distance from joint to end-effector
	}		
}

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

/******************** Compute location for the C.O.M. for each link ********************/
void SerialLink::update_com()
{
	for(int i = 0; i < this->n; i++)
	{
		this->com[i] = this->fkchain[i] * this->link[i+1].get_com();
	}
}

/******************** Compute inertia matrices for each link ********************/
void SerialLink::update_link_inertia()
{
	Eigen::AngleAxisf R;
	R = this->baseTF.rotation() * this->link[0].get_pose(this->q(0)).rotation();
	R = this->fkchain[0].rotation();

	I[0] = R*this->link[1].get_inertia()*R.toRotationMatrix().transpose();

	for(int i = 0; i < this->n; ++i)
	{
		R = this->fkchain[i].rotation();
		I[i] = R*this->link[i+1].get_inertia()*R.toRotationMatrix().transpose();
	}
}

/******************** Returns the Jacobian matrix ********************

NOTE: TESTING NEW JACOBIAN METHOD. IF IT WORKS, THIS CODE BLOCK CAN BE DELETED IN FUTURE

Eigen::MatrixXf SerialLink::get_jacobian()
{
	// No input arguments, so get the Jacobian to the end-effector
	return get_jacobian(this->fkchain[this->n].translation(), this->n);

	Eigen::MatrixXf J;
	J.setZero(6,n);

	for(int i = 0; i < this->n; i++)
	{
		if(this->link[i].is_revolute())					// Revolute joint
		{
			J.block(0,i,3,1) = this->a[i].cross(this->r[i]);		// a_i x r_i
			J.block(3,i,3,1) = this->a[i];				// a_i
		}
		else									// Prismatic joint
		{
			J.block(0,i,3,1) = this->a[i];				// a_i
			// J.block(3,i,3,1) = zeros
		}
	}

	return J;
}
*/

/******************** Compute a Jacobian matrix to any given point ********************/
Eigen::MatrixXf SerialLink::get_jacobian(const Eigen::Vector3f &point, const int &numJoints)
{
	Eigen::MatrixXf J;								// Value to be returned
	J.setZero(6,numJoints);
	
	for(int i = 0; i < numJoints; i++)
	{
		if(this->link[i].is_revolute())					// Revolute joint
		{
			J.block(0,i,3,1) = this->a[i].cross(point - this->fkchain[i].translation()); // a_i x r_i
			J.block(3,i,3,1) = this->a[i];				// a_i
		}
		else									// Prismatic joint
		{
			J.block(0,i,3,1) = this->a[i];				// a_i
//			J.block(3,i,3,1) = zeros					// 0
		}
	}
	return J;
}

/******************** Returns the Jacobian matrix for the centre of mass for every link ********************/
void SerialLink::update_com_jacobian()
{
	for(int i = 0; i < this->n; i++)
	{
		this->Jc[i] = get_jacobian(this->com[i], i+1);			// Super easy now!
	}
}

/******************** Compute the effect of gravity on the joints ********************/
void SerialLink::update_gravity_torque()
{
	this->g.setZero(this->n);							// Set all the elements to zero
	
	
	for(int i = 0; i < this->n; i++)
	{
		// Since gravity points down, we need to negate its effects here
		this->g.head(i+1) -= this->link[i+1].get_mass()*this->Jc[i].block(0,0,3,i+1).transpose()*this->gravityVector;
	}
}
	

/******************** Compute inertia matrix in joint space ********************/
void SerialLink::update_inertia()
{
	this->M.setZero(this->n, this->n);						// Zero all the elements

	for(int i = 0; i < this->n; i++)
	{	
		M.block(0,0,i+1,i+1) += this->link[i].get_mass()*this->Jc[i].block(0,0,3,i+1).transpose()*this->Jc[i].block(0,0,3,i+1)
					+ this->Jc[i].block(3,0,3,i+1).transpose()*this->I[i]*this->Jc[i].block(3,0,3,i+1);
	}
}

/******************** Returns the Jacobian matrix for the centre of mass for every link ********************/
// NOTE: THIS FUNCTION IS SCHEDULED FOR DELETION!
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
			d = this->com[i] - this->fkchain[j].translation();

			if (link[i].is_revolute())
			{
				Jm[i].block(0,j,3,1) = this->a[j].cross(d);
				Jm[i].block(3,j,3,1) = this->a[j];
				Jm[i].col(j) << this->a[j].cross(d), this->a[j];
			}
			else
				Jm[i].block(0,j,3,1) = this->a[j];
		}
	}
	return Jm;
}

/******************** Returns the inertia matrix of the manipulator in the joint space ********************/
// NOTE: SCHEDULED FOR DELETION
Eigen::MatrixXf SerialLink::get_inertia()
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


/******************** Returns the gravity torque ********************/
Eigen::VectorXf SerialLink::get_gravity_torque()
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

#endif

/* OLD CODE:

class SerialLink
{
	public:
		// Basic constructor
		SerialLink(const std::vector<Link> &links,
					const Eigen::Affine3f &baseTransform,
					const Eigen::Affine3f &finalTransform);

		// Functions

		bool update_state(const Eigen::VectorXf &pos,				// Update all internal kinematics & dynamics
				// Basic kinematic properties
		int n;								// Number of joints
		Eigen::VectorXf q, qdot;					// Joint positions, velocities
		Eigen::VectorXf baseTwist;					// Base twist
		Eigen::Affine3f baseTF;					// Transform to first joint
		Eigen::Affine3f finalTF;					// Transform from final joint to endpoint
		Eigen::Affine3f endpointTF;					// New endpoint offset (default identity)
		std::vector<Link> link;					// Array of link objects
		void update_omega();
		void update_com();						// Compute location for the C.O.M. for each link
		void update_link_inertia();
		// Variable kinematic properties
		std::vector<Eigen::Affine3f> fkchain;				// Transforms for each link
		std::vector<Eigen::Vector3f> a;				// Axis of actuation for each joint, in base frame
		std::vector<Eigen::Vector3f> r;				// Translation from joint origin to endpoint
		std::vector<Eigen::Vector3f> w;				// Cumulative angular velocity up the kinematic chain

		// Dynamic properties
		Eigen::MatrixXf C;						// Coriolis matrix (nxn)
		Eigen::MatrixXf D;						// Damping matrix (nxn)
		Eigen::MatrixXf M;						// Inertia matrix (nxn)
		Eigen::VectorXf g;						// Gravitational torque (nx1)
		std::vector<Eigen::Vector3f> com;				// Center of mass for each link, in base frame
		std::vector<Eigen::MatrixX3f> I;				// Inertia matrices for each link in the origin frame
		std::vector<Eigen::MatrixXf> Jm;				// Jacobian to the center of mass
		std::vector<Eigen::MatrixXf> JmDot;				// Time derivative of Jacobian

		// Functions
		void update_forward_kinematics();				// Compute forward kinematic chain based on current state
		void update_axis_and_distance();				// Compute kinematic properties of mechanism for current state
		void update_omega();
		void update_com();						// Compute location for the C.O.M. for each link
		void update_link_inertia();
		Eigen::MatrixXf get_inverse_jacobian(const Eigen::MatrixXf &J); // Get the inverse, automatically applies DLS
		Eigen::MatrixXf get_joint_weighting_matrix();		const Eigen::VectorXf &vel);

		void get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel);	// Get the current internal joint state

		Eigen::Affine3f get_endpoint() const {return this->fkchain[this->n];}// Get the pose of the endpoint

		Eigen::Affine3f get_endpoint_error(const Eigen::Affine3f &desired);	// Get the error between endpoint pose and desired

		Eigen::MatrixXf get_jacobian();					// Get Jacobian at current state

		std::vector<Eigen::MatrixXf> get_mass_jacobian();			// Get the Jacobian to the c.o.m for every link

		// std::vector<Eigen::MatrixXf> get_mass_jacobian_dot();

		Eigen::MatrixXf get_inertia();					// Get the inertia matrix of the manipulator

		Eigen::VectorXf get_gravity_torque();

		void set_endpoint_offset(const Eigen::Affine3f &transform);		// Define a new endpoint from the original

		Eigen::VectorXf rmrc(const Eigen::VectorXf &xdot, const Eigen::VectorXf &qdot_d);

		int get_number_of_joints() const{return this->n;}			// Returns the number of joints

	private:
		// Basic kinematic properties
		int n;								// Number of joints
		Eigen::VectorXf q, qdot;					// Joint positions, velocities
		Eigen::VectorXf baseTwist;					// Base twist
		Eigen::Affine3f baseTF;					// Transform to first joint
		Eigen::Affine3f finalTF;					// Transform from final joint to endpoint
		Eigen::Affine3f endpointTF;					// New endpoint offset (default identity)
		std::vector<Link> link;					// Array of link objects
		void update_omega();
		void update_com();						// Compute location for the C.O.M. for each link
		void update_link_inertia();
		// Variable kinematic properties
		std::vector<Eigen::Affine3f> fkchain;				// Transforms for each link
		std::vector<Eigen::Vector3f> a;				// Axis of actuation for each joint, in base frame
		std::vector<Eigen::Vector3f> r;				// Translation from joint origin to endpoint
		std::vector<Eigen::Vector3f> w;				// Cumulative angular velocity up the kinematic chain

		// Dynamic properties
		Eigen::MatrixXf C;						// Coriolis matrix (nxn)
		Eigen::MatrixXf D;						// Damping matrix (nxn)
		Eigen::MatrixXf M;						// Inertia matrix (nxn)
		Eigen::VectorXf g;						// Gravitational torque (nx1)
		std::vector<Eigen::Vector3f> com;				// Center of mass for each link, in base frame
		std::vector<Eigen::MatrixX3f> I;				// Inertia matrices for each link in the origin frame
		std::vector<Eigen::MatrixXf> Jm;				// Jacobian to the center of mass
		std::vector<Eigen::MatrixXf> JmDot;				// Time derivative of Jacobian

		// Functions
		void update_forward_kinematics();				// Compute forward kinematic chain based on current state
		void update_axis_and_distance();				// Compute kinematic properties of mechanism for current state
		void update_omega();
		void update_com();						// Compute location for the C.O.M. for each link
		void update_link_inertia();
		Eigen::MatrixXf get_inverse_jacobian(const Eigen::MatrixXf &J); // Get the inverse, automatically applies DLS
		Eigen::MatrixXf get_joint_weighting_matrix();

};										// Semicolon needed after class declaration


/******************** Basic constructor ********************
SerialLink::SerialLink(const std::vector<Link> &links,
			const Eigen::Affine3f &baseTransform,
			const Eigen::Affine3f &finalTransform)
			:
			n(links.size()-1),					// Get the number of joints
			q(Eigen::VectorXf::Zero(this->n)),			// Assume zero position
			qdot(Eigen::VectorXf::Zero(this->n)),			// Zero starting velocity
			baseTwist(Eigen::VectorXf::Zero(6)),
			baseTF(baseTransform),										
			finalTF(finalTransform),
			endpointTF(Eigen::Affine3f::Identity()),
			link(links),
			C(Eigen::MatrixXf::Zero(this->n, this->n)),		
			D(Eigen::MatrixXf::Identity(this->n, this->n)),
			M(Eigen::MatrixXf::Zero(this->n, this->n)),
			g(Eigen::VectorXf::Zero(this->n))
{
	// Initialize all the std::vector objects
	this->fkchain.push_back(Eigen::Affine3f::Identity());
	for(int i = 0; i < this->n; i++)
	{
		this->fkchain.push_back(Eigen::Affine3f::Identity());
		this->a.push_back(Eigen::Vector3f::Zero());
		this->r.push_back(Eigen::Vector3f::Zero());
		this->w.push_back(Eigen::Vector3f::Zero());
		this->com.push_back(Eigen::Vector3f::Zero());
		this->I.push_back(Eigen::MatrixXf::Zero(3, 3));
		this->Jm.push_back(Eigen::MatrixXf::Zero(6, this->n));
		this->JmDot.push_back(Eigen::MatrixXf::Zero(6,this->n));
	}

	update_state(this->q, this->qdot);					// Set the initial state
}

/******************** Update all the internal kinematic & dynamic properties ********************
bool SerialLink::update_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() == vel.size() && pos.size() == this->n)		// Dimension of all arguments are correct
	{
		this->q = pos;
		this->qdot = vel;

		update_forward_kinematics();
		update_axis_and_distance();
		update_com();
		update_omega();
		update_link_inertia();
		get_inertia();

		return 1;
	}
	else
	{
		// SEND AN ERROR MESSAGE HERE? 
		return 0;
	}
}

/******************** Get the current joint position, velocites ********************
void SerialLink::get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel)
{
	pos = this->q;
	vel = this->qdot;
}

/******************** Compute forward kinematics chain at current joint position ********************
void SerialLink::update_forward_kinematics()
{	
	this->fkchain[0] = this->baseTF*this->link[0].get_pose(this->q[0]); 			// Pre-multiply by base transform

	for(int i = 1; i < this->n; i++)
	{
		this->fkchain[i] = this->fkchain[i-1]*this->link[i].get_pose(this->q[i]);	// Compute chain of joint/link transforms
	}
	this->fkchain[this->n] = this->fkchain[this->n-1] * this->link[this->n].get_pose(0) * this->endpointTF; // Offset the endpoints
}


/******************** Compute kinematic properties of mechanism based on current joint position ********************
void SerialLink::update_axis_and_distance()
{
	for(int i = 0; i < this->n; i++)
	{
		this->a[i] = this->fkchain[i].rotation()*this->link[i].get_axis();			// Rotate axis from local frame to global frame
		this->r[i] = this->fkchain[this->n].translation() - this->fkchain[i].translation();	// Distance from joint to end-effector
	}		
}

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

/******************** Compute location for the C.O.M. for each link ********************
void SerialLink::update_com()
{
	for(int i = 0; i < this->n; i++)
	{
		this->com[i] = this->fkchain[i] * this->link[i+1].get_com();
	}
}

/******************** Compute inertia matrices for each link ********************
void SerialLink::update_link_inertia()
{
	Eigen::AngleAxisf R;
	R = this->baseTF.rotation() * this->link[0].get_pose(this->q(0)).rotation();
	R = this->fkchain[0].rotation();

	I[0] = R*this->link[1].get_inertia()*R.toRotationMatrix().transpose();

	for(int i = 0; i < this->n; ++i)
	{
		R = this->fkchain[i].rotation();
		I[i] = R*this->link[i+1].get_inertia()*R.toRotationMatrix().transpose();
	}
}

/******************** Returns the Jacobian matrix ********************
Eigen::MatrixXf SerialLink::get_jacobian()
{
	Eigen::MatrixXf J;
	J.setZero(6,n);

	for(int i = 0; i < this->n; i++)
	{
		if(this->link[i].is_revolute())					// Revolute joint
		{
			J.block(0,i,3,1) = this->a[i].cross(this->r[i]);		// a_i x r_i
			J.block(3,i,3,1) = this->a[i];				// a_i
		}
		else									// Prismatic joint
		{
			J.block(0,i,3,1) = this->a[i];				// a_i
			// J.block(3,i,3,1) = zeros
		}
	}

	return J;
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
			d = this->com[i] - this->fkchain[j].translation();

			if (link[i].is_revolute())
			{
				Jm[i].block(0,j,3,1) = this->a[j].cross(d);
				Jm[i].block(3,j,3,1) = this->a[j];
				Jm[i].col(j) << this->a[j].cross(d), this->a[j];
			}
			else
				Jm[i].block(0,j,3,1) = this->a[j];
		}
	}
	return Jm;
}

/******************** Returns the time derivative of the  Jacobian matrix for the centre of mass for every link ********************
// std::vector<Eigen::MatrixXf> SerialLink::get_mass_jacobian_dot()
// {
// 	std::vector<Eigen::MatrixXf> Jmdot;


// 	return Jmdot;
// }

/******************** Returns the inertia matrix of the manipulator in the joint space ********************
Eigen::MatrixXf SerialLink::get_inertia()
{
	Eigen::MatrixXf M = Eigen::MatrixXf::Zero(this->n, this->n);
	std::vector<Eigen::MatrixXf> Jm = get_mass_jacobian();
	std::vector<Eigen::MatrixX3f> linkInertia = this->I;				// Inertia matrices for each link in the origin frame


	for(int i = 0; i < this->n; ++i)
	{
		M = M + this->link[i+1].get_mass()*Jm[i].block(0,0,3,this->n).transpose() * Jm[i].block(0,0,3,this->n)
				+ Jm[i].block(3,0,3,this->n).transpose() * linkInertia[i] * Jm[i].block(3,0,3,this->n);
	}

	return M;
}


/******************** Returns the gravity torque ********************
Eigen::VectorXf SerialLink::get_gravity_torque()
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

/******************** Adds an additional transform to the endpoint of the forward kinematics ********************
void SerialLink::set_endpoint_offset(const Eigen::Affine3f &transform)
{
	this->endpointTF = transform;
}

/******************** Get the error between the current endpoint and a desired value ********************
Eigen::Affine3f SerialLink::get_endpoint_error(const Eigen::Affine3f &desired)
{
		return desired*get_endpoint().inverse();
}

/******************** Get the joint velocities to move the end-effector at a given speed (with a redundant task) ********************
Eigen::VectorXf SerialLink::rmrc(const Eigen::VectorXf &xdot, const Eigen::VectorXf &qdot_d)
{
	// Forward kinematics:
	//	x = f(q)
	//  Velocity kinematics:
	//	xdot = J(q)qdot
	//  Resolved motion rate control (rmrc):
	//	qdot = inv(J)*xdot
	//

	Eigen::VectorXf qdot(this->n);
	
	if(xdot.size() != 6)
	{
		// ERROR
		qdot.setZero();
	}
	else
	{
		Eigen::MatrixXf    J = get_jacobian();
		Eigen::MatrixXf invJ = get_inverse_jacobian(J);
		
		if(this->n < 6)							// Underactuated
		{
			// ERROR: Functionality not currently supported.
			qdot.setZero();
		}
		else if(this->n == 6)							// Full actuated
		{
			qdot = invJ*xdot;
		}
		else									// Redundant
		{
			qdot = invJ*xdot + (Eigen::MatrixXf::Identity(this->n, this->n) - invJ*J)*qdot_d;
		}
	}
	
	return qdot;
}

/******************** Invert the Jacobian, add weighting, DLS as necessary ********************
Eigen::MatrixXf SerialLink::get_inverse_jacobian(const Eigen::MatrixXf &J)
{
	Eigen::MatrixXf invJ;								// To be returned

	// First check to see if the matrix is singular
	float manipulability = sqrt((J*J.transpose()).determinant());		// Proximity to a singularity
	float damping = 0.0;								// Value for damped least squares (DLS)
	float threshold = 0.001;							// Minimum value for activating DLS
	if(manipulability < 0.001)
	{
		damping = (1 - pow((manipulability/threshold),2))*0.01;		// Increase damping
	}
	
	// Check the number of joints
	if(this->n == 6)								// Not redundant
	{
		if(damping == 0)	invJ = J.inverse();				// Direct inverse of square matrix
		else			invJ = J.transpose()*(J*J.transpose() + damping*Eigen::MatrixXf::Identity(6,6)).inverse();	// DLS inverse
	}
	else										// Redundant case
	{
		// Fill this in later:
		// Eigen::MatrixXf W = this->M + get_joint_weighting_matrix();
		// invW = W.inverse();
		// invJ = invW*J.transpose()*(J*invW*J.transpose() + damping*Eigen::MatrixXf::Identity(6,6)).inverse()
		
		invJ = J.transpose()*(J*J.transpose() + damping*Eigen::MatrixXf::Identity(6,6)).inverse();
	}
	
	return invJ;
}

/******************** Weight the joint motion to avoid joint limits ********************
Eigen::MatrixXf SerialLink::get_joint_weighting_matrix()
{
	Eigen::MatrixXf W;
	W.setIdentity(this->n, this->n);
	
	// Compute velocity for weighting function wdot;
	// If wdot > 0
	//	Add weighting
	// else
	//	Leave as 1
	// end
	//
	return W;
}
*/
