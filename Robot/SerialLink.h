/*
	TO DO:
		- Test the forward kinematics calculation is correct
		- Test that the Jacobian calculation is correct
		- Add some sort of error message (Line 124)
		
*/

#ifndef SERIAL_LINK_H_								// If not yet defined...
#define SERIAL_LINK_H_								// ... include this header file, otherwise ignore

#include <Eigen/Geometry>							// Transforms
#include <Link.h>								// Link class
#include <vector>								// std::vector

class SerialLink
{
	public:
		// Basic constructor
		SerialLink(const std::vector<Link> &links,
					const Eigen::Affine3f &baseTransform,
					const Eigen::Affine3f &finalTransform);

		// Functions

		bool update_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel);	// Update all the internal dynamic & kinematic properties

		void get_joint_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel);

		Eigen::Affine3f get_endpoint() const {return this->fkchain[this->n];}	// Get the pose of the endpoint

		Eigen::Affine3f get_endpoint_error(const Eigen::Affine3f &desired);		// Get the error between endpoint pose and desired

		Eigen::MatrixXf get_jacobian();						// Get Jacobian at current state

		std::vector<Eigen::MatrixXf> get_mass_jacobian();						// Get the Jacobian matrix for the centre of mass for every link

		Eigen::VectorXf get_gravity_torque();

		void set_endpoint_offset(const Eigen::Affine3f &transform);			// Define a new endpoint from the original

		Eigen::VectorXf rmrc(const Eigen::VectorXf &xdot, const Eigen::VectorXf &qdot_d);

		int get_number_of_joints() const{return this->n;}				// Returns the number of joints

	private:
		// Basic kinematic properties
		int n;									// Number of joints
		Eigen::VectorXf q, qdot;				// Joint positions, velocities
		Eigen::Affine3f baseTF;					// Transform to first joint
		Eigen::Affine3f finalTF;				// Transform from final joint to endpoint
		Eigen::Affine3f endpointTF;				// New endpoint offset (default identity)
		std::vector<Link> link;					// Array of link objects

		// Variable kinematic properties
		std::vector<Eigen::Affine3f> fkchain;		// Transforms for each link
		std::vector<Eigen::Vector3f> a;				// Axis of actuation for each joint, in base frame
		std::vector<Eigen::Vector3f> r;				// Translation from joint origin to endpoint

		// Dynamic properties
		Eigen::MatrixXf C;						// Coriolis matrix (nxn)
		Eigen::MatrixXf D;						// Damping matrix (nxn)
		Eigen::MatrixXf M;						// Inertia matrix (nxn)
		Eigen::VectorXf g;						// Gravitational torque (nx1) USUALLY AUTOMATICALL COMPENSATED ON A REAL ROBOT
		std::vector<Eigen::Vector3f> com;				// Center of mass for each link, in base frame
		std::vector<Eigen::MatrixXf> Jm;				// Jacobian to the center of mass
		std::vector<Eigen::MatrixXf> Jmdot;				// Time derivative of Jacobian

		// Functions
		void update_forward_kinematics();				// Compute forward kinematic chain based on current state
		void update_axis_and_distance();				// Compute kinematic properties of mechanism for current state
		void update_com();								// Compute location for the C.O.M. for each link
		Eigen::MatrixXf get_inverse_jacobian(const Eigen::MatrixXf &J); // Get the inverse, automatically applies DLS
		Eigen::MatrixXf get_joint_weighting_matrix();

};										// Semicolon needed after class declaration


/******************** Basic constructor ********************/
SerialLink::SerialLink(const std::vector<Link> &links,
			const Eigen::Affine3f &baseTransform,
			const Eigen::Affine3f &finalTransform)
			:
			n(links.size()-1),					// Get the number of joints
			q(Eigen::VectorXf::Zero(this->n)),			// Assume zero position
			qdot(Eigen::VectorXf::Zero(this->n)),			// Zero starting velocity
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
		this->com.push_back(Eigen::Vector3f::Zero());
		this->Jm.push_back(Eigen::MatrixXf::Zero(6, this->n));
		this->Jmdot.push_back(Eigen::MatrixXf::Zero(6,this->n));
	}

	update_state(this->q, this->qdot);					// Set the initial state
}

/******************** Update all the internal kinematic & dynamic properties ********************/
bool SerialLink::update_state(const Eigen::VectorXf &pos, const Eigen::VectorXf &vel)
{
	if(pos.size() == vel.size() && pos.size() == this->n)		// Dimension of all arguments are correct
	{
		this->q = pos;
		this->qdot = vel;

		update_forward_kinematics();
		update_axis_and_distance();
		update_com();
		return 1;
	}
	else
	{
		// SEND AN ERROR MESSAGE HERE? 
		return 0;
	}
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
	this->fkchain[0] = this->baseTF*this->link[0].get_pose(this->q[0]);						// Pre-multiply first link by base transform

	for(int i = 1; i < this->n; i++)
	{
		this->fkchain[i] = this->fkchain[i-1]*this->link[i].get_pose(this->q[i]);			// Compute chain of joint/link transforms
	}
	this->fkchain[this->n] = this->fkchain[this->n-1] * this->link[this->n].get_pose(0) * this->endpointTF;	// Post-multiply the last one by any offset
}


/******************** Compute kinematic properties of mechanism based on current joint position ********************/
void SerialLink::update_axis_and_distance()
{
	for(int i = 0; i < this->n; i++)
	{
		this->a[i] = this->fkchain[i].rotation()*this->link[i].get_axis();			// Rotate axis from local frame to global frame
		this->r[i] = this->fkchain[this->n].translation() - this->fkchain[i].translation();	// Distance from joint to end-effector
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

/******************** Returns the Jacobian matrix ********************/
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

/******************** Returns the Jacobian matrix for the centre of mass for every link ********************/
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

/******************** Adds an additional transform to the endpoint of the forward kinematics ********************/
void SerialLink::set_endpoint_offset(const Eigen::Affine3f &transform)
{
	this->endpointTF = transform;
}

/******************** Get the error between the current endpoint and a desired value ********************/
Eigen::Affine3f SerialLink::get_endpoint_error(const Eigen::Affine3f &desired)
{
		return desired*get_endpoint().inverse();
}

/******************** Get the joint velocities to move the end-effector at a given speed (with a redundant task) ********************/
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

/******************** Invert the Jacobian, add weighting, DLS as necessary ********************/
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

/******************** Weight the joint motion to avoid joint limits ********************/
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


#endif