/*
*	Minimum jerk trajectory between two points.
*/

#ifndef QUINTIC_H_
#define QUINTIC_H_

#include <Eigen/Geometry>

class Quintic
{
	public:
		Quintic() {}						// Empty constructor
	
		Quintic(const Eigen::VectorXf &startPoint,		// Constructor for trajectory across real numbers
			const Eigen::VectorXf &endPoint,
			const float &startTime,
			const float &endTime);
			
		Quintic(const Eigen::Quaternionf &startPoint,		// Constructor for orientation trajectory
			const Eigen::Quaternionf &endPoint,
			const float &startTime,
			const float &endTime);
			
		void get_state(Eigen::VectorXf &pos,			// Get the desired position for the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
		
		void get_state(Eigen::Quaternionf &quat,		// Get the desired rotation for a given time
				Eigen::Vector3f &vel,
				Eigen::Vector3f &acc,
				const float &time);
	
	private:
		bool isQuaternion = false;				// Self explanatory
		
		float a, b, c;						// Polynomial coefficients
		float s, sd, sdd;					// Interpolation scalars
		float t1, t2;						// Start time and end time
		
		Eigen::VectorXf p1, p2;				// Start point and end point for real numbers
		
		Eigen::Quaternionf q1, q2;				// Used for rotation trajectories
		float angle; Eigen::Vector3f axis;
		
		void compute_coefficients();		
		void compute_scalars(const float &time);		// Compute the interpolation scalars for the given time	

};									// Semicolon needed after class declaration

/******************** Create a trajectory over real numbers ********************/
Quintic::Quintic(const Eigen::VectorXf &startPoint,			// Constructor for trajectory across real numbers
		const Eigen::VectorXf &endPoint,
		const float &startTime,
		const float &endTime)
		:
		p1(startPoint),
		p2(startPoint),
		t1(startTime),
		t2(endTime),
		isQuaternion(false)
{
	if(startPoint.size() != endPoint.size())
	{
		std::cout << "ERROR: Quintic::Quintic() : Vectors are not of equal length! The first vector has "
			<< startPoint.size() << " elements and the second vector has " << endPoint.size() << " elements." << std::endl;
	}
	
	compute_coefficients();					// Compute polynomial coefficients
}

/******************** Get the desired state for real numbers ********************/
void Quintic::get_state(Eigen::VectorXf &pos,				// Get the desired position for the given time
			Eigen::VectorXf &vel,
			Eigen::VectorXf &acc,
			const float &time)
{
	// Check inputs are sound
	if(this->isQuaternion)
	{
		std::cout << "ERROR: Quintic::get_state() : This is an orientation trajectory, but you called a function for position!" << std::endl;
	}
	
	if(pos.size() != vel.size () || vel.size() != acc.size())
	{
		std::cout << "ERROR: Quintic::get_state() : Input vectors are not of equal length! The pos vector has " 
			<< pos.size() << " elements, the vel vector has " << vel.size()
			<< " elements and the acc vector has " << acc.size() << " elements." << std::endl;
	}
	
	compute_scalars(time);						// Get the scalars to interpolate along the trajectory
	
	pos = (1 - this->s)*this->p1 + this->s*this->p2;
	vel = this->sd*(this->p2 - this->p1);
	acc = this->sdd*(this->p2 - this->p1);
}

/******************** Create a trajectory over a rotation  ********************/
Quintic::Quintic(const Eigen::Quaternionf &startPoint,
		const Eigen::Quaternionf &endPoint,
		const float &startTime,
		const float &endTime)
		:
		q1(startPoint),
		q2(endPoint),
		t1(startTime),
		t2(startTime),
		isQuaternion(true)
{
	compute_coefficients();					// Compute polynomial coefficients
	
	// We want to interpolate over the difference in orientations
	// such that: q1*dq = q2 ---> dq = conj(q1)*q2
	Eigen::AngleAxisf angleAxis(startPoint.conjugate()*endPoint); // Do conj(q1)*q2 then convert to angle-axis
	this->angle = angleAxis.angle();				// Get the angle encoded in the rotation
	this->axis = angleAxis.axis();				// Get the axis of rotation
	
	if(this->angle > M_PI) this->angle = 2*M_PI - this->angle;	// If rotation is > 180 degrees, take the shorter path
}



/******************** Get the desired state for rotation ********************/
void Quintic::get_state(Eigen::Quaternionf &quat,			// Get the desired rotation for a given time
			Eigen::Vector3f &vel,
			Eigen::Vector3f &acc,
			const float &time)
{
	if(!this->isQuaternion)
	{
		std::cout << "ERROR: Quintic::get_state() : This is a position trajectory,"
			<< " but you called the function for an orientation trajectory!" << std::endl;
	}
	
	compute_scalars(time);							// Get the scalar to interpolate the trajectory

	float temp = this->s*this->angle;					// Compute the interpolated angle
	Eigen::Quaternionf dq(Eigen::AngleAxisf(temp, this->axis));		// Convert to quaternion
	
	quat = this->q1*dq;							// q(t) = q1*dq(t)
	vel = this->sd*this->angle*this->axis;				// Angular velocity
	acc = this->sdd*this->angle*this->axis;				// Angular acceleration
}


/******************** Compute the quintic polynomial coefficients ********************/
void Quintic::compute_coefficients()
{
	// NOTE: I put the error check on the time inputs here because it's common
	// to all constructors.
	if(this->t1 > this->t2)
	{
		std::cout << "ERROR: Quintic::Quintic() : Start time of " << this->t1
			<< " is greater than end time of " << this->t2 << "! Swapping the times..." << std::endl;
		float temp = this->t2;
		this->t2 = this->t1;
		this->t1 = temp;
	}
	
	float dt = this->t1 - this->t2;						// Time difference
	
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}

/******************** Compute the scalars for the time interpolation ********************/
void Quintic::compute_scalars(const float &time)
{
	float dt = time - this->t1;							// Get elapsed time since the beginning
	
	this->s =      this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
	this->sd =   5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
	this->sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c;	
}

#endif

