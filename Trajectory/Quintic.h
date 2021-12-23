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
Quintic::Quintic(const Eigen::Quaternionf &startPoint,		// Constructor for orientation trajectory
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

/*

#include <Eigen/Core>

class Quintic
{
	public:
		Quintic();								// Empty constructor
		
		Quintic(const Eigen::VectorXf &start_point,				// Full constructor
			const Eigen::VectorXf &end_point,
			const float &start_time,
			const float &end_time,
			const bool &is_quaternion);

		void get_state(Eigen::VectorXf &pos,					// Get the desired position, velocity, acceleration at the given time
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);					
	
	private:
	
	bool isQuaternion;								// If true, use SLERP instead of LERP
	
	Eigen::VectorXf p1, p2;							// Start point and end point

	float a, b, c;									// Coefficients for the polynomial
	
	float angle;									// Angle between 2 quaternions
		
	float s, sd, sdd;								// Coefficients for interpolation
	
	float t1, t2;									// Start time and end time
	
	int m;										// Number of dimensions
	
	void compute_coefficients();							// Compute the polynomial coefficients
	
	
};											// Needed after class declaration

/******************** Empty Constructor ********************
Quintic::Quintic() : 	isQuaternion(false),						// Default values
			p1(Eigen::VectorXf::Zero(3)),
			p2(Eigen::VectorXf::Ones(3)),
			angle(0.0),							// Not used with default
			t1(0.0),
			t2(5.0),
			m(3)
{
	compute_coefficients();							// Compute the polynomial coefficients
}

/******************** Full Constructor ********************
Quintic::Quintic(const Eigen::VectorXf &start_point,					// Full constructor
		const Eigen::VectorXf &end_point,
		const float &start_time,
		const float &end_time,
		const bool &is_quaternion)
		:	
		isQuaternion(is_quaternion),
		p1(start_point),
		p2(end_point),
		t1(start_time),
		t2(end_time),
		m(start_point.size())
{
	if(start_point.size() != end_point.size())
	{
		// ERROR: Vectors are not of equal size.
	}

	if(start_time >= end_time)
	{
		// ERROR: Trajectory finishes before it starts!
	}
	
	compute_coefficients();							// Compute the polynomial coefficients
	
	// Ensure that quaternion interpolation takes shortest path
	if(this->isQuaternion && m == 4)
	{
		this->p1.normalize();							// First ensure unit norm...
		this->p2.normalize();
		
		this->angle = 2*acos(this->p1.dot(this->p2));				// ... Then compute the angle between orientations
		
		if(this->angle > M_PI)							// If the angle is greater than 180 degrees...
		{
			this->angle = 2*M_PI - this->angle;				// ... Take the shorter path ...
			this->p2 *= -1;						// ... and flip the quaternion to match
		}
	}
	else
	{
		// ERROR: Quaternion must have a length of 4.
	}
}

/******************** Set the polynomial coefficients ********************
void Quintic::compute_coefficients()
{
	float dt = this->t2 - this->t1;
	
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}

/******************** Get the desired state for a given time ********************
void Quintic::get_state(Eigen::VectorXf &pos,						// Get the desired position, velocity, acceleration at the given time
			Eigen::VectorXf &vel,
			Eigen::VectorXf &acc,
			const float &time)
{
	// Check that input dimensions are sound
	if(pos.size() != this->m && pos.size() != vel.size() && vel.size() != acc.size())
	{
		// ERROR: Vectors are not of equal length.
	}


	// Figure at when we are on the trajectory
	if(time < this->t1)								// Not yet started - remain at start point
	{
		this->s   = 0;								
		this->sd  = 0;
		this->sdd = 0;
	}
	else if(time <= this->t2)							// Somewhere inbetween
	{
		float dt = time - this->t1;						// Get elapsed time since the beginning
		
		this->s =      this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
		this->sd =   5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
		this->sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c;
	}
	else										// Finished - remain at end point
	{
		this->s   = 1;
		this->sd  = 0;
		this->sdd = 0;
	}
	
	// Use linear interpolation for R^n, and spherical linear for H
	if(!this->isQuaternion)							// Linear Interpolation (LERP)
	{
		for(int i = 0; i < this->m; i++)
		{
			pos(i) = (1.0 - this->s)*this->p1(i) + this->s*p2(i);
			vel(i) = this->sd*(this->p2(i) - this->p1(i));
			acc(i) = this->sdd*(this->p2(i) - this->p1(i));
		}
	}
	else										// Spherical Linear Interpolation (SLERP)
	{
		if(this->angle < 0.005)						// Very small difference, so don't bother moving
		{									// (avoids singularity)
			pos = this->p2;
			
			for(int i = 0; i < 3; i++)
			{
				vel(i) = 0.0;
				acc(i) = 0.0;
			}
		}
		else
		{
			float x = (1-this->s)*this->angle;				// This makes calcs a little easier
			float y = this->s*this->angle;
			float denominator = sin(this->angle);				
			
			pos = (sin(x)*this->p1 + sin(y)*this->p2)/denominator;	// Actual SLERP interpolation
			pos.normalize();						// Ensure unit norm
			
			// Compute velocity and acceleration in quaternion space
			Eigen::Vector4f dQ = 	(this->angle/denominator)*
						(cos(y)*this->p2 - cos(x)*this->p1);	// Partial derivative w.r.t. s
			
			Eigen::Vector4f Qdot  = this->sd*dQ;				// dQ/dt = dQ/ds*ds/dt
			Eigen::Vector4f Qddot = this->sdd*dQ - pow(this->angle*this->sd,2)*pos; // This is more complicated...
			
			// Convert from quaternion space to angular velocity, acceleration
			Eigen::MatrixXf M(3,4);
			M << -pos(1),  pos(0), -pos(3),  pos(2),
			     -pos(2),  pos(3),  pos(0), -pos(1),
			     -pos(3), -pos(2),  pos(1),  pos(0);
			M *= 2;
			
			for(int i = 0; i < 3; i++)
			{
				for(int j = 0; j < 4; j++)
				{
					vel(i) = M(i,j)*Qdot(j);
					acc(i) = M(i,j)*Qddot(j);
				}
			}
		}
	}
}
*/
#endif

