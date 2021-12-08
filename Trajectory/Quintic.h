/*
*	Minimum jerk trajectory between two points.
*/

#ifndef QUINTIC_H_
#define QUINTIC_H_

#include <Eigen/Geometry>

class Quintic
{
	public:
		Quintic();						// Empty constructor
	
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
		Eigen::Vector3f angleAxis;				// Interpolate over angle-axis for rotations			
		
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
		// ERROR: Inputs are not the same length!
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
		// ERROR: Did you mean to call the one for rotations?
	}
	
	if(pos.size() != vel.size () || vel.size() != acc.size())
	{
		// ERROR: Input vectors are not of the same length!
	}
		
	// Figure out where we are on the trajectory
	if(time < this->t1)						// Not yet started
	{
		pos = this->p1;					// Remain at the start
		vel.setZero();
		acc.setZero();
	}
	else if(time < this->t2)					// Somewhere in the middle
	{
		compute_scalars(time);					// Get the scalars for the current time
	
		pos = (1.0 - this->s)*this->p1 + this->s*this->p2;	// Position interpolation
		vel = this->sd*(this->p2 - this->p1);			// Velocity
		acc = this->sdd*(this->p2 - this->p1);		// Acceleration
	}
	else								// Finished
	{
		pos = this->p2;					// Remain at the end		
		vel.setZero();
		acc.setZero();
	}
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
	
	Eigen::Quaternionf dq = startPoint.inverse()*endPoint;	// We want to interpolate over the *difference* in rotation
	
	float angle = 2*acos(dq.w());					// Angle encoded in the difference

	this->angleAxis = angle*dq.vec().normalized();		// This is used later for interpolation purposes
	
	if(angle > M_PI)						// If the angle is greater than 180 degrees...
	{
		angle = 2*M_PI - angle;				// ... Take the shorter path...
		this->angleAxis *= -1;					// ... and flip the direction to match
	}
}



/******************** Get the desired state for rotation ********************/
void Quintic::get_state(Eigen::Quaternionf &quat,			// Get the desired rotation for a given time
			Eigen::Vector3f &vel,
			Eigen::Vector3f &acc,
			const float &time)
{
	if(!this->isQuaternion)
	{
		// ERROR: Did you call the right function?
	}
	
	// Figure out where we are on the trajectory
	if(time < this->t1)						// Not yet started
	{
		quat = this->q1;					// Remain at the start
		vel.setZero();
		acc.setZero();
	}
	else if(time < this->t2)
	{
		compute_scalars(time);					// Get the scalars for the current time
		
		Eigen::Vector3f temp = this->s*this->angleAxis;	// Assumes zero starting rotation
		Eigen::Quaternionf dq(Eigen::AngleAxisf(temp.norm(), temp.normalized())); // Convert to angle-axis then to quaternion
		
		dq = this->q1*dq;				
		vel = this->sd*this->angleAxis;			// I hope this correct...
		acc = this->sdd*this->angleAxis;
	}
	else								// Remain at the end
	{
		quat = this->q2;
		vel.setZero();
		acc.setZero();
	}

}


/******************** Compute the quintic polynomial coefficients ********************/
void Quintic::compute_coefficients()
{
	if(this->t1 > this->t2)
	{
		// ERROR: Trajectory ends before it begins!
		// Swapping times...
		float temp = this->t1;
		this->t1 = this->t2;
		this->t2 = temp;
	}
	
	float dt = this->t1 - this->t2;				// Time difference
	
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}

/******************** Compute the scalars for the time interpolation ********************/
void Quintic::compute_scalars(const float &time)
{
	float dt = time - this->t1;					// Get elapsed time since the beginning
	
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

