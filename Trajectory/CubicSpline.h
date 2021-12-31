/*
*	A minimum acceleration trajectory across multiple points.
*
*	Can interpolate quaternions.
*/

#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <Eigen/Geometry>							// Eigen::Quaternionf and the like
#include <iostream>								// std::cout << lol << std::endl
#include <vector>								// std::vector

class CubicSpline
{
	public:
		// Constructors
		CubicSpline() {}						// Empty constructor
		
		CubicSpline(const std::vector<Eigen::VectorXf> &points,	// Constructor for position
				const std::vector<float> &times);
			
		CubicSpline(const std::vector<Eigen::Quaternionf> &points,	// Constructor for orientation
				const std::vector<float> &times);
			
		// Functions
		void get_state(Eigen::VectorXf &pos,				// Get the desired state for translation
				Eigen::VectorXf &vel,
				Eigen::VectorXf &acc,
				const float &time);
			
		void get_state(Eigen::Quaternionf &quat,			// Get desired state for orientation
				Eigen::Vector3f &vel,
				Eigen::Vector3f &acc,
				const float &time);
	
	private:
		// Variables
		bool isQuaternion;						// Self explanatory
		int m, n;							// m dimensions with n waypoints
		std::vector<Eigen::VectorXf> a, b, c, d;			// Spline coefficients
		std::vector<Eigen::VectorXf> p;				// Position waypoints
		std::vector<Eigen::Quaternionf> q;				// Quaternion waypoints
		std::vector<float> t;						// Time to reach each waypoint
		
		// Functions
		void resize_vectors();						// Resize vectors a, b, c, d
		void compute_coefficients();					// Compute values for a, b, c, d
		int get_spline_number(const float &time);
		
};										// Semicolon needed after class declaration

/******************** Constructor for real numbers ********************/
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &points,
			const std::vector<float> &times)
			:
			isQuaternion(false),
			m(points[0].size()),					// Number of dimensions
			n(points.size()),					// Number of waypoints
			p(points),
			t(times)	
{
	// Check the inputs are sound
	if(points.size() != times.size())
	{
		std::cout << "ERROR CubicSpline::CubicSpline() : Input vectors must have the same length!" << std::endl;
	}
	else if(points.size() < 3)
	{
		std::cout << "ERROR CubicSpline::CubicSpline() : A minimum of 3 points is needed for CubicSpline!" << std::endl;
	}
	else
	{
		for(int i = 0; i < this->n-1; i++)
		{
			if(times[i+1] < times[i])
			{
				std::cout << "ERROR CubicSpline::CubicSline() : Times are not in ascending order!" << std::endl;
			}
		}
	}
	
	resize_vectors();							// Resize vectors a, b, c, d
	compute_coefficients();						// Compute the values for a, b, c, d
}

/******************** Constructor for orientation ********************/
CubicSpline::CubicSpline(const std::vector<Eigen::Quaternionf> &points,
			const std::vector<float> &times)
			:
			isQuaternion(true),					// A safety check
			m(3),							// Interpolates across angle*axis, which as 3 elements
			n(points.size()),					// n points, n-1 splines
			q(points),						// Waypoints
			t(times)
{
	// Check the inputs are sound
	if(points.size() != times.size())
	{
		std::cout << "ERROR: CubicSpline::CubicSpline() : Input vectors must have the same length!"
			<< "You input " << points.size() << " points and " << times.size() << " times." << std::endl;
	}
	else if(points.size() < 3)
	{
		std::cout << "ERROR CubicSpline::CubicSpline() : A minimum of 3 points is needed for CubicSpline!" << std::endl;
	}
	else
	{
		for(int i = 0; i < this->n-1; i++)
		{
			if(times[i+1] < times[i])
			{
				std::cout << "ERROR: CubicSpline::CubicSpline() : Times are not in ascending order!" << std::endl;
			}
		}
	}
	
	resize_vectors();							// Resize vectors a, b, c, and d
	
	// We need to interpolate over the DIFFERENCE in orientation
	// Note to self: p has dimensions [n][m] which is confusing,
	// but that's the way it is stored with std::vector
	this->p.resize(this->n);						// We will store the angle*axis in here
	Eigen::AngleAxisf angleAxis;						// Angle-axis encoded in the rotation
	float angle; Eigen::Vector3f axis;					// To separate out angle and axis
	
	for(int j = 0; j < this->n-1; j++)
	{
		// Note: q(t0)*dq(tf) = q(tf) ---> dq(tf) = conj(q(t0))*q(tf),
		// and we interpolate across dq(tf).
		// Hence later we will compute dq(t) ---> q(t) = q(t0)*dq(t)
		angleAxis = Eigen::AngleAxisf(this->q[j].conjugate()*this->q[j+1]); // Get the difference in orientation then convert
		
		angle = angleAxis.angle();
		if(angle > M_PI) angle = 2*M_PI - angle;			// If rotation > 180 degrees, take the shorter path
		
		axis = angleAxis.axis().normalized();				// Get the axis of rotation, ensure unit norm
		
		this->p[j] = Eigen::VectorXf(3);				// Set the jth point as a 3x1 vector
		for(int i = 0; i < 3; i++) p[j][i] = angle*axis[i];		// Interpolate across angle*axis
	}
	
	compute_coefficients();						// Compute the coefficients a, b, c, and d
}

/******************** Get the state for the given time ********************/
void CubicSpline::get_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	// Check inputs are sound
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		std::cout << "ERROR: CubicSpline::get_state() : Vectors are not of equal size!" << std::endl;
	}
	
	int j = get_spline_number(time);			// Figure out where we are
	
	if(j == -1)						// Not yet started
	{
		pos = this->p[0];				// Remain at the start
		vel.setZero();					// Don't move
		acc.setZero();
	}
	else if(j == this->n)					// At the end
	{
		pos = this->p[this->n-1];			// Remain at the end
		vel.setZero();					// Don't move
		acc.setZero();
	}
	else
	{
		float dt = time - this->t[j];
		
		for(int i = 0; i < this->m; i++)
		{
			pos[i] =   this->a[i][j]*pow(dt,3) +   this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
			vel[i] = 3*this->a[i][j]*pow(dt,2) + 2*this->b[i][j]*dt        + this->c[i][j];
			acc[i] = 6*this->a[i][j]*dt        + 2*this->b[i][j];
		}
	}
}

void CubicSpline::get_state(Eigen::Quaternionf &quat, Eigen::Vector3f &vel, Eigen::Vector3f &acc, const float &time)
{
	int j = get_spline_number(time);			// Figure out where we are on the trajectory
	
	if(j == -1)
	{
		quat = this->q[0];				// Remain at the start
		vel.setZero();					// Don't move
		acc.setZero();
	}
	else if(j == this->n)
	{
		quat = this->q[this->n-1];			// Remain at the end
		vel.setZero();					// Don't move
		acc.setZero();
	}
	else							// Somewhere inbetween
	{
		float dt = time - this->t[j];			// Elapsed time since start of spline j
		
		Eigen::Vector3f temp;				// This will be the DIFFERENCE in orientation, dq(t)
		
		for(int i = 0; i < 3; i++)
		{
			// temp is actually the angle*axis, so we can pull out
			// the angular velocity and acceleration directly.
			
			temp[i] =   this->a[i][j]*pow(dt,3) +   this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
			vel[i]  = 3*this->a[i][j]*pow(dt,2) + 2*this->b[i][j]*dt        + this->c[i][j];
			acc[i]  = 6*this->a[i][j]*dt        + 2*this->b[i][j];
		}
		
		// q(t) = q(t0)*dq(t)
		quat = this->q[j]*Eigen::Quaternionf(Eigen::AngleAxisf(temp.norm(), temp.normalized()));
	}
}

/********************* Initialize the sixe of std::vector objects ********************/
void CubicSpline::resize_vectors()
{
	// Note to self:
	// p has dimensions [n][m] because we are storing n waypoints of m dimensions in a std::vector
	// a, b, c, d have dimensions [m][n] so that the m dimensions are punctuated across n waypoints
	for(int i = 0; i < this->m; i++)
	{
		this->a.push_back(Eigen::VectorXf::Zero(this->n-1));		// NOTE: There are n-1 splines!
		this->b.push_back(Eigen::VectorXf::Zero(this->n-1));
		this->c.push_back(Eigen::VectorXf::Zero(this->n-1));
		this->d.push_back(Eigen::VectorXf::Zero(this->n-1));
	}
}

/********************* Compute polynomial coefficients for each spline ********************/
void CubicSpline::compute_coefficients()
{
	// Relationship between acceleration & position: A*sddot = B*s
	Eigen::MatrixXf A(this->n, this->n); A.setZero();
	Eigen::MatrixXf B(this->n, this->n); B.setZero();
	
	// Set constraints for the start and end of trajectory
	if(!this->isQuaternion)
	{
		A(0,0) = (this->t[1] - this->t[0])/2;
		A(0,1) = (this->t[1] - this->t[0])/2 + (this->t[2] - this->t[1])/3;
		A(0,2) = (this->t[2] - this->t[1])/6;
		
		B(0,1) = -1/(this->t[2] - this->t[1]);
		B(0,2) =  1/(this->t[2] - this->t[1]);
		B(this->n-1, this->n-2) = 1/(this->t[this->n-1] - this->t[this->n-2]);
		B(this->n-1, this->n-1) = -1/(this->t[this->n-1] - this->t[this->n-2]);
	}
	else
	{
		A(0,0) = (this->t[1] - this->t[0])/3;
		A(0,1) = (this->t[1] - this->t[0])/6;
		
		B(0,0) = 1/(this->t[1] - this->t[0]);
		B(this->n-1, this->n-2) = -1/(this->t[this->n-1] - this->t[this->n-2]);
	}
	
	A(this->n-1, this->n-2) = (this->t[this->n-1] - this->t[n-2])/6;
	A(this->n-1, this->n-1) = (this->t[this->n-1] - this->t[n-2])/3;
	
	// Set the constraints for each waypoint inbetween
	float dt1, dt2;
	for(int i = 1; i < this->n-2; i++)
	{
		dt1 = this->t[i] - this->t[i-1];
		dt2 = this->t[i+1] - this->t[i];
		
		A(i,i-1)	= dt1/6;
		A(i,i)		= (dt1 + dt2)/3;
		A(i, i+1)	= dt2/6;
		
		if(!this->isQuaternion)
		{
			B(i,i-1) = 1/dt1;
			B(i,i)   = -1/dt1 - 1/dt2;
			B(i,i+1) = 1/dt2;
		}
		else
		{
			B(i,i-1) = -1/dt1;
			B(i,i)   = 1/dt2;
		}
	}
	
	Eigen::MatrixXf C = A.inverse()*B; 						// A*sddot = B*s ---> sddot = A^-1*B*s
	
	// Solve coefficients for each spline across each dimension
	Eigen::VectorXf s(this->n), sdd(this->n);					// Pos, acc for a single dimension
	float dt;
	for(int i = 0; i < this->m; i++)
	{		
		for(int j = 0; j < this->n; j++)					// There are only n-1 splines
		{
			// Note to self:
			// p has dimensions [n][m] because we are storing n vectors
			// which each of m dimensions. So what we're asking for here
			// is the jth waypoint of the ith dimension.
			s[j] = this->p[j][i];						// Fill in the position for each waypoint
		}
		sdd = C*s;								// Solve the acceleration at each waypoint
		
		// Compute the coefficient for the n-1 splines
		for(int j = 0; j < this->n-1; j++)
		{
			dt = this->t[j+1] - this->t[j];				// Difference in time
			
			// Note to self: coefficients a, b, c, d are stored as
			// m dimensions across n waypoints (opposite to vector p)
			this->a[i][j] = (sdd[j+1] - sdd[j])/(6*dt);
			this->b[i][j] = sdd[j]/2;

			if(!this->isQuaternion) this->d[i][j] = s[j];			// Position at the start of each spline
			
			if(j==0)		this->c[i][j] = 0;			// Initial velocity of zero
			else if(j==this->n-1)	this->c[i][j] = dt*(sdd[j+1] + sdd[j])/2;
			else
			{
				float ds;
				if(!this->isQuaternion) ds = s[j+1] - s[j];		// Difference between start and end
				else			 ds = s[j];			// Rotations interpolate over difference in orientation
				
				this->c[i][j] = ds/dt - dt*(sdd[j+1] - 2*sdd[j])/6;
			}
		}
	}
}

/******************** Figure out where we are on the trajectory ********************/
int CubicSpline::get_spline_number(const float &time)
{
	int j = this->n;
	for(int i = -1; i < this->n-1; i++)
	{
		if(time < this->t[i+1])
		{
			j = i;
			break;
		}
	}
	return j;
}

#endif
