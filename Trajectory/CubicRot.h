/*
*	A minimum acceleration trajectory through multiple orientations
*/

#ifndef CUBICROT_H_
#define CUBIC_H_

#include <CubicSpline.h>							// Base class
#include <Eigen/Geometry>							// Eigen::Quaternionf, Eigen::AngleAxisf
#include <vector>								// std::vector

class CubicRot	: public CubicSpline
{
	public:
		CubicRot() : CubicSpline () {}
		
		CubicRot(const std::vector<Eigen::Quaternionf> &rots,
				const std::vector<float> &times);
				
		bool get_state(Eigen::Quaternionf &rot,
				Eigen::Vector3f &vel,
				Eigen::Vector3f &acc,
				const float &time);
	
	private:
		std::vector<Eigen::Quaternionf> Q;				// Vector of waypoints

};										// Semicolon needed after class declaration

/******************** Constructor ********************/
CubicRot::CubicRot(const std::vector<Eigen::Quaternionf> &rots,
			const std::vector<float> &times)
			: CubicSpline()
			, Q(rots)						// Orientation at each waypoint
{
	// Need to set these values in the base class
	this->m = 3; this->n = rots.size();
	
	// Check the input dimensions are sound
	if(rots.size() != times.size())
	{
		std::cerr << "[ERROR][CUBICROT] Constructor : Vectors are not of equal length!" << std::endl;
		std::cerr << "rots: " << rots.size() << " times: " << times.size() << std::endl;
	}
	else if(this->n < 2)
	{
		std::cerr << "[ERROR][CUBICROT] Constructor : A minimum of 2 points is required to create a trajectory." << std::endl;
	}
	else
	{
		this->isNotValid = false;

		// Check the time are in ascending order
		for(int i = 0; i < this->t.size()-1; i++)
		{
			if(this->t[i] > this->t[i+1])
			{
				std::cerr << "[ERROR][CUBICSPLINE] Constructor : Times are not in ascending order!" << std::endl;
				this->isNotValid = true;
				break;
			}
		}
		
		if(this->n == 2)
		{
			// NEED TO FILL THIS IN
			this->isNotValid = true;
		}
		else
		{
			// For each spline, compute the *difference* in rotation
			std::vector<Eigen::VectorXf> points;
			for(int i = 0; i < this->Q.size() - 1; i++)
			{
				Eigen::Quaternionf dQ = this->Q[i].conjugate()*this->Q[i+1];	// Get the difference in rotation
				Eigen::AngleAxisf aa(dQ);					// Convert to angle-axis representation
				float angle = aa.angle();					// Get the angle between the two orientations
				if(angle > M_PI) angle = 2*M_PI - angle;			// If angle > 180 degrees, take the shorter path...
				points.push_back(angle*aa.axis());				// Store as angle*axis
			}
			points.push_back(Eigen::VectorXf::Zero(3));				// Not actually used - assume zero rotation
			
			// Relationship between acceleration & position: A*sddot = B*s
			Eigen::MatrixXf A(this->n, this->n); A.setZero();
			Eigen::MatrixXf B(this->n, this->n); B.setZero();
			
			// Set constraints for the start and end of trajectory
			A(0,0) = (this->t[1] - this->t[0])/3;
			A(0,1) = (this->t[1] - this->t[0])/6;
			B(0,0) = 1/(this->t[1] - this->t[0]);

			// Set the constraints for the end of the trajectory
			A(this->n-1, this->n-2) = (this->t[this->n-1] - this->t[n-2])/6;
			A(this->n-1, this->n-1) = (this->t[this->n-1] - this->t[n-2])/3;
			B(this->n-1, this->n-2) = -1/(this->t[this->n-1] - this->t[this->n-2]);
			
			// Set the constraints for each waypoint inbetween
			float dt1, dt2;
			for(int i = 1; i < this->n-2; i++)
			{
				dt1 = this->t[i] - this->t[i-1];
				dt2 = this->t[i+1] - this->t[i];
				
				A(i,i-1)	= dt1/6;
				A(i,i)		= (dt1 + dt2)/3;
				A(i, i+1)	= dt2/6;
				
				B(i,i-1) = -1/dt1;
				B(i,i)   = 1/dt2;
			}
			
			Eigen::MatrixXf C = A.inverse()*B; 				// A*sddot = B*s ---> sddot = A^-1*B*s
			Eigen::VectorXf s(this->n), sdd(this->n);			// Pos, acc for a single dimension
			
			// Resize the rows of the array
			this->a.resize(this->m);
			this->b.resize(this->m);
			this->c.resize(this->m);
			this->d.resize(this->m);					
			
			for(int i = 0; i < this->m; i++)
			{
				// Resize the columns of the array	
				this->a[i].resize(this->n-1);				// There are only n-1 splines
				this->b[i].resize(this->n-1);
				this->c[i].resize(this->n-1);
				this->d[i].resize(this->n-1);
				
				for(int j = 0; j < this->n; j++)			// There are only n-1 splines
				{
					s[j] = points[j][i];				// points is stored as n waypoints each with m dimensions
				}
				sdd = C*s;						// Solve the acceleration at each waypoint
				
				// Compute the coefficient for the n-1 splines
				for(int j = 0; j < this->n-1; j++)
				{
					double dt = this->t[j+1] - this->t[j];	// Difference in time
					
					// Note to self: coefficients a, b, c, d are stored as
					// m dimensions across n waypoints
					this->a[i][j] = (sdd[j+1] - sdd[j])/(6*dt);
					this->b[i][j] = sdd[j]/2;
					if(j==0)		this->c[i][j] = 0;
					else if(j < this->n-1)	this->c[i][j] = s[j] - dt*(sdd[j+1] - 2*sdd[j])/6;
					else			this->c[i][j] = dt*(sdd[j+1] + sdd[j])/2;
					this->d[i][j] = 0;				// Starting position of zero for difference over rotation
				}
			}
		}
	}
}

/******************** Get the desired state for the given time ********************/
bool CubicRot::get_state(Eigen::Quaternionf &rot, Eigen::Vector3f &vel, Eigen::Vector3f &acc, const float &time)
{
	Eigen::VectorXf p(3), v(3), a(3);					// Position, velocity, acceleration
	if(CubicSpline::get_state(p, v, a, time))
	{
		Eigen::AngleAxisf temp(p.norm(), p.normalized());		// Separate out the angle and axis
		Eigen::Quaternionf dQ(temp);					// Convert to quaternion
		
		// Figure out which spline we are on
		int j;
		if(time < this->t[0]) j = 0;					// Not yet started; first spline
		else if(time < this->t.back())				// Somewhere inbetween
		{
			for(int i = 0; i < this->n; i++)
			{
				if(time < this->t[i])	j = i-1;		// Not yet reached waypoint i, so must be on i-1
			}
		}
		else j = this->n-1;						// Finished, so start from spline n-1
		
		rot = this->Q[j]*dQ;
		vel = v;
		acc = a;
		
		return true;
	}
	else
	{
		std::cerr << "[ERROR][CUBICROT] get_state() : Could not obtain the desired state." << std::endl;
		
		rot = this->Q[0];						// Remain at the start
		vel.setZero(); acc.setZero();					// Don't move
		
		return false;
	}
}

#endif
