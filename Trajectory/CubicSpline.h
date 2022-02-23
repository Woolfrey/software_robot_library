    ////////////////////////////////////////////////////////////////////////////////////////////////////  
   //                                                                                                //
  //                  A minimum acceleration trajectory across multiple points                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <Eigen/Geometry>                                                                          // Eigen::Quaternionf and the like
#include <iostream>                                                                                // std::cerr << lol << std::endl
#include <vector>                                                                                  // std::vector

class CubicSpline
{
	public:
		CubicSpline() {}
		
		CubicSpline(const std::vector<Eigen::VectorXf> &points, const std::vector<float> &times);

		bool get_state(Eigen::VectorXf &pos,
                               Eigen::VectorXf &vel,
                               Eigen::VectorXf &acc,
                               const float &time);
				
	protected:                                                                                 // CubicRot class can access these

		bool isNotValid = true;                                                            // Object will not do calcs if this is true
		int m, n;                                                                          // n waypoints
		std::vector<float> t;                                                              // Time to reach each waypoint
		std::vector<std::vector<float>> a, b, c, d;                                        // Spline coefficients
		bool check_inputs();
		
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &points,
                         const std::vector<float> &times):
                         m(points[0].size()),                                                      // Number of dimensions
                         n(points.size()),                                                         // Number of waypoints (for n-1 splines)
                         t(times)	
{
	// Check the inputs are of equal length
	if(points.size() != times.size())
	{
		std::cerr << "[ERROR][CUBICSPLINE] Constructor : Input vectors are not the same length!" << std::endl;
		std::cerr << "points: " << points.size() << " times: " << times.size() << std::endl;
	}
	else if(this->n < 2)
	{
		std::cerr << "[ERROR][CUBICSPLINE] Constructor : A minimum of 2 points is needed to create a trajectory!" << std::endl;
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
			// NEED TO FILL THIS
			this->isNotValid = true;
		}
		else
		{
			// Relationship between acceleration & position: A*sddot = B*s
			Eigen::MatrixXf A(this->n, this->n); A.setZero();
			Eigen::MatrixXf B(this->n, this->n); B.setZero();
			
			// Set constraints for the start of the trajectory
			A(0,0) = (this->t[1] - this->t[0])/2;
			A(0,1) = (this->t[1] - this->t[0])/2 + (this->t[2] - this->t[1])/3;
			A(0,2) = (this->t[2] - this->t[1])/6;
			B(0,1) = -1/(this->t[2] - this->t[1]);
			B(0,2) =  1/(this->t[2] - this->t[1]);
			
			// Set the constraints for the end of the trajectory
			A(this->n-1, this->n-2) = (this->t[this->n-1] - this->t[n-2])/6;
			A(this->n-1, this->n-1) = (this->t[this->n-1] - this->t[n-2])/3;			
			B(this->n-1, this->n-2) = 1/(this->t[this->n-1] - this->t[this->n-2]);
			B(this->n-1, this->n-1) = -1/(this->t[this->n-1] - this->t[this->n-2]);
			
			// Set the constraints for each waypoint inbetween
			float dt1, dt2;
			for(int i = 1; i < this->n-2; i++)
			{
				dt1 = this->t[i] - this->t[i-1];
				dt2 = this->t[i+1] - this->t[i];

				A(i,i-1)  = dt1/6;
				A(i,i)    = (dt1 + dt2)/3;
				A(i, i+1) = dt2/6;

				B(i,i-1) = 1/dt1;
				B(i,i)   = -1/dt1 - 1/dt2;
				B(i,i+1) = 1/dt2;
			}
			
			Eigen::MatrixXf C = A.inverse()*B;                                         // A*sddot = B*s ---> sddot = A^-1*B*s
			Eigen::VectorXf s(this->n), sdd(this->n);                                  // Pos, acc for a single dimension
			
			// Resize the rows of the array
			this->a.resize(this->m);
			this->b.resize(this->m);
			this->c.resize(this->m);
			this->d.resize(this->m);
			
			for(int i = 0; i < this->m; i++)
			{	
				// Resize the columns of the array	
				this->a[i].resize(this->n-1);
				this->b[i].resize(this->n-1);
				this->c[i].resize(this->n-1);
				this->d[i].resize(this->n-1);
				
				for(int j = 0; j < this->n; j++)                                   // There are only n-1 splines
				{
					// NOTE: points has n waypoints with m dimensions
					s[j] = points[j][i];                                       // Fill in the position for each waypoint
				}
				sdd = C*s;                                                         // Solve the acceleration at each waypoint
				
				// Compute the coefficient for the n-1 splines
				for(int j = 0; j < this->n-1; j++)
				{
					double dt = this->t[j+1] - this->t[j];                     // Difference in time
					
					// NOTE: a, b, c, d are m dimensions across n-1 splines
					this->a[i][j] = (sdd[j+1] - sdd[j])/(6*dt);
					this->b[i][j] = sdd[j]/2;
					if(j == 0) this->c[i][j] = 0;
					else if(j < this->n-1)
					{
						this->c[i][j] = (s[j+1] - s[j])/dt - dt*(sdd[j+1] - 2*sdd[j])/6;
					}
					else this->c[i][j] = dt*(sdd[j+1] + sdd[j])/2;
					this->d[i][j] = s[j];                                      // Position at the start of each spline
				}
			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicSpline::get_state(Eigen::VectorXf &pos, Eigen::VectorXf &vel, Eigen::VectorXf &acc, const float &time)
{
	// Check the input arguments are the same size
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		std::cerr << "[ERROR][CUBICSPLINE] get_state() : Input vectors are not of equal length!" << std::endl;
		std::cerr << "pos: " << pos.size() << " vel: " << vel.size() << " acc: " << acc.size() << std::endl;
				
		pos.resize(this->m);
		for(int i = 0; i < this->m; i++) pos[i] = d[i][0];                                 // Remain at the start
		vel.setZero(); acc.setZero();                                                      // Don't move
		
		return false;
	}
	else if(this->isNotValid)
	{
		std::cerr << "[ERROR][CUBICSPLINE] get_state() : Something went wrong during construction of this object." << std::endl;
		
		for(int i = 0; i < this->m; i++) pos[i] = d[i][0];                                 // Remain at the start
		vel.setZero(); acc.setZero();                                                      // Don't move
		
		return false;
	}
	else
	{
		// Figure out where we are on the trajectory
		int j; float dt;
		if(time < this->t[0])                                                              // Not yet started
		{
			j = 0;                                                                     // On the first spline
			dt = 0;                                                                    // Remain at the beginning
		}
		else if(time < this->t.back())                                                     // Somewhere in the middle
		{
			for(int i = 1; i < this->n-1; i++)
			{
				if(time < this->t[i])                                              // Not yet reached the ith waypoint...
				{
					j = i-1;                                                   // ... so must be on spline i-1
					dt = time - this->t[j];                                    // Elapsed time since start of spline i-1
					break;
				}
			}
		}
		else                                                                               // Must be finised
		{
			j = this->n-1;                                                             // Start from the last spline
			dt = this->t[j] - this->t[j-1];                                            // Interpolate up to the last waypoint
		}
		
		// Interpolate along the jth spline
		for(int i = 0; i < this->m; i++)
		{
			pos[i] =   this->a[i][j]*pow(dt,3) +   this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
			vel[i] = 3*this->a[i][j]*pow(dt,2) + 2*this->b[i][j]*dt        + this->c[i][j];
			acc[i] = 6*this->a[i][j]*dt        + 2*this->b[i][j];
		}
		
		return true;
	}
}

#endif
