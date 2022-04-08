    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A minimum acceleration trajectory across multiple points                   //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <Polynomial.h>                                                                            // Custom class

class CubicSpline
{
	public:
	
		CubicSpline();
		
		CubicSpline(const std::vector<Eigen::VectorXf> &waypoint,
					const std::vector<float> &time);
					
		CubicSpline(const std::vector<Eigen::AngleAxisf> &waypoint,
					const std::vector<float> &time);

		bool get_state(Eigen::VectorXf &pos,
					   Eigen::VectorXf &vel,
					   Eigen::VectorXf &acc,
					   const float &time);
		
		bool get_state(Eigen::AngleAxisf &rot,
					   Eigen::Vector3f &vel,
					   Eigen::Vector3f &acc,
					   const float &time);
					   
	private:
	
		bool isValid = false;                                                                      // Won't do calcs if this is false
		std::vector<Polynomial> spline;                                                            // Array of cubic polynomials
		std::vector<float> t;                                                                      // Time at which to pass each waypoint
		unsigned int m;                                                                            // Number of dimensions
		unsigned int n;                                                                            // Number of waypoints (for n-1 splines)
		
		bool inputs_are_sound(const std::vector<Eigen::VectorXf> &waypoint,
							  const std::vector<float> &time);
							  
		std::vector<Eigen::VectorXf> compute_velocities(const std::vector<Eigen::VectorXf> &pos,
														const std::vector<float> &time);
	
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Constructor for spline over real numbers                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &waypoint,
						 const std::vector<float> &time):
						 m(waypoint[0].size()),                                                    // Dimensions for each point
						 n(waypoint.size()),                                                       // Number of points
						 t(time);                                                                  // Time at which to pass each point
{
	// Check the input vectors are the same length
	if(waypoint.size() != time.size())
	{
		std::cerr << "[ERROR] [CUBICSPLINE] Constructor: "
				  << "Inputs are not of equal length! "
				  << "There were " << waypoint.size() << " waypoints "
				  << "and " << time.size() << " times." << std::endl;
	}
	else if(times_are_sound(time))
	{
		// Compute the displacements between each waypoint
		std::vector<Eigen::VectorXf> displacement; displacement.resize(this->n-1);
		for(int i = 0; i < this->n-1; i++)
		{
			for(int j = 0; j < this->m; j++) displacement[i][j] = waypoint[i+1][j] - waypoint[i][j]; // Get the difference in position
		}
		
		std::vector<Eigen::VectorXf> velocity = compute_velocties(displacement, time);             // Compute the velocities for each waypoint
		
		// Now create n-1 cubic polynomials for the spline
		for(int i = 0; i < this->n-1; i++) this->spline[i].push_back( Polynomial(waypoint[i],
																				 waypoint[i+1],
																				 velocity[i],
																				 velocity[i+1],
																				 time[i],
																				 time[i+1],
																				 3));
		this->isValid = true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Constructor for spline over orientations                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicSpline::CubicSpline(const std::vector<Eigen::AngleAxisf> &waypoint,
						 const std::vector<float> &time) :
						 m(3),
						 n(waypoint.size()),
						 t(time)						 
{
	// Check the input vectors are the same length
	if(waypoint.size() != time.size())
	{
		std::cerr << "[ERROR] [CUBICSPLINE] Constructor: "
				  << "Inputs are not of equal length! "
				  << "There were " << waypoint.size() << " waypoints "
				  << "and " << time.size() << " times." << std::endl;
				  
		return false;
	}
	else if(times_are_sound(time))
	{
		std::vector<Eigen::VectorXf> displacement; displacement.resize(this->n-1);
		
		// Compute the displacements in orientation
		for(int i = 0; i < this->n-1; i++)
		{
			Eigen::AngleAxisf dR = waypoint[i].inverse()*waypoint[i+1];                            // Difference in orientation
			
			float angle = dR.angle(); if(angle > M_PI) angle = 2*M_PI - angle;                     // If > 180 degrees, take the shorter path
			Eigen::Vector3f axis = dR.axis();                                                      // Get the axis of rotation
			
			displacement[i].resize(this->m);
			for(int j = 0; j < this->m; j++) displacement[i][j] = angle*axis[j];                   // Store the displacement as angle*axis
		}
		
		std::vector<Eigen::VectorXf> velocity = compute_velocities(displacement,time);             // Solve for the velocities at each waypoint
		
		// Now generate n-1 cubic polynomials for the spline
		for(int i = 0; i < this->n-1; i++) this->spline[i].push_back( Polynomial(waypoint[i],
																				 waypoint[i+1],
																				 velocity[i],
																				 velocity[i+1],
																				 time[i],
																				 time[i+1],
																				 3));
		this->isValid = true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Check that the inputs are sound                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicSpline::times_are_sound(const std::vector<float> &time)
{
	for(int i = 0; i < time.size()-1; i++)
	{
		if(time[i] == time[i+1] or time[i] > time[i+1])
		{
			std::cerr << "[ERROR] [CUBICSPLINE] Constructor: "
					  << "Times are not in ascending order! "
					  << "Time " << i+1 << " was " << time[i] << " seconds and "
					  << "time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;

			return false;
		}
	}
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Compute the velocities at each waypoint                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::VectorXf> CubicSpline::compute_velocities(const std::vector<Eigen::VectorXf> &dx,
														     const std::vector<float> &time)
{
	// Velocities are related to the displacements via A*xdot = B*dx.
	// Note that dx = x(i+1) - x(i) is used instead of just x because
	// it generalizes better to orientations as well
	Eigen::MatrixXf A = Eigen::MatrixXf::Identity(this->n, this->n);
	Eigen::MatrixXf B = Eigen::MatrixXf::Zeros(this->n, this->n-1);
	
	// Set the constraints at each waypoint
	for(int i = 1; i < this->n-1; i++)
	{
		double dt1 = this->t[i]   - this->t[i-1];
		double dt2 = this->t[i+1] - this->t[i];
		
		A(i,i-1) = 1/dt1;
		A(i,i)   = 2*(1/dt1 + 1/dt2);
		A(i,i+1) = 1/dt2;
		
		B(i,i-1) = 3/(dt1*dt1);
		B(i,i)   = 3/(dt2*dt2);
	}
	
	Eigen::MatrixXf C = A.inverse()*B;                                                             // This makes calcs a little easier
	
	std::vector<Eigen::VectorXf> xdot; xdot.resize(this->n);                                       // Value to be returned
	for(int i = 0; i < this->n; i++) xdot[i].resize(this->m);                                      // Set the appropriate dimensions
	
	// Solve the velocity eat each waypoint for each dimension
	// Note: Waypoints are stored column wise, but we need to solve velocities across each row
	for(int i = 0; i < this->m; i++)
	{
		Eigen::VectorXf displacement(this->n-1);
		for(int j = 0; j < displacement.size(); j++) displacement[j] = dx[j][i];                   // Get all waypoints for the ith row
		
		Eigen::VectorXf velocity = C*displacement;                                                 // Compute all n velocities
		for(int j = 0; j < velocity.size(); j++) xdot[j][i] = velocity[j];                         // Store velocities along ith row
	}
	
	return xdot;
}
						 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicSpline::get_state(Eigen::VectorXf &pos,
							Eigen::VectorXf &vel,
							Eigen::VectorXf &acc,
							const float &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [CUBICSPLINE] Constructor: "
				  << "Something went wrong during the construction of this object. "
				  << "Could not obtain the state." << std::endl;
				  
		return false;
	}
	else if(pos.size() != this->m or vel.size() != this->m or acc.size() != this->m)
	{
		std::cerr << "[ERROR] [CUBICSPLINE] Constructor: "
				  << "Input vectors are not the correct length."
				  << "This object has " this->m << " dimensions but "
				  << "pos had " << pos.size() << " elements, "
				  << "vel had " << vel.size() << " elements, and "
				  << "acc had " << acc.size() << " elements." << std::endl;
				  
		return false;
	}
	else
	{
		int j;
		if(time < this->t[0])               j = 0;                                                 // Not yet started, stay on first spline
		else if(time >= this->t[this->n-1]) j = this->n-1;                                         // Finished, stay on last spline
		else                                                                                       // Somewhere in between...
		{
			for(int i = 1; i < this->n-1; i++)
			{
				if(time < this->time[i])                                                           // Not yet reached ith waypoint...
				{
					j = i-1;                                                                       // ... so must be on spline i-1
					break;
				}
			}
		}
		
		return spline[j].get_state(pos, vel, acc, time);
	}
}

#endif