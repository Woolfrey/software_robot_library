#include <CubicSpline.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Constructor for spline over real numbers                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicSpline::CubicSpline(const std::vector<Eigen::VectorXf> &waypoint,
			 const std::vector<float> &time)
			 :
			 dimensions(waypoint[0].size()),
			 numPoints(waypoint.size()),                                                // Assign the number of waypoints
			 _time(time)
{
	std::string errorMessage = "[ERROR] [CUBIC SPLINE] Constructor: ";
	
	// Ensure inputs are of equal length
	if(waypoint.size() != time.size())
	{
		errorMessage += "Size of input arguments does not match. "
		                "The waypoint vector had " + std::to_string(waypoint.size()) + " elements, and "
		                "the time vector had " + std::to_string(time.size()) + " elements.";
		                      
		throw std::invalid_argument(errorMessage);
	}
	
	// Ensure times are in ascending order
	for(int i = 0; i < time.size()-1; i++)
	{
		if(time[i] == time[i+1])
		{
			errorMessage += "Time of " + std::to_string(time[i]) + " for waypoint " + std::to_string(i) + " "
			                "is equal to time of " + std::to_string(time[i+1]) + " for waypoint " + std::to_string(i+1) + ". "
			                "You cannot teleport between places in zero time.";
			                      
			throw std::logic_error(message);
		}
		else if(time[i] > time[i+1])
		{
			errorMessage += "Times are not in ascending order! "
			                "Waypoint " + std::to_string(i) + " was " + std::to_string(time[i]) + " seconds, and "
			                "waypoint " + std::to_string(i+1) + " was " + std::to_string(time[i+1]) + " seconds. "
			                "The arrow of time only moves in one direction.";
			                      
			throw std::logic_error(message);
		}
	}
	
	// Compute displacements between points to determine velocities
	// NOTE TO SELF: I need to change the underlying code so that it doesn't
	// require displacements
	
	std::vector<Eigen::VectorXf> displacement; displacement.resize(this->numPoints-1);
	for(int i = 0; i < this->numPoints -1; i++)
	{
		displacement[i].resize(waypoint[i].size());
		
		for(int j = 0; j < waypoint[0].size(); j++)
		{
			displacement[i][j] = waypoint[i+1][j] - waypoint[i][j];                     // Difference between this point and the next
		}
	}
	
	std::vector<Eigen::VectorXf> velocity = compute_velocities(displacement,time);              // Compute the velocities from the displacement
	
	// Now create n-1 cubic polynomials for the spline
	for(int i = 0; i < this->numPoints-1; i++)
	{
		try
		{
			this->spline.push_back(Polynomial(waypoint[i],
			                                  waypoint[i+1],
			                                  velocity[i],
			                                  velocity[i+1],
			                                  time[i],
			                                  time[i+1],
			                                  3));
		}
		catch(const std::exception &exception)
		{
			std::cout << exception.what() << std::endl;
			throw std::runtime_error("[ERROR] [CUBIC SPLINE] Constructor: Could not generate the spline.");
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Compute the velocities at each waypoint                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::VectorXf> CubicSpline::compute_velocities(const std::vector<Eigen::VectorXf> &dx,
							     const std::vector<float> &time)
{
	// Decent explanation here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (Fourth Edition)
        // Springer,  p. 258 - 276
        // Method is altered here to constraint velocities instead of accelerations
        
	// Velocities are related to the displacements via A*xdot = B*dx.
	// Note that dx = x(i+1) - x(i) is used instead of just x because
	// it generalizes better to orientations as well
	
	Eigen::MatrixXf A = Eigen::MatrixXf::Identity(this->numPoints, this->numPoints);
	Eigen::MatrixXf B = Eigen::MatrixXf::Zero(this->numPoints, this->numPoints-1);
	
	// Set the constraints at each waypoint
	for(int i = 1; i < this->numPoints-1; i++)
	{
		double dt1 = this->_time[i]   - this->_time[i-1];
		double dt2 = this->_time[i+1] - this->_time[i];
		
		A(i,i-1) = 1/dt1;
		A(i,i)   = 2*(1/dt1 + 1/dt2);
		A(i,i+1) = 1/dt2;
		
		B(i,i-1) = 3/(dt1*dt1);
		B(i,i)   = 3/(dt2*dt2);
	}
	
	Eigen::MatrixXf C = A.inverse()*B;                                                          // This makes calcs a little easier
	
	std::vector<Eigen::VectorXf> xdot; xdot.resize(this->numPoints);                            // Value to be returned
	for(int i = 0; i < this->numPoints; i++) xdot[i].resize(this->dimensions);                  // Set the appropriate dimensions
	
	// Solve the velocity eat each waypoint for each dimension
	// Note: Waypoints are stored column wise, but we need to solve velocities across each row
	for(int i = 0; i < this->dimensions; i++)
	{
		Eigen::VectorXf displacement(this->numPoints-1);
		for(int j = 0; j < displacement.size(); j++) displacement[j] = dx[j][i];            // Get all waypoints for the ith row
		
		Eigen::VectorXf velocity = C*displacement;                                          // Compute all n velocities
		for(int j = 0; j < velocity.size(); j++) xdot[j][i] = velocity[j];                  // Store velocities along ith row
	}
	
	return xdot;
}
						 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicSpline::get_state(Eigen::VectorXf &pos,
			    Eigen::VectorXf &vel,
			    Eigen::VectorXf &acc,
			    const float     &time)
{
	if(pos.size() != vel.size()
	or vel.size() != acc.size())
	{
		std::cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
			  << "Input vectors are not the same length! "
			  << "The position vector had " << pos.size() << " elements, "
			  << "the velocity vector had " << vel.size() << " elements, and "
			  << "the acceleration vector had " << acc.size() << " elements.";
			  
		return false;
	}
	else if(pos.size() != this->dimensions)
	{
		std::cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
	                  << "This spline has " << this->dimensions << " dimensions, "
	                  << "but the input vectors had " << pos.size() << " elements.";

		return false;
	}
	else
	{
		int splineNumber;
		
		if(time < this->_time.front())     splineNumber = 0;                                // Not yet started; on the first spline
		else if(time > this->_time.back()) splineNumber = this->numPoints-1;                // Finished; on the last spline
		else                                                                                // Somewhere inbetween
		{
			for(int i = 0; i < this->numPoints-1; i++)
			{
				if(time < this->_time[i])                                           // Not yet reached ith point....
				{
					splineNumber = i-1;                                         // ... so must be on spline i
					break;
				}
			}
		}
		
		return spline[splineNumber].get_state(pos,vel,acc,time);
	}
}
