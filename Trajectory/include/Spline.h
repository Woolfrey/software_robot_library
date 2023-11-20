/**
 * @file   Spline.h
 * @author Jon Woolfrey
 * @date   November 2023
 * @brief  Specifies a series of polynomial trajectories joined together.
 */
 
 #ifndef SPLINE_H_
 #define SPLINE_H_
 
 #include <Eigen/Dense>                                                                             // Allows matrix decomposition
 #include <Polynomial.h>
 #include <Waypoints.h>
 
 template <class DataType>
 class Spline : public Waypoints<DataType, Polynomial<DataType>>
 {
 	public:
 		/**
 		 * Constructor where start and end velocities are assumed zero.
 		 * @param waypoints An array of Eigen::Vector objects defining waypoints on the trajectory.
 		 * @param times The time at which to pass through each waypoint.
 		 */
 		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		       const std::vector<DataType> &times,
 		       const unsigned int &polynomialOrder)
		:
		Spline(waypoints, times, polynomialOrder,
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.front().size()),
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.back().size()));
		 
 		/**
 		 * Constructor with start and end velocities specified.
 		 *
 		 */
 		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		       const std::vector<DataType> &times,
		       const unisgned int &polynomialOrder,
 		       const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
 		       const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity);
 		       
		/**
		 * Query the state (position, velocity, acceleration) for the given time.
		 * @param time The point at which to evaluate the trajectory.
		 * @return A State data structure.
		 */
 		State<DataType> query_state(const DataType &time);
 		
	private:
	
 };                                                                                                 // Semicolon needed after class declaration
 
   //////////////////////////////////////////////////////////////////////////////////////////////////
  //                                        Constructor                                           //
 //////////////////////////////////////////////////////////////////////////////////////////////////
 template <class DataType>
 Spline<DataType>::Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		          const std::vector<DataType> &times,
 		          const unsigned int polynomialOrder,
 		          const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
 		          const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity)
 		          :
 		          _numberOfWaypoints(waypoints.size()),
 		          _times(times)
{
	using namespace Eigen;
	
	// Ensure input arguments are sound
	if(polynomialOrder != 3)
	{
		throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
		                            "The polynomial order argument was " + std::to_string(polynomialOrder) +
		                            " but the Spline class can only currently handle 3.");
	}
	else if(waypoints.size() != times.size())
	{
		throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
		                            "The number of waypoints does not match the number of times! ("
		                            + std::to_string(waypoints.size()) + " != " + std::to_string(times.size()) + ".");
	}
	
	// Make sure times are in ascending order
	for(int i = 0; i < times.size()-1; i++)
	{
		if(times[i] >= times[i+1])
		{
			throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
						    "Times must be in ascending order. "
			                            "Time for waypoint " + std::to_string(i+1) + " is equal to "
			                            "time for waypoint " + std::to_string(i+2) + "("
			                            + std::to_string(times[i]) + " >= " + std::to_string(times[i+1]) + ".");
		}
	}
	
	// Compute the velocities for each of the waypoints.
	// They are governed by the relationship A*xdot = B*x
	
	Matrix<DataType,Dynamic,Dynamic> A(this->_numberOfPoints,this->_numberOfPoints);
	A.setZero();
	
	Matrix<DataType,Dynamic,Dynamic> B = A;
	
	A(0,0) = 1;
	for(int i = 1; i < this->_numberOfPoints-1; i++)
	{
		DataType dt1 = times[i-1];
		DataType dt2 = times[i];
		
		A(i-1,i) = 1/dt1;
		A(i,i)   = 2*(1/dt1 + 1/dt2);
		A(i,i+1) = 1/dt2;
	
		B(i-1,i) = -3/(dt1*dt1);
		B(i,i)   =  3(1/(dt1*dt1) - 1/(dt2*dt2));
		B(i,i+1) =  3/(dt2*dt2);
	}
	A(this->_numberOfPoints, this->_numberOfPoints) = 1;
	
	Matrix<DataType,Dynamic,Dynamic> C = A.partialPivLu().solve(B);                             // Makes calcs a little easier
	
	std::vector<Vector<DataType,Dynamic>> velocity; velocity.resize(this->_numberOfPoints);     // Array of velocities for all dimensions, waypoints

	for(int i = 0; i < this->_numberOfPoints-1; i++)
	{
	
	}
/*	
	for(int i = 0; i < this->_numberOfPoints; i++)
	{
		velocity[i].resize(this->_dimensions);                                              // Resize the Eigen::Vector object within the std::vector object
		
		for(int j = 0; j < this->_dimensions; j++)
		{
			velocity[i][j] = 0.0;
			
			// Note: waypoints for a single dimension are stored along the rows,
			//       so we need to project the rows of the C matrix on to the rows of
			//       the waypoint array
			for(int k = 0; k < this->_dimensions; k++) velocity[i][j] += C(j,k)*waypoint[i][k];
		}
	}

	// Now create n-1 cubic polynomial splines
	for(int i = 0; i < this->numPoints-1; i++)
	{
		try
		{
			this->spline.emplace_back(waypoint[i], waypoint[i+1], velocity[i], velocity[i+1], time[i], time[i+1], 3);
		}
		catch(const exception &exception)
		{
			std::cerr << exception.what() << std::endl;                                 // Use std::endl instead of "\n"; for std::cerr?
			
			throw runtime_error("[ERROR] [CUBIC SPLINE] Constructor: Could not generate the spline.");
		}
	}
	*/
}

#endif
