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
		 * Basic constructor for 3rd order polynomials (i.e. cubic spline) with zero start and end velocities.
		 * @param waypoint The points which to pass through in the trajectory.
		 * @param times The time at which to pass each point.
		 */
		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
		       const std::vector<DataType> &times)
		:
		Spline(waypoints,times,
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.front().size()),
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.back().size()),
		       3) {}
 		
 		/**
 		 * Constructor where start and end velocities are assumed zero.
 		 * @param waypoints An array of Eigen::Vector objects defining waypoints on the trajectory.
 		 * @param times The time at which to pass through each waypoint.
 		 */
 		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		       const std::vector<DataType> &times,
 		       const unsigned int &polynomialOrder)
		:
		Spline(waypoints,time,
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.front().size()),
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.back().size()),
		       polynomialOrder) {}
		 
 		/**
 		 * Full constructor which specifies the polynomial order, start and end velocities.
 		 *
 		 *
 		 */
 		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		       const std::vector<DataType> &times,
		       const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
		       const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
		       const unsigned int &polynomialOrder);
 		       
 };                                                                                                 // Semicolon needed after class declaration
 
   //////////////////////////////////////////////////////////////////////////////////////////////////
  //                                        Constructor                                           //
 //////////////////////////////////////////////////////////////////////////////////////////////////
 template <class DataType>
 Spline<DataType>::Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		          const std::vector<DataType> &times,
 		          const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
 		          const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
 		          const unsigned int &polynomialOrder)
{
	using namespace Eigen;
	
	// Set values in underlying Waypoints class
	this->_numberOfWaypoints = waypoints.size();
	this->_time = times;
	
	// Ensure input arguments are sound
	if(waypoints.size() != times.size())
	{
		throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
		                            "The number of waypoints does not match the number of times! ("
		                            + std::to_string(waypoints.size()) + " != " + std::to_string(times.size()) + ".");
	}
	else if(waypoints.size() < 2)
	{
		throw std::invalid_argument("[ERROR] [SPLINE] Constructor: "
		                            "A minimum of 2 waypoints is needed to construct a spline, "
		                            "but you provided " + std::to_string(waypoints.size()) + ".");
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
		
	if(this->_numberOfWaypoints == 2)
	{
		this->_trajectory.push_back(Polynomial<DataType>(waypoints.front(),
		                                                 waypoints.back(),
		                                                 startVelocity,
		                                                 endVelocity,
		                                                 times.front(),
		                                                 times.back(),
		                                                 polynomialOrder));
	}
	else
	{
		// Solve the velocities for all intermediary waypoints.
		// For convenience, assume a 3rd order polynomial.
		// The relationship between velocities and positions is governed by:
		// A*vel = B*pos ---> vel = A^-1*B*pos
		
		Matrix<DataType,Dynamic,Dynamic> A(this->_numberOfWaypoints,this->_numberOfWaypoints);
		
		A.setZero();
		
		Matrix<DataType,Dynamic,Dynamic> B = A;
		
		A(0,0) = 1;
		
		for(int i = 1; i < this->_numberOfWaypoints-1; i++)
		{
			DataType dt1 = times[i-1];
			DataType dt2 = times[i];
			
			A(i-1,i) = 1/dt1;
			A(i,i)   = 2*(1/dt1 + 1/dt2);
			A(i,i+1) = 1/dt2;
		
			B(i-1,i) = -3/(dt1*dt1);
			B(i,i)   =  3*(1/(dt1*dt1) - 1/(dt2*dt2));
			B(i,i+1) =  3/(dt2*dt2);
		}
		
		A(this->_numberOfWaypoints, this->_numberOfWaypoints) = 1;
		
		Matrix<DataType,Dynamic,Dynamic> C = A.partialPivLu().solve(B);                             // Makes calcs a little easier
		
		std::vector<Vector<DataType,Dynamic>> velocity; velocity.resize(this->_numberOfWaypoints);  // Array of velocities for all dimensions, waypoints
		
		for(int i = 0; i < velocity.size(); i++)
		{
			velocity[i].resize(waypoints[i].size());                                            // Resize the Eigen::Vector object within the std::vector object
			
			for(int j = 0; j < velocity[i].size(); j++)
			{
				velocity[i][j] = 0.0;
				
				// Note: waypoints for a single dimension are stored along the rows,
				//       so we need to project the rows of the C matrix on to the rows of
				//       the waypoint array
				for(int k = 0; k < velocity.front().size(); k++) velocity[i][j] += C(j,k)*waypoints[i][k];
			}
		}
		
		std::cout << "\nHere are all the velocity vectors:\n";
		for(auto &vel : velocity)
		{
			std::cout << vel.transpose() << "\n";
		}
		std::cout << std::endl;

		// Now create n-1 polynomials that define the spline
		for(int i = 0; i < this->_numberOfWaypoints-1; i++)
		{
			this->_trajectory.push_back(Polynomial<DataType>(waypoints[i],
				                                         waypoints[i+1],
				                                         velocity[i],
				                                         velocity[i+1],
				                                         times[i],
				                                         times[i+1],
				                                         polynomialOrder));
		}
	}
}

#endif
