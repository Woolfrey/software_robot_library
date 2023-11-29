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
		Spline(waypoints,times,
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.front().size()),
		       Eigen::Vector<DataType,Eigen::Dynamic>::Zero(waypoints.back().size()),
		       polynomialOrder) {}
		 
 		/**
 		 * Full constructor which specifies the polynomial order, start and end velocities.
 		 * @param waypoints A vector of Eigen::Vector objects specifying points to pass through.
 		 * @param times The time at which to pass through each waypoint.
 		 * @startVelocity The initial speed for the trajectory.
 		 * @endVelocity The final speed for the trajectory.
 		 * @polynomialOrder The number of coefficients in the polynomial.
 		 */
 		Spline(const std::vector<Eigen::Vector<DataType,Eigen::Dynamic>> &waypoints,
 		       const std::vector<DataType> &times,
		       const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
		       const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
		       const unsigned int &polynomialOrder);
 		       
};                                                                                                 // Semicolon needed after class declaration
 
typedef Spline<float>  Splinef;
typedef Spline<double> Splined;
 
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
			DataType dt1 = times[i]   - times[i-1];
			DataType dt2 = times[i+1] - times[i];
			
			if(dt1 == 0)
			{
				throw std::logic_error("[ERROR] [SPLINE] Constructor: "
				                       "Time for waypoint " + std::to_string(i-1) + " "
				                       "is the same as time for waypoint " + std::to_string(i) + 
				                       "(" + std::to_string(times[i-1]) + " == " + std::to_string(times[i]) + "). "
				                       "You cannot move in zero time! ಠ_ಠ");
			}
			else if(dt1 < 0)
			{
				throw std::logic_error("[ERROR] [SPLINE] Constructor: "
				                       "Times are not in ascending order. (Time " + std::to_string(i-1) + " of "
				                       + std::to_string(times[i-1]) + " seconds == time " + std::to_string(i) + " of "
				                       + std::to_string(times[i]) + " seconds). "
				                       "You cannot go backwards in time! ヽ༼ ಠ益ಠ ༽ﾉ");
			}
			
			A(i,i-1) = 1/dt1;
			A(i,i)   = 2*(1/dt1 + 1/dt2);
			A(i,i+1) = 1/dt2;
		
			B(i,i-1) = -3/(dt1*dt1);
			B(i,i)   =  3*(1/(dt1*dt1) - 1/(dt2*dt2));
			B(i,i+1) =  3/(dt2*dt2);
		}
		
		A(this->_numberOfWaypoints-1, this->_numberOfWaypoints-1) = 1;
		
		PartialPivLU<Matrix<DataType,Dynamic,Dynamic>> LU(A);                               // Compute decompisition to make things faster

		unsigned int dim = waypoints[0].size();                                             // Number of dimensions
		
		Eigen::Matrix<DataType,Dynamic,Dynamic> velocity(dim,this->_numberOfWaypoints);
		
		for(int i = 0; i < dim; i++)
		{
			Vector<DataType,Dynamic> pos(this->_numberOfWaypoints);

			for(int j = 0; j < this->_numberOfWaypoints; j++) pos(j) = waypoints[j][i]; // Get the ith dimension of the jth waypoint
			
			Vector<DataType,Dynamic> des(this->_numberOfWaypoints); des.setZero();      // All numbers inbetween should be zero
			des(0)   = startVelocity(i);                                                // Start point of the ith dimension
			des(this->_numberOfWaypoints-1) = endVelocity(i);                           // End point for the ith dimension

			velocity.row(i) = (LU.solve(B*pos + des)).transpose();                      // Solve the velocities
		}
		
		// Now create n-1 polynomials that define the spline
		for(int i = 0; i < this->_numberOfWaypoints-1; i++)
		{
			this->_trajectory.push_back(Polynomial<DataType>(waypoints[i],
				                                         waypoints[i+1],
				                                         velocity.col(i),
				                                         velocity.col(i+1),
				                                         times[i],
				                                         times[i+1],
				                                         polynomialOrder));
		}
	}
}

#endif
