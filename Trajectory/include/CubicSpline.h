/**
 * @file   CubicSpline.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Specifies a series of cubic polynomials connecting several waypoints.
 */

#ifndef CUBICSPLINE_H_
#define CUBICSPLINE_H_

#include <Eigen/Dense>                                                                              // Matrix inverse
#include <Polynomial.h>                                                                             // Custom class

using namespace Eigen;
using namespace std;

template <class DataType>
class CubicSpline
{
	public:

		/**
		 * An empty constructor.
		 */
		CubicSpline() {}                                                                    // Won't compile without {} for some reason ಠ_ಠ 

		/**
		 * A fully defined constructor.
		 * @param waypoint An array of waypoints to move through.
		 * @param time The time at which to reach every waypoint.
		 */
		CubicSpline(const vector<Vector<DataType,Dynamic>> &waypoint,
			    const vector<DataType>                 &time);

		/**
		 * Query the current trajectory state for the given input time.
		 * This is a virtual function and must be defined in any derived class.
		 * @param pos The current position.
		 * @param vel The current velocity.
		 * @param acc The current acceleration.
		 * @param time The time at which to query the state.
		 * @return Returns true if there were no problems.
		 */
		bool get_state(Vector<DataType,Dynamic> &pos,
			       Vector<DataType,Dynamic> &vel,
			       Vector<DataType,Dynamic> &acc,
			       const DataType           &time);
					   
	private:
	
		vector<Polynomial<DataType>> spline;                                                ///< Array of cubic polynomials
		
		vector<DataType> _time;                                                             ///< Array of times to reach each waypoint
		
		unsigned int _dimensions;                                                           ///< Dimensions for the vector
		
		unsigned int _numPoints;                                                            ///< Number of waypoints
			
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
CubicSpline<DataType>::CubicSpline(const vector<Vector<DataType,Dynamic>> &waypoint,
			           const vector<DataType>                 &time)
			           :
			           _dimensions(waypoint[0].size()),
			           _numPoints(waypoint.size()),                                     // Assign the number of waypoints
			           _time(time)
{
	// Ensure inputs are of equal length
	if(waypoint.size() != time.size())
	{
		throw logic_error("[ERROR] [CUBIC SPLINE] Constructor: "
		                  "Number of elements do not match. "
		                  "There was " + to_string(waypoint.size()) + " waypoints, and "
		                  + to_string(time.size()) + " times.");
	}
	
	// Ensure times are in ascending order
	for(int i = 0; i < time.size()-1; i++)
	{
		if(time[i] == time[i+1])
		{
			throw logic_error("[ERROR] [CUBIC SPLINE] Constructor: "
			                  "Time of " + to_string(time[i]) + " for waypoint " + to_string(i) + " "
			                  "is equal to time of " + to_string(time[i+1]) + " for waypoint " + to_string(i+1) + ". "
			                  "You cannot be in two places at once.");
		}
		else if(time[i] > time[i+1])
		{
			throw logic_error("[ERROR] [CUBIC SPLINE] Constructor: "
			                  "Time of " + to_string(time[i]) + " for waypoint " + to_string(i) + " "
			                  "is greater than time of " + to_string(time[i]) + " for waypoint "
			                  + to_string(i) + ". The arrow of time only moves in one direction.");
		}
	}
	
	// Compute the velocities for each of the waypoints. They are governed by the relationship:
	// A*xdot = B*x
	
	Matrix<DataType,Dynamic,Dynamic> A;
	A.resize(this->_numPoints,this->_numPoints);
	A.setZero();
	
	Matrix<DataType,Dynamic,Dynamic> B = A;
	
	A(0,0) = 1;
	for(int i = 1; i < this->_numPoints-1; i++)
	{
		DataType dt1 = time[i-1];
		DataType dt2 = time[i];
		
		A(i-1,i) = 1/dt1;
		A(i,i)   = 2*(1/dt1 + 1/dt2);
		A(i,i+1) = 1/dt2;
	
		B(i-1,i) = -3/(dt1*dt1);
		B(i,i)   =  3(1/(dt1*dt1) - 1/(dt2*dt2));
		B(i,i+1) =  3/(dt2*dt2);
	}
	A(this->_numPoints, this->_numPoints) = 1;
	
	Matrix<DataType,Dynamic,Dynamic> C = A.partialPivLu().solve(B);                             // Makes calcs a little easier
	
	vector<Vector<DataType,Dynamic>> velocity; velocity.resize(this->_numPoints);               // Array of velocities for all dimensions, waypoints
	
	for(int i = 0; i < this->_numPoints; i++)
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
}
						 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool CubicSpline<DataType>::get_state(Vector<DataType,Dynamic> &pos,
			              Vector<DataType,Dynamic> &vel,
			              Vector<DataType,Dynamic> &acc,
			              const DataType           &time)
{
	if(pos.size() != vel.size()
	or vel.size() != acc.size())
	{
		cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
	             << "Input vectors are not the same length! "
		     << "The position vector had " << pos.size() << " elements, "
		     << "the velocity vector had " << vel.size() << " elements, and "
		     << "the acceleration vector had " << acc.size() << " elements.\n";
			  
		return false;
	}
	else if(pos.size() != this->_dimensions)
	{
		cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
	             << "This spline has " << this->dimensions << " dimensions, "
	             << "but the input vectors had " << pos.size() << " elements.\n";

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

#endif
