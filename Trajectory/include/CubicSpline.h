    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A minimum acceleration trajectory across multiple points                   //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
		// Constructors
		CubicSpline() {}                                                                    // Won't compile without {} for some reason ಠ_ಠ 

		CubicSpline(const vector<Vector<DataType,Dynamic>> &waypoint,
			    const vector<DataType>                 &time);

		// Methods
		bool get_state(Vector<DataType,Dynamic> &pos,
			       Vector<DataType,Dynamic> &vel,
			       Vector<DataType,Dynamic> &acc,
			       const DataType           &time);
					   
	private:
	
		// Private properties
		vector<Polynomial> spline;                                                          // Array of cubic polynomials
		
		vector<DataType> _time;                                                             // Array of times to reach each waypoint
		
		unsigned int _dimensions;                                                           // Dimensions for the vector
		
		unsigned int _numPoints;                                                            // Number of waypoints
			
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                          //
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
			                  "is greater than time of " + to_string(time[i])) + " for waypoint "
			                  + to_string(i) + ". The arrow of time only moves in one direction.");
		}
	}
	
	// Compute the velocities for each of the waypoints
	Vector<DataType,Dynamic> velocity::zero(this->numPoints);
	
	// Now create n-1 cubic polynomial splines
	for(int i = 0; i < this->numPoints-1; i++)
	{
		try
		{
			this->spline.emplace_back(waypoint[i],
			                          waypoint[i+1],
			                          velocity[i],
			                          velocity[i+1],
			                          time[i],
			                          time[i+1],
			                          3)
		}
		catch(const exception &exception)
		{
			cout << exception.what() << endl;
			
			throw runtime_error("[ERROR] [CUBIC SPLINE] Constructor: "
			                    "Could not generate the spline.");
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
	else if(pos.size() != this->dimensions)
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
