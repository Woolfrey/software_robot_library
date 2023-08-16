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
	
		CubicSpline() {}                                                                    // Won't compile without {} for some reason ಠ_ಠ 

		CubicSpline(const vector<Vector<DataType,Dynamic>> &waypoint,
			    const vector<DataType>                 &time);

		bool get_state(Vector<DataType,Dynamic> &pos,
			       Vector<DataType,Dynamic> &vel,
			       Vector<DataType,Dynamic> &acc,
			       const DataType           &time);
					   
	private:
	
		vector<Polynomial> spline;                                                          // Array of cubic polynomials
		
		vector<DataType> _time;                                                             // Array of times to reach each waypoint
		
		unsigned int _dimensions;                                                           // Dimensions for the vector
		
		unsigned int _numPoints;                                                            // Number of waypoints
							  
		vector<Vector<DataType,Dynamic>> compute_velocities(const vector<Vector<DataType,Dynamic>> &pos,
								    const vector<DataType> &time);
	
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
	
	
	
	// Compute displacements between points to determine velocities
	// NOTE TO SELF: I need to change the underlying code so that it doesn't
	// require displacements
	
	vector<Vector<DataType,Dynamic>> displacement; displacement.resize(this->numPoints-1);
	for(int i = 0; i < this->numPoints -1; i++)
	{
		displacement[i].resize(waypoint[i].size());
		
		for(int j = 0; j < waypoint[0].size(); j++)
		{
			displacement[i][j] = waypoint[i+1][j] - waypoint[i][j];                     // Difference between this point and the next
		}
	}
	
	vector<Vector<DataType,Dynamic>> velocity = compute_velocities(displacement,time);              // Compute the velocities from the displacement
	
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
		catch(const exception &exception)
		{
			cout << exception.what() << endl;
			throw runtime_error("[ERROR] [CUBIC SPLINE] Constructor: Could not generate the spline.");
		}
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//                           Constructor for spline over real numbers                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicSpline::CubicSpline(const vector<Eigen::Matrix<DataType,6,1>> &waypoint,
                         const vector<DataType> &time)
        :
        dimensions(waypoint[0].size()),
        numPoints(waypoint.size()),                                                // Assign the number of waypoints
        _time(time)
{
    vector<Vector<DataType,Dynamic>> waypoint_temp;

    for (const auto & i : waypoint)
    {
        waypoint_temp.emplace_back(i);
    }

    CubicSpline cs(waypoint_temp,time);
    this->spline = cs.spline;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Compute the velocities at each waypoint                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Vector<DataType,Dynamic>> CubicSpline::compute_velocities(const vector<Vector<DataType,Dynamic>> &dx,
							     const vector<DataType> &time)
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
	
	vector<Vector<DataType,Dynamic>> xdot; xdot.resize(this->numPoints);                            // Value to be returned
	for(int i = 0; i < this->numPoints; i++) xdot[i].resize(this->dimensions);                  // Set the appropriate dimensions
	
	// Solve the velocity eat each waypoint for each dimension
	// Note: Waypoints are stored column wise, but we need to solve velocities across each row
	for(int i = 0; i < this->dimensions; i++)
	{
		Vector<DataType,Dynamic> displacement(this->numPoints-1);
		for(int j = 0; j < displacement.size(); j++) displacement[j] = dx[j][i];            // Get all waypoints for the ith row
		
		Vector<DataType,Dynamic> velocity = C*displacement;                                          // Compute all n velocities
		for(int j = 0; j < velocity.size(); j++) xdot[j][i] = velocity[j];                  // Store velocities along ith row
	}
	
	return xdot;
}
						 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the desired state for the given time                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicSpline::get_state(Vector<DataType,Dynamic> &pos,
			    Vector<DataType,Dynamic> &vel,
			    Vector<DataType,Dynamic> &acc,
			    const DataType     &time)
{
	if(pos.size() != vel.size()
	or vel.size() != acc.size())
	{
		cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
			  << "Input vectors are not the same length! "
			  << "The position vector had " << pos.size() << " elements, "
			  << "the velocity vector had " << vel.size() << " elements, and "
			  << "the acceleration vector had " << acc.size() << " elements.";
			  
		return false;
	}
	else if(pos.size() != this->dimensions)
	{
		cerr << "[ERROR] [CUBIC SPLINE] get_state() : "
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

#endif
