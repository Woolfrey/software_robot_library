    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A minimum acceleration trajectory in Cartesian space                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <CubicSpline.h>                                                                            // Custom trajectory class
#include <Pose.h>

using namespace Eigen;                                                                              // Eigen::Vector
using namespace std;                                                                                // std::vector, std::logic_error

template <class DataType>
class CartesianTrajectory
{
	public:
	
		// Constructors
		CartesianTrajectory();
		
		CartesianTrajectory(const vector<Pose<DataType>>  &waypoint,
		                    const vector<DataType>        &time);
		             
		// Methods       
		bool get_state(Pose<DataType>      &pose,
			       Vector<DataType,6>  &vel,
			       Vector<DataType,6>  &acc,
			       const DataType      &time);
	
	private:
	
		unsigned int numPoints;

		CubicSpline<DataType> trajectory;                                                   // This is the underlying trajectory object      
		
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
CartesianTrajectory<DataType>::CartesianTrajectory(const vector<Pose<DataType>> &waypoint,
                                                   const vector<DataType>       &time)
                                                   :
                                                   numPoints(waypoint.size())
{
	if(waypoint.size() != time.size())
	{
		throw logic_error("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
                                  "Length of input arguments do not match. "
                                  "There were " + to_string(waypoint.size()) + " waypoints "
                                  "and " + to_string(time.size()) + " times.");
	}

	vector<Vector<DataType,6>> point;                                                           // 3 for translation and 3 for orientation
	
	point.resize(waypoint.size());                                                              // Resize object for the number of waypoints
	
	for(int i = 0; i < waypoint.size(); i++)
	{
		point[i].head(3) = waypoint[i].position();                                          // Grab the translation component directly
		
		Vector<DataType,3> epsilon = waypoint[i].quaternion().vec();                        // Get the vector part of the quaternion
		
		DataType norm = epsilon.norm();                                                     // Magnitude of the vector                                     
		
		DataType angle = 2*asin(norm);                                                      // Extract angle embedded in quaternion
		
		if(abs(angle) < 1e-04) point[i].tail(3) = Vector<DataType,3>::Zero();               // Very small; zero rotation
		else                   point[i].tail(3) = angle*(epsilon/norm);                     // Angle*axis
	}
	
	try
	{
		this->trajectory = CubicSpline<DataType>(point,time);
	}
	catch(const exception &exception)
	{
		cout << exception.what() << endl;
		
		throw runtime_error("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		                    "Could not generate a trajectory.");
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool CartesianTrajectory<DataType>::get_state(Pose<DataType>      &pose,
                                              Vector<DataType,6>  &vel,
                                              Vector<DataType,6>  &acc,
                                              const DataType      &time)
{
	Vector<DataType,6> pos;                                                                     // Temporary placeholder
	
	if(this->trajectory.get_state(pos,vel,acc,time))
	{
		DataType angle = pos.tail(3).norm();                                                // Last 3 elements are angle x axis
		
		if(abs(angle) < 1e-04) pose = (pos.head(3), Quaternion<DataType>(1,0,0,0));         // Assume zero rotation
		else
		{
			Vector<DataType,3> axis = pos.tail(3).normalized();
			
			pose = Pose<DataType>(pos.head(3), Quaternion<DataType>(cos(0.5*angle),
                                                                                sin(0.5*angle)*axis(0),
			                                                        sin(0.5*angle)*axis(1),
			                                                        sin(0.5*angle)*axis(2)));
		}
		
		return true;
	}
	else
	{
		cerr << "[ERROR] [CARTESIAN TRAJECTORY] get_state(): "
		     << "Unable to get the state for some reason.\n";

		return false;
	}
}
#endif
