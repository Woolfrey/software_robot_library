    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //           A class for defining trajectories in SE(3), i.e. position & orientation              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <CubicSpline.h>                                                                            // Custom trajectory class
#include <Pose.h>

template <typename DataType>
struct CartesianState
{
	Pose<DataType>            pose;
	Eigen::Vector<DataType,6> twist;
	Eigen::Vector<DataType,6> acceleration;
};                                                                                                  // Semicolon needed after class declaration

template <class DataType>
class CartesianTrajectory
{
	public:
		/**
		 * Constructor.
		 * @param poses The poses that define the waypoints on the trajectory.
		 * @param times The time at which to pass through each pose.
		 */
		CartesianTrajectory(const std::vector<Pose<DataType>>  &poses,
		                    const std::vector<DataType>        &times);
		
		/**
		 * Query the pose for the given input time.
		 * @param time The point at which to evaluate the pose.
		 * @return The pose (position & orientation) as RobotLibrary object.
		 */
		Pose<DataType> query_pose(const DataType &time)
		{
			return query_state(time).pose;                                              // Too easy lol ᕙ(▀̿̿ĺ̯̿̿▀̿ ̿) ᕗ
		}
		
		/**
		 * Query the state (pose, velocity, acceleration) for the given time.
		 * @param time The point at which to evaluate the state.
		 * @return The state as a CartesianState data structure.
		 */
		CartesianState<DataType> query_state(const DataType &time);
		
	private:
	
		unsigned int numPoints;

		CubicSpline<DataType> trajectory;                                                   // This is the underlying trajectory object      
		
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
CartesianTrajectory<DataType>::CartesianTrajectory(const std::vector<Pose<DataType>> &poses,
                                                   const std::vector<DataType>       &times)
                                                   :
                                                   numPoints(waypoint.size())
{
	if(waypoint.size() != time.size())
	{
		throw std::invalid_argument("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
                                            "Length of input arguments do not match. "
                                            "There were " + std::to_string(waypoint.size()) + " waypoints "
                                            "and " + std::to_string(time.size()) + " times.");
	}

	std::vector<Eigen::Vector<DataType,6>> point;                                               // 3 for translation and 3 for orientation
	
	point.resize(waypoint.size());                                                              // Resize object for the number of waypoints
	
	for(int i = 0; i < waypoint.size(); i++)
	{
		point[i].head(3) = waypoint[i].position();                                          // Grab the translation component directly
		
		Eigen::Vector<DataType,3> epsilon = waypoint[i].quaternion().vec();                 // Get the vector part of the quaternion
		
		DataType norm = epsilon.norm();                                                     // Magnitude of the vector                                     
		
		DataType angle = 2*asin(norm);                                                      // Extract angle embedded in quaternion
		
		if(abs(angle) < 1e-04) point[i].tail(3) = Eigen::Vector<DataType,3>::Zero();        // Very small; zero rotation
		else                   point[i].tail(3) = angle*(epsilon/norm);                     // Angle*axis
	}
	
	try
	{
		this->trajectory = CubicSpline<DataType>(point,time);
	}
	catch(const exception &exception)
	{
		std::cerr << exception.what() << std::endl;                                         // Use std::endl in conjunction with std::cerr?
		
		throw std::runtime_error("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		                         "Could not generate a trajectory.");
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
CartesianState<DataType> CartesianTrajectory<DataType>::query_state(const DataType &time)
{
	auto &[position, velocity, acceleration] = this->_trajectory.query_state(time);             // Get the state for the given time
	
	// Convert the position vector to a pose object
	
	DataType angle = position.tail(3).norm();                                                   // Norm of the angle*axis component
	
	Pose pose;                                                                                  // We need to compute this
	
	if(abs(angle) < 1e-04) pose = Pose(position.head(3), Eigen::Quaternion<DataType>(1,0,0,0)); // Assume zero rotation
	else
	{
		Eigen::Vector<DataType,3> axis = position.tail(3).normalized();
		
		pose = Pose<DataType>(position.head(3), Eigen::Quaternion<DataType>(cos(0.5*angle),
		                                                                    sin(0.5*angle)*axis(0),
		                                                                    sin(0.5*angle)*axis(1),
		                                                                    sin(0.5*angle)*axis(2)));
	}
	
	CartesianState<DataType> returnValue = {pose, velocity, acceleration};                      // Put them together in data structure
	
	return returnValue;
}

#endif
