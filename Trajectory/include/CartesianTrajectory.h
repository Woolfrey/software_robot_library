    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //           A class for defining trajectories in SE(3), i.e. position & orientation              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <MultiTrapezoid.h>
#include <Pose.h>
#include <Spline.h>

/**
 * A data structure for returning state information from functions.
 */
template <typename DataType>
struct CartesianState
{
	Pose<DataType>            pose;                                                                // Does this really need an explanation?
	Eigen::Vector<DataType,6> twist;                                                               // Linear and angular velocity
	Eigen::Vector<DataType,6> acceleration;                                                        // Linear and angular acceleration
};                                                                                                  // Semicolon needed after declaration

template <class DataType, class TrajectoryType>
class CartesianTrajectory : public Waypoints<DataType,TrajectoryType>
{
	public:
		/**
		 * Constructor for a trajectory with 2 poses.
		 * @param startPose The initial position & orientation.
		 * @param endPose The final position & orientation.
		 * @param startTime The time at which to begin.
		 * @param endTime Do I really have to explain this?
		 */
		CartesianTrajectory(const Pose<DataType> &startPose,
		                    const Pose<DataType> &endPose,
		                    const DataType &startTime,
		                    const DataType &endTime)
		:
		CartesianTrajectory(std::vector<Pose<DataType>> {startPose, endPose},
		                    std::vector<DataType> {startTime, endTime}) {}
		/**
		 * Constructor for a trajectory with waypoints.
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
			return query_state(time).pose;                                                       // Too easy lol ᕙ(▀̿̿ĺ̯̿̿▀̿ ̿) ᕗ
		}
		
		/**
		 * Query the state (pose, velocity, acceleration) for the given time.
		 * @param time The point at which to evaluate the state.
		 * @return The state as a CartesianState data structure.
		 */
		CartesianState<DataType> query_state(const DataType &time);
		
	private:
	
		TrajectoryType _trajectory;                                                               // The underlying trajectory object
		
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType, class TrajectoryType>
CartesianTrajectory<DataType,TrajectoryType>::CartesianTrajectory(const std::vector<Pose<DataType>> &poses,
                                                                  const std::vector<DataType>       &times)
{
	if(poses.size() < 2)
	{
		throw std::invalid_argument("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
		                            "A minimum number of 2 poses is required to construct a "
		                            "trajectory but you provided " + std::to_string(poses.size()) + ".");
	}
	else if(poses.size() != times.size())
	{
		throw std::invalid_argument("[ERROR] [CARTESIAN TRAJECTORY] Constructor: "
                                      "Length of input arguments do not match. "
                                      "There were " + std::to_string(poses.size()) + " waypoints "
                                      "and " + std::to_string(times.size()) + " times.");
	}

	std::vector<Eigen::Vector<DataType,6>> points;                                                 // Convert SE(3) to 6x1 vectors
	
	for(auto pose : poses)
	{
          Eigen::Vector<DataType,6> point;                                                          // Temporary placeholder
		
          point.head(3) = pose.translation();                                                       // Assign the translation component
		
          Eigen::Vector<DataType,3> vec = pose.quaternion().vec();                                  // Vector component of quaternion
		
          DataType norm = vec.norm();                                                               // As it says on the label
		
          DataType angle = 2*asin(norm);                                                            // vec = (angle/2)*axis; asin = [-pi, pi]
		
		if(abs(angle) < 1e-04) point.tail(3) = Eigen::Vector<DataType,3>::Zero();                 // Trivially small; zero rotation
		else                   point.tail(3) = angle*(vec/norm);                                  // angle*axis
	
		points.push_back(point);                                                                  // Add to end
	}
	
	this->_trajectory = TrajectoryType(points,times);                                              // Construct the trajectory
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType, class TrajectoryType> inline
CartesianState<DataType> CartesianTrajectory<DataType, TrajectoryType>::query_state(const DataType &time)
{
	auto &[position, velocity, acceleration] = this->_trajectory.query_state(time);                // Get the state for the given time
	
	// Convert the position vector to a pose object
	
	DataType angle = position.tail(3).norm();                                                      // Norm of the angle*axis component
	
	Pose pose;                                                                                     // We need to compute this
	
	if(abs(angle) < 1e-04) pose = Pose(position.head(3), Eigen::Quaternion<DataType>(1,0,0,0));    // Assume zero rotation
	else
	{
		Eigen::Vector<DataType,3> axis = position.tail(3).normalized();                           // Ensure magnitude of 1 
		
		pose = Pose<DataType>(position.head(3), 
		                      Eigen::Quaternion<DataType>(cos(0.5*angle),
		                                                  sin(0.5*angle)*axis(0),
		                                                  sin(0.5*angle)*axis(1),
		                                                  sin(0.5*angle)*axis(2)));
	}
	
	CartesianState<DataType> returnValue = {pose, velocity, acceleration};                         // Put them together in data structure
	
	return returnValue;
}

#endif
