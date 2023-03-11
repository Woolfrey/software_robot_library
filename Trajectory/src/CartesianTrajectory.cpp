#include <CartesianTrajectory.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<Pose>  &waypoint,
                                         const std::vector<float> &time)
                                         :
                                         numPoints(waypoint.size())
{
	if(waypoint.size() != time.size())
	{
		throw "[ERROR] [CARTESIAN TRAJECTORY] Length of input arguments do not match! "
		    + "There were " + waypoint.size() + " waypoints and " + time.size() + " times.";
	}
	
	std::vector<Eigen::VectorXf>    translation;
	std::vector<Eigen::Quaternionf> orientation;
	
	for(int i = 0; i < this->numPoints; i++)
	{
		translation.push_back(waypoint[i].pos());
		orientation.push_back(waypoint[i].quat());
	}
	
	try this->translationTrajectory = CubicSpline(translation,time);
	catch(const char* error_message) throw error_message;
	
	try this->orientationTrajectory = CubicSpline(orientation,time);
	catch(const char* error_message) throw error_message;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(Pose                      &pose,
                                    Eigen::Matrix<double,6,1> &vel,
                                    Eigen::Matrix<double,6,1> &acc,
                                    const float               &time)
{
	Eigen::VectorXf position, velocity, acceleration;
	Eigen::Quaternionf quaternion;
	
	if(this->translationTrajectory.get_state(position,velocity,acceleration,time)
	{
		vel.head(3) = velocity;
		acc.head(3) = acceleration;
	}
	else	return false;
	
	if(this->orientationTrajectory.get_state(quaternion,velocity,acceleration,time)
	{
		vel.tail(3) = velocity;
		acc.tail(3) = acceleration;
	}
	else	return false;
	
	pose = Pose(position,quaternion);
	
	return true;
}
