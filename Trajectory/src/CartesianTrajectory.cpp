#include <CartesianTrajectory.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<Pose>  &waypoint,
                                         const std::vector<float> &time)
                                         :
                                         numPoints(waypoint.size())
{
	std::string errorMessage = "[ERROR] [CARTESIAN TRAJECTORY] Constructor: ";

	if(waypoint.size() != time.size())
	{
		errorMessage += "Length of input arguments do not match. There were "
		              + std::to_string(waypoint.size()) + " waypoints and " + std::to_string(time.size()) + " times.\n";

		throw std::logic_error(errorMessage);
	}

	std::vector<Eigen::Matrix<float,6,1>> point;                                                // 3 for translation and 3 for orientation
	
	point.resize(waypoint.size());                                                              // Resize object for the number of waypoints

	for(int i = 0; i < waypoint.size(); i++)
	{
		point[i].head(3) = waypoint[i].position();                                          // Grab the translation component directly
		
		Eigen::Vector3f epsilon = waypoint[i].quaternion().vec();                           // Get the vector part of the quaternion
		
		float norm = epsilon.norm();                                                        // Magnitude of the vector                                               
		
		float angle = 2*asin(norm);                                                         // Extract angle embedded in quaternion
		
		if(abs(angle) < 1e-04) point[i].tail(3) = Eigen::Vector3f::Zero();
		else                   point[i].tail(3) = angle*(epsilon/norm);                     // Angle*axis
	}
	
	try
	{
		this->trajectory = CubicSpline(point,time);
	}
	catch(const std::exception &exception)
	{
		std::cout << exception.what() << std::endl;
		
		errorMessage += "Could not generate a trajectory.\n";

		throw std::runtime_error(errorMessage);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(Pose                      &pose,
                                    Eigen::Matrix<float,6,1>  &vel,
                                    Eigen::Matrix<float,6,1>  &acc,
                                    const float               &time)
{
	Eigen::VectorXf x(6), xdot(6), xddot(6);                                                    // Temporary placeholder

	if(this->trajectory.get_state(x,xdot,xddot,time))
	{
        	vel = xdot;                                                                         // Twist vector
        	
        	acc = xddot;                                                                        // Acceleration vector
        	
        	Eigen::Vector3f position = x.head(3);                                               // Position / translation component
        	
		Eigen::Vector3f angleAxis = x.tail(3);                                              // Orientation component

		float angle = angleAxis.norm();                                                     // Get the angle component

		if(abs(angle) <= 1e-04)
		{
			pose = Pose(position,Eigen::Quaternionf(1,0,0,0));
		}
		else
		{
			Eigen::Vector3f axis = angleAxis.normalized();

			pose = Pose(position,Eigen::Quaternionf(cos(0.5*angle),
			                                        sin(0.5*angle)*axis(0),
			                                        sin(0.5*angle)*axis(1),
			                                        sin(0.5*angle)*axis(2)));
		}

		return true;
	}
	else
	{
		std::cerr << "[ERROR] [CARTESIAN TRAJECTORY] get_state(): "
		          << "Unable to get the state for some reason.\n";

		return false;
	}
}
