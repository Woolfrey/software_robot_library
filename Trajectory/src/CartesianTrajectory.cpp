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
		              + waypoint.size() + " waypoints and " + time.size() + " times.\n";
		
		throw std::logic_error(errorMessage);
	}
	
	std::vector<Eigen::Matrix<double,6,1>> point;                                               // 3 for translation and 3 for orientation
	x.resize(waypoint.size());
	
	for(int i = 0; i < waypoint.size(); i++)
	{
		point[i].head(3) = waypoint[i].pos();                                               // Grab the translation component directly
		
		Eigen::Vector3f epsilon = waypoint[i].quat().vec();                                 // Get the vector part of the quaternion
		
		float norm = epsilon.norm();                                                        // Magnitude of the vector                                               
		
		float angle = 2*asin(norm);                                                         // Extract angle embedded in quaternion
		
		if(angle == 0) point[i].tail(3) = Eigen::Vector3f::Zero();
		else           point[i].tail(3) = angle*(epsilon/norm);                             // Angle*axis
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
                                    Eigen::Matrix<double,6,1> &vel,
                                    Eigen::Matrix<double,6,1> &acc,
                                    const float               &time)
{
	Eigen::VectorXf x(6);                                                                        // Temporary placeholder
	
	if(this->trajectory.get_state(x,vel,acc,time))
	{
		Eigen::Vector3f pos       = x.head(3);                                              // Translation component
		Eigen::Vector3f angleAxis = x.tail(3);                                              // Orientation component
		
		float angle = angleAxis.norm();                                                     // Get the angle component
		
		if(angle == 0)
		{
			pose = Pose(pos,Eigen::Quaternionf(1,0,0,0));
		}
		else
		{
			Eigen::Vector3 axis = angleAxis.normalized();
			
			pose = Pose(pos,Eigen::Quaternionf(cos(0.5*angle),
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
