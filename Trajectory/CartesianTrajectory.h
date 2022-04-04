    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A minimum acceleration trajectory in Cartesian space                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <CubicSpline.h>                                                                           // Custom trajectory class
#include <Eigen/Geometry>                                                                          // Eigen::Isometry3f, Eigen::AngleAxisf
#include <vector>                                                                                  // std::vector

class CartesianTrajectory
{
	public:
	
		CartesianTrajectory();
		
		CartesianTrajectory(const std::vector<Eigen::Isometry3f> &waypoint,
							const std::vector<float> &time);
							
		bool get_state(Eigen::Isometry3f &pose,
					   Eigen::VectorXf &vel,
					   Eigen::VectorXf &acc,
					   const float &time);
	
	private:
	
		bool isValid = false;                                                                      // Won't do calcs if this is false
		CubicSpline translationTrajectory;                                                         // Translation component
		CubicSpline orientationTrajectory;                                                         // Orientation component
		unsigned int n;                                                                            // Number of waypoints
		
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<Eigen::Isometry3f> &waypoint,
										 const std::vector<float> &time):
										 n(waypoint.size())
{
	// Check that the vectors are of equal length
	if(time.size() != this->n)
	{
		std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
				  << "Input arguments are not of equal length. "
				  << "There were " << waypoint.size() << " waypoints "
				  << "and " << time.size() << " times." << std::endl;
	}
	else
	{
		// Check that times are in increasing order
		for(int i = 0; i < this->n-1; i++)
		{
			if(time[i] == time[i+1])
			{
				std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
						  << "Cannot move in zero time. "
						  << "Time " << i+1 << " was " << time[i] << " seconds and "
						  << "time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;
						  
				break;
			}
			else if(time[i] > time[i+1])
			{
				std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
						  << "Times are not in ascending order. "
						  << "Time " << i+1 << " was " << time[i] << " seconds and "
						  << "time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;
						  
				break;
			}
			else this->isValid = true;
		}
		
		// If no problems, instantiate the object
		if(this->isValid)
		{
			std::vector<Eigen::VectorXf> pos;
			std::vector<Eigen::AngleAxisf> rot;
			for(int i = 0; i < this->n; i++)
			{
				      pos.push_back( waypoint[i].translation()                 );                  // Add all translation vectors to array
					  rot.push_back( Eigen::AngleAxisf(waypoint[i].rotation()) );                  // Add all rotation objects to array
			}
			
			this->translationTrajectory = CubicSpline(pos, time);                                  // Create spline for translation
			this->orientationTrajectory = CubicSpline(rot, time);                                  // Create spline for orientation

			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(Eigen::Isometry3f &pose,
									Eigen::VectorXf &vel,
									EIgen::VectorXf &acc,
									const float &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR][CARTESIANTRAJECTORY] get_state(): "
				  << "Something went wrong during the construction of this object. "
				  << "Could not obtain the desired state." << std::endl;
				  
		return false;
	}
	else if(vel.size() != 6 or accel.size() != 6)
	{
		std::cerr << "[ERROR][CARTESIANTRAJECTORY] get_state(): "
				  << "Expected a 6x1 vector for the velocity and acceleration but "
				  << "the velocity input had " << vel.size() << " elements and "
				  << "the acceleration input had " << acc.size() << " elements." << std::endl;
				  
		return false;
	}
	else
	{
		Eigen::VectorXf pos(3), linearVel(3), angularVel(3), linearAcc(3), angularAcc(3);
		Eigen::AngleAxisf rot;
		
		if( this->translationTrajectory.get_state(pos, linearVel,  linearAcc,  time)
		and this->orientationTrajectory.get_state(rot, angularVel, angularAcc, time)
		{
			pose.translation() = pos;
			pose.rotation() = rot;
			
			for(int i = 0; i < 3; i++)
			{
				vel[i]   = linearVel[i];
				vel[i+3] = angularVel[i];
				acc[i]   = linearAcc[i];
				acc[i+3] = angularAcc[i];
			}
			
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [CARTESIANTRAJECTORY] get_state(): "
					  << "Could not obtain the desired state." << std::endl;
					  
			return false;
		}
	}
	
}

#endif
