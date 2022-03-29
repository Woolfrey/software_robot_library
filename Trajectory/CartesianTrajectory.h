    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                      A minimum acceleration trajectory in Cartesian space                      //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
		int n;                                                                                     // Number of waypoints
		CubicSpline translationTrajectory, orientationTrajectory;
		
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
		}
		
		// If no problems, instantiate the object
		if(this->isValid)
		{
			std::vector<Eigen::VectorXf> pos;
			std::vector<Eigen::AngleAxisf> rot;
			for(int i = 0; i < this->n; i++)
			{
				      pos.push_back( waypoint[i].translation() );                                  // Add all translation vectors to array
					  rot.push_back( waypoint[i].rotation() );                                     // Add all rotation objects to array
			}
			
			this->translationTrajectory = CubicSpline(pos, time);
			this->orientationTrajectory = CubicSpline(rot, time);

			}
		}
	}
}
