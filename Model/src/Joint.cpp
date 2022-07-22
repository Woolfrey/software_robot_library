#include <Joint.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Joint::Joint(const Eigen::Isometry3f &origin,
             const Eigen::Vector3f &axisOfActuation,
             const float positionLimits[2],
             const float &velocityLimit,
             const float &torqueLimit,
             const bool &revolute):
             pose(origin),
             axis(axisOfActuation),
             pLim{positionLimits[0], positionLimits[1]},
             vLim(velocityLimit),
             tLim(torqueLimit),
             isRevolute(revolute)
{
	this->axis.normalize();                                                                    // Ensure unit norm
	
	if(this->pLim[0] > this->pLim[1])
	{
		std::cerr << "[ERROR] [JOINT] Constructor: "
			  << "Lower joint limits of " << this->pLim[0] << "is greater than "
			  << "upper joint limit of " << this->pLim[1] << ". "
			  << "Swapping their values to avoid problems..." << std::endl;
			  
		float temp = this->pLim[1];
		this->pLim[1] = this->pLim[0];
		this->pLim[0] = temp;
	}
	if(this->vLim < 0)
	{
		std::cerr << "[ERROR] [JOINT] Constructor: "
		          << "Velocity limit " << this->vLim << " is less than zero! "
                         << "Making the value positive to avoid problems..." << std::endl;
                         
		this->vLim *= -1;
	}
	if(this->vLim == 0)
	{
		std::cerr << "[ERROR] [JOINT] Constructor: "
		          << "Velocity limit is zero!" << std::endl;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the transform from the joint displacement                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Isometry3f Joint::get_pose(const float &pos)
{
	if(this->isRevolute) 	return this->pose*Eigen::AngleAxisf(pos, this->axis);
	else			return this->pose*Eigen::Translation3f(pos*this->axis);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the position limits from the joint                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Joint::get_position_limits(float &lower, float &upper)
{
	// Can't think of a good error check / message here ¯\_(ツ)_/¯
	lower = this->pLim[0];
	upper = this->pLim[1];
}