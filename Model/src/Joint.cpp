#include <Joint.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Joint::Joint(const Pose &origin,
             const Eigen::Vector3f &axisOfActuation,
             const float positionLimits[2],
             const float &velocityLimit,
             const float &torqueLimit,
             const bool &revolute):
             _pose(origin),
             _axis(axisOfActuation),
             pLim{positionLimits[0], positionLimits[1]},
             vLim(velocityLimit),
             tLim(torqueLimit),
             isRevolute(revolute)
{
	this->_axis.normalize();                                                                    // Ensure unit norm
	
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
Pose Joint::pose(const float &jointPosition)
{
	// NOTE: NEED TO PROGRAM A CASE FOR "FIXED" JOINTS
	
	if(this->isRevolute)
	{
		return this->_pose * Pose(Eigen::Vector3f::Zero(),
		                          Eigen::Quaternionf(Eigen::AngleAxisf(jointPosition,this->_axis)));
	}
	else // this->isPrismatic
	{
		return this->_pose * Pose(this->_axis*jointPosition,Eigen::Quaternionf(1,0,0,0));
	}
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
