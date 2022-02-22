    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <Eigen/Geometry>                                                                          // Eigen::Isometry3f, Eigen::Vector3f

class Joint
{
	public:
		Joint() {}                                                                         // Empty constructor
		
		Joint(const Eigen::Isometry3f &origin,
                     const Eigen::Vector3f &axisOfActuation,
                     const float positionLimits[2],
                     const float &velocityLimit,
                     const float &torqueLimit,
                     const bool &revolute);
		
		// Get Functions
		bool is_revolute() const {return this->isRevolute;}                                // Returns true if revolute
		bool is_prismatic() const {return !this->isRevolute;}                              // Returns true if NOT revolute
		Eigen::Isometry3f get_pose(const float &pos);                                      // Get the pose from the displacement of this joint
		Eigen::Vector3f get_axis() const {return this->axis;}
		float get_velocity_limit() const {return this->vLim;}                              // Get the speed limit
		void get_position_limits(float &lower, float &upper);                              // Get the position limits
		
	protected:
		bool isRevolute = true;
		Eigen::Isometry3f pose;                                                            // Pose relative to some origin
		Eigen::Vector3f axis = {0,0,1};                                                    // Axis of actuation
		float damping = 1.0;                                                               // Viscous friction for the joint
		float pLim[2] = {-M_PI, M_PI};                                                     // Position limits
		float vLim = 10;                                                                   // Speed limits
		float tLim = 10;                                                                   // Torque / force limits

};                                                                                                 // Semicolon needed after a class declaration

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
		std::cerr << "[ERROR] [JOINT] Constructor : Lower joint limits of " << this->pLim[0]
			<< " is greater than upper joint limit of " << this->pLim[1] << ". Swapping their values to avoid problems..." << std::endl;
		float temp = this->pLim[1];
		this->pLim[1] = this->pLim[0];
		this->pLim[0] = temp;
	}
	if(this->vLim < 0)
	{
		std::cerr << "[ERROR] [JOINT] Constructor : Velocity limit " << this->vLim << " is less than zero!"
			<< " Making the value positive to avoid problems..." << std::endl;
		this->vLim *= -1;
	}
	if(this->vLim == 0) std::cerr << "[ERROR] [JOINT] Constructor : Velocity limit is zero!" << std::endl;
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

#endif

