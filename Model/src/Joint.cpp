#include <Joint.h>
#include <Link.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Joint::Joint(const std::string     &name,
	     const std::string     &type,
	     const Eigen::Vector3f &axis,
	     const Pose            &offset,
	     const float            positionLimit[2],
	     const float           &speedLimit,
	     const float           &effortLimit,
	     const float           &damping,
	     const float           &friction)
	     :
	     _name(name),
	     _type(type),
	     _axis(axis.normalized()),                                                              // Ensure unit norm for good measure
	     _offset(offset),
	     _positionLimit{positionLimit[0],positionLimit[1]},
	     _speedLimit(speedLimit),
	     _effortLimit(effortLimit),
	     _damping(damping),
	     _friction(friction)
{
	// Check that the inputs are sound
	std::string message = "[ERROR] [JOINT] Constructor: ";
	
	if(positionLimit[0] >= positionLimit[1])
	{
		message += "Lower position limit " + std::to_string(positionLimit[0]) +
		           " is greater than upper position limit " + std::to_string(positionLimit[1]) +
		           " for joint " + name + ".";
		
		throw std::logic_error(message);
	}
	else if(speedLimit <= 0)
	{
		message += "Speed limit for " + name + " joint was " + std::to_string(speedLimit) +
		           " but it must be positive.";
		
		throw std::invalid_argument(message);
	}
	else if(effortLimit <= 0)
	{
		message += "Force/torque limit for " + name + " joint was " + std::to_string(effortLimit) + 
		           " but it must be positive.";
		
		throw std::invalid_argument(message);
	}
	else if(damping < 0)
	{
		message += "Damping for " + name + " joint was " + std::to_string(damping) +
		           " but it cannot be negative.";
		
		throw std::invalid_argument(message);
	}
	else if(friction < 0)
	{
		message += "Friction for " + name + " joint was " + std::to_string(friction) +
		           " but it cannot be negative.";
		                       
		throw std::invalid_argument(message);
	}
	
	// Make sure the type is correctly specified
	     if(this->_type == "revolute" or this->_type == "continuous") this->isRevolute = true;
	else if(this->_type == "prismatic")                               this->isRevolute = false;
	else if(this->_type != "fixed")
	{
		message += "Joint type was " + this->_type + " but expected 'revolute', "
		         + "'continuous', or 'prismatic'.";
		         
		throw std::invalid_argument(message);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Set the pointer to the parent link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Joint::set_parent_link(Link &link)
{
	this->_parentLink = &link;
	
	return true;
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the pointer to the child link                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Joint::set_child_link(Link &link)
{
		this->_childLink = &link;                                                             // Set pointer to child
		
		return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //             Update the pose of the joint with respect to the global frame of reference         //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Joint::update_state(const Pose &previousPose, const float &position)
{
	if(position <= this->_positionLimit[0])
	{
		std::cerr << "[FLAGRANT SYSTEM ERROR] [JOINT] update_state(): "
		          << "Position for the " << this->_name << " joint is below the lower limit ("
		          << position << " < " << this->_positionLimit[0] << ").\n";
		
		return false;
	}
	else if(position >= this->_positionLimit[1])
	{
		std::cerr << "[FLAGRANT SYSTEM ERROR] [JOINT] update_state(): "
		          << "Position for the " << this->_name << " joint is above the lower limit ("
		          << position << " > " << this->_positionLimit[1] << ").\n";
		
		return false;
	}
	else
	{
/*		this->_pose = previousPose * this->_offset;
		
		if(this->isRevolute)
		{
			this->_pose *= Pose(Eigen::Vector3f::Zero,
				            Eigen::Quaternionf(cos(0.5*position),
				                               sin(0.5*position)*this->_localAxis(0),
				                               sin(0.5*position)*this->_localAxis(1),
				                               sin(0.5*position)*this->_localAxis(2)));
		}
		else // prismatic
		{
			this->_pose *= Pose(position*this->_localAxis, Eigen::Quaternionf(1, 0, 0, 0));
		}*/
	
		return true;
	}
}

