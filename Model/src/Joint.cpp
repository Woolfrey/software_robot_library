#include <Joint.h>
#include <Link.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Joint::Joint(const std::string     &name,
	     const std::string     &type,
	     const Eigen::Vector3f &axis,
	     const Pose            &origin,
	     const float            positionLimit[2],
	     const float           &velocityLimit,
	     const float           &forceLimit,
	     const float           &damping,
	     const float           &friction)
	     :
	     _name(name),
	     _type(type),
	     _axis(axis.normalized()),                                                              // Ensure unit norm for good measure
	     _origin(origin),
	     _positionLimit{positionLimit[0],positionLimit[1]},
	     _velocityLimit(velocityLimit),
	     _forceLimit(forceLimit),
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
	else if(velocityLimit <= 0)
	{
		message += "Velocity limit for " + name + " joint was " + std::to_string(velocityLimit) +
		           " but it must be positive.";
		
		throw std::invalid_argument(message);
	}
	else if(forceLimit <= 0)
	{
		message += "Force/torque limit for " + name + " joint was " + std::to_string(forceLimit) + 
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
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Set the pointer to the parent link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Joint::set_parent_link(Link* link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [JOINT] set_parent_link(): "
		          << "Reference is a null pointer.\n";
		
		return false;
	}
	else
	{
		this->parentLink = link;
		
		return true;
	}
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Set the pointer to the child link                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Joint::set_child_link(Link *link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [JOINT] set_child_link(): "
		          << "Reference is a null pointer.\n";
		
		return false;
	}
	else
	{
		this->childLink = link;                                                             // Set pointer to child
		
		link->attach_joint(this);                                                           // Add pointer to this object in the link
		
		return true;
	}
}
