#include <Joint.h>
#include <Link.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Assign pointer to the joint attached to this link                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::attach_joint(Joint &joint)
{
		std::cout << "[INFO] [LINK] attach_joint(): Attaching the " << joint.name()
		          << " joint to the " << this->_name << " link.\n";
		          
		this->_joint = &joint;
		
		std::cout << "    > After assignment: " << this->_joint->name() << " <-- " << this->_name << std::endl;
		
		return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Assign pointer to the previous link in the kinematic chain                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::set_previous_link(Link &link)
{
		this->previousLink = &link;
		
		return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Add a pointer to a proceeding link in the kinematic chain                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::add_next_link(Link &link)
{
	this->nextLinks.push_back(&link);
}
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::merge(Link *otherLink)
{
	if(otherLink == nullptr)
	{
		std::cerr << "[ERROR] [LINK] merge(): Input is a null pointer.\n";
	
		return false;
	}
	else
	{
		this->_mass += otherLink->mass();                                                   // Add the masses together
	
		Pose T = otherLink->joint()->offset();                                              // Coordinate frame of the other joint/link relative to this one
		
		Eigen::Matrix3f R = T.rotation();                                                   // Get the rotation matrix
		
		Eigen::Vector3f t = T.position() + R*otherLink->com();                              // Transform the center of mass for the other link to THIS coordinate frame
		
		Eigen::Matrix3f S; S <<    0 , -t(2),  t(1), 
		                         t(2),    0 , -t(0),
		                        -t(1),  t(0),    0;                                         // As a skew-symmetric matrix
		
		this->_inertia += R*otherLink->inertia()*R.transpose() - otherLink->mass()*S*S;     // From the parallel axis theorem
		
		return true;
	}
}
