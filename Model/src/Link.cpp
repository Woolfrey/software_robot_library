#include <Joint.h>
#include <Link.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Assign pointer to the joint attached to this link                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::attach_joint(Joint *joint)
{
	if(joint == nullptr)
	{
		std::cerr << "[ERROR] [LINK] attach_joint(): Input is a null pointer.\n";

		return false;
	}
	else
	{
		this->attachedJoint = joint;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Assign pointer to the previous link in the kinematic chain                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::set_previous_link(Link *link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [LINK] set_previous_link(): Input was a null pointer.\n";
		
		return false;
	}
	else
	{
		this->previousLink = link;
		
		return true;
	}
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Add a pointer to a proceeding link in the kinematic chain                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::add_next_link(Link *link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [LINK] add_next_link(): Input was a null pointer.\n";
		
		return false;
	}
	else
	{
		this->nextLinks.push_back(link);
		
		return true;
	}
}

  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::merge(Link *other)
{
	if(other == nullptr)
	{
		std::cerr << "[ERROR] [LINK] merge(): Input is a null pointer.\n";
	
		return false;
	}
	else
	{
		this->_mass += other->mass();                                                       // Add the masses together
	
		Pose T = other->attached_joint()->origin();                                         // Origin of the joint/link
		
		Eigen::Matrix3f R = T.rotation_matrix();                                            // Get the rotation matrix
		
		Eigen::Vector3f t = T.pos() + R*other->com();                                       // Translation to other c.o.m. in THIS link's frame
		
		Eigen::Matrix3f S; S <<    0 , -t(2),  t(1), 
		                         t(2),    0 , -t(0),
		                        -t(1),  t(0),    0;                                         // As a skew-symmetric matrix
		
		this->_inertia += R*other->inertia()*R.transpose()
		                - other->mass()*S*S;                                                // From the parallel axis theorem
		
		// Maybe add the transform here?
		
		return true;
	}
}

