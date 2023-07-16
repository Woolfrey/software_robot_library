#include <Joint.h>
#include <Link.h>


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
	
		Pose T = otherLink->parent_joint()->offset();                                       // Coordinate frame of the other joint/link relative to this one
		
		Eigen::Matrix3f R = T.rotation();                                                   // Get the rotation matrix
		
		Eigen::Vector3f t = T.position() + R*otherLink->com();                              // Transform the center of mass for the other link to THIS coordinate frame
		
		Eigen::Matrix3f S; S <<    0 , -t(2),  t(1), 
		                         t(2),    0 , -t(0),
		                        -t(1),  t(0),    0;                                         // As a skew-symmetric matrix
		
		this->_inertia += R*otherLink->inertia()*R.transpose() - otherLink->mass()*S*S;     // From the parallel axis theorem
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Set the preceding joint in the kinematic chain                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::set_parent_joint(Joint *joint)
{
	if(joint == nullptr)
	{
		std::cerr << "[ERROR] [LINK] set_parent_joint(): Argument was a null pointer.\n";
		
		return false;
	}
	else
	{
		this->parentJoint = joint;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Add a proceeding joint                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::add_child_joint(Joint *joint)
{
	if(joint == nullptr)
	{
		std::cerr << "[ERROR] [LINK] add_child_joint(): Argument was a null pointer.\n";
	
		return false;
	}
	else
	{
		this->childJoint.push_back(joint);
		
		return true;
	}
}
