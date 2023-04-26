#include <Link.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the preceding link                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::attach_joint(Joint *joint)
{
	if(joint == nullptr)
	{
		std::cerr << "[ERROR] [LINK] attach_joint(): "
		          << "Input is a null pointer.\n";

		return false;
	}
	else
	{
		this->attachedJoint = joint;
		
		return true;
	}
}

