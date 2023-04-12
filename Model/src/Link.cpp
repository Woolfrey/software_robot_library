#include <Link.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Link::Link(const Joint       &_joint,
           const RigidBody   &rigidBody,
           const std::string &name)
           :
           joint(_joint),
           RigidBody(rigidBody),
           _name(name)
{
	// Worker bees can leave
	// Even drones can fly away
	// The Queen is their slave
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the preceding link                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::set_preceding_link(const std::shared_ptr<Link> link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [LINK] set_preceding_link(): "
		          << "Input appears to be a null pointer.\n";
		
		return false;
	}
	else
	{
		this->_precedingLink = link;                                                        // Assign the previous link
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add this link to the list of proceeding links                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::add_proceeding_link(const std::shared_ptr<Link> link)
{
	if(link == nullptr)
	{
		std::cerr << "[ERROR] [LINK] set_proceeding_link(): "
		          << "Input appears to be a null pointer.\n";
		
		return false;
	}
	else
	{
		this->_proceedingLink.push_back(link);                                              // Add it to the list
		
		return true;
	}
}
