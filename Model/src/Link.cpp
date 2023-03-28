#include <Link.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Link::Link(const Joint     &_joint,
           const RigidBody &rigidBody)
           :
           joint(_joint),
           RigidBody(rigidBody)
{
	// Worker bees can leave
	// Even drones can fly away
	// The Queen is their slave
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Set the preceding link                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::set_preceding_link(Link &link)
{
	try
	{
		this->_precedingLink = std::make_shared<Link>(std::move(link));
		
		return true;
	}
	catch(const std::exception &exception)
	{
		std::cout << exception.what() << std::endl;
		
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add this link to the list of proceeding links                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Link::add_proceeding_link(Link &link)
{
	try
	{
		this->_proceedingLink.push_back(std::make_shared<Link>(std::move(link)));
		return true;
	}
	catch(const std::exception &exception)
	{
		std::cout << exception.what() << std::endl;
		return false;
	}
}
