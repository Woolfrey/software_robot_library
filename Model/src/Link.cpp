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
