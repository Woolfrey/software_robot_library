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
 //                             Combine another link with this one                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Link::combine_link(const Link &other)
{
	this->_mass += other.mass();                                                                // Add the mass
	
	// Transform the inertia to the origin of this link
	Pose pose = other.joint.origin();
	Eigen::Matrix3f R = pose.quat().toRotationMatrix();
	Eigen::Vector3f r = pose.pos();
	Eigen::Matrix3f S; S <<   0 , -r(2),  r(1),
	                        r(2),    0 , -r(0),
	                       -r(1),  r(0),    0 ;
	                       
	this->_inertia += R*other.inertia()*R.transpose() - other.mass()*S*S;                       // From the parallel axis theorem
}
