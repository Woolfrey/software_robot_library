#include <RigidBody.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const Eigen::Isometry3f &origin,
                     const Eigen::Vector3f &centreOfMass,
                     const float &_mass,
                     const Eigen::VectorXf &momentOfInertia):
                     pose(origin),
                     com(centreOfMass),
                     mass(_mass)
{
	if(momentOfInertia.size() != 6)
	{
		std::cerr << "[ERROR] [RIGIDBODY] Constructor: "
			  << "Expected 6 elements for the momentOfInertia argument. "
			  << " Your input had " << momentOfInertia.size() << " elements." << std::endl;
	}
	else
	{
		this->inertia << momentOfInertia(0), momentOfInertia(1), momentOfInertia(2),
				  momentOfInertia(1), momentOfInertia(3), momentOfInertia(4),
				  momentOfInertia(2), momentOfInertia(4), momentOfInertia(5);
	}
				
	if(this->mass < 0)
	{
		std::cerr << "[WARNING] [RIGIDBODY] Constructor: "
                         << "Mass cannot negative. "
                         << "Your input was " << this->mass << ". "
			  << "Ensuring the value is positive to avoid problems..." << std::endl;
			  
		this->mass *= -1;
	}
}
