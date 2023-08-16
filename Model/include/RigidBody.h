    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                           A class representing a single solid object                           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Eigen/Core>
#include <string>

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;                                                                                // std::invalid_argument

template <class DataType>
class RigidBody
{
	public:   
		// Constructor
		
		RigidBody() {}
		
		RigidBody(const string               &name,
			  const DataType             &mass,
		          const Matrix<DataType,3,3> &inertia,
		          const Vector<DataType,3>   &centerOfMass);
		      
		// Methods  
		
		DataType mass()                const { return this->_mass; }
		
		Matrix<DataType,3,3> inertia() const { return this->_inertia; }
		
		Pose<DataType> pose()          const { return this->_pose; }
		
		string name()                  const { return this->_name; }
		
		Vector<DataType,3> com()       const { return this->_centerOfMass; }                // Get the center of mass
		
		Vector<DataType,6> twist()     const { return this->_twist; }
		
		void combine_inertia(const RigidBody<DataType> &other,
		                     const Pose<DataType>      &pose);
		                     
		void update_state(const Pose<DataType> &pose, const Vector<DataType,6> &twist)
		{
			this->_pose  = pose;
			this->_twist = twist;
		}                  
		
	protected:
	          
		DataType _mass = 0.0;                                               
		
		Matrix<DataType,3,3> _inertia = Matrix<DataType,3,3>::Zero();
		
		Pose<DataType> _pose;
		
		string _name = "unnamed";
		
		Vector<DataType,3> _centerOfMass = {0,0,0};
		
		Vector<DataType,6> _twist = Vector<DataType,6>::Zero();
		
};                                                                                                  // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
RigidBody<DataType>::RigidBody(const string               &name,
			       const DataType             &mass,
                               const Matrix<DataType,3,3> &inertia,
                               const Vector<DataType,3>   &centerOfMass)
                               :
                               _name(name),
                               _mass(mass),
                               _inertia(inertia),
                               _centerOfMass(centerOfMass)
{	
	if(mass < 0.0)
	{
		throw invalid_argument("[ERROR] [RIGID BODY] Constructor: "
		                       "Mass argument was " + to_string(mass) + " but cannot be negative.");
	}
	else if((inertia - inertia.transpose()).norm() > 1e-04)
	{
		throw invalid_argument("[ERROR] [RIGID BODY] Constructor: "
		                       "Moment of inertia does not appear to be symmetric.");
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Combine the inertia of another rigid body object with this one                //  
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
void RigidBody<DataType>::combine_inertia(const RigidBody<DataType> &other,
                                          const Pose<DataType>      &pose)
{
	this->_mass += other.mass();                                                                // Add the masses together

	Matrix<DataType,3,3> R = pose.rotation();                                                   // Get the rotation matrix
	
	Vector<DataType,3> t = pose.position() + R*other.com();                                     // Transform the center of mass for the other link to THIS coordinate frame
	
	Matrix<DataType,3,3> S; S <<   0 , -t(2),  t(1), 
	                             t(2),    0 , -t(0),
	                            -t(1),  t(0),    0;                                             // As a skew-symmetric matrix
	
	this->_inertia += R*other.inertia()*R.transpose() - other.mass()*S*S;                       // From the parallel axis theorem
}

#endif
