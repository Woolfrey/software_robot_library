/**
 * @file   RigidBody.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class specifying dynamic properties of a solid object.
 */

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
		/**
		 * Empty constructor.
		 */
		RigidBody() {}
		
		/**
		 * Full constructor for creating a RigidBody object.
		 * @param name A unique identifier.
		 * @param mass The weight of this object (kg).
		 * @param inertia 3x3 matrix specifying the moment of inertia (kg*m^2)
		 * @param centerOfMass The position of the center of mass relative to a reference frame on the object.
		 */
		RigidBody(const string               &name,
			  const DataType             &mass,
		          const Matrix<DataType,3,3> &inertia,
		          const Vector<DataType,3>   &centerOfMass);
		      
		/**
		 * @return Returns the mass of this object.
		 */
		DataType mass()                const { return this->_mass; }
		
		/**
		 * @return Returns the moment of inertia for this object.
		 */
		Matrix<DataType,3,3> inertia() const { return this->_inertia; }
		
		/**
		 * @return The pose of this object in some global frame.
		 */
		Pose<DataType> pose()          const { return this->_pose; }
		
		/**
		 * @return The name of this object.
		 */
		string name()                  const { return this->_name; }
		
		/**
		 * @return The center of mass for this object in its local reference frame.
		 */
		Vector<DataType,3> com()       const { return this->_centerOfMass; }                // Get the center of mass
		
		/**
		 * @return The linear and angular velocity of this object.
		 */
		Vector<DataType,6> twist()     const { return this->_twist; }
		
		/**
		 * Combines the inertial properties of another rigid body with this one.
		 * @param other The other rigid body object to be added to this one.
		 * @param pose The pose of the other rigid body relative to this one.
		 */
		void combine_inertia(const RigidBody<DataType> &other,
		                     const Pose<DataType>      &pose);
		       
		/**
		 * Updates the kinematic properties for this object.
		 * @param pose The pose of this object in a global reference frame.
		 * @param twist The linear and angular velocity of this object.
		 */              
		void update_state(const Pose<DataType> &pose, const Vector<DataType,6> &twist)
		{
			this->_pose  = pose;
			this->_twist = twist;
		}                  
		
	protected:
	          
		DataType _mass = 0.0;                                                               ///< How heavy the object is (kg)                                             
		
		Matrix<DataType,3,3> _inertia = Matrix<DataType,3,3>::Zero();                       ///< Moment of inertia (kg*m^2)
		
		Pose<DataType> _pose;                                                               ///< Pose of the object in global reference frame
		
		string _name = "unnamed";                                                           ///< Unique identifier for this object.
		
		Vector<DataType,3> _centerOfMass = {0,0,0};                                         ///< Location for the center of mass in its LOCAL frame
		
		Vector<DataType,6> _twist = Vector<DataType,6>::Zero();                             ///< Linear and angular velocity of the object.
		
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
	
	Vector<DataType,3> t = pose.translation() + R*other.com();                                  // Transform the center of mass for the other link to THIS coordinate frame
	
	Matrix<DataType,3,3> S; S <<   0 , -t(2),  t(1), 
	                             t(2),    0 , -t(0),
	                            -t(1),  t(0),    0;                                             // As a skew-symmetric matrix
	
	this->_inertia += R*other.inertia()*R.transpose() - other.mass()*S*S;                       // From the parallel axis theorem
}

#endif
