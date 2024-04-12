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
		RigidBody(const std::string                 &name,
			     const DataType                    &mass,
		          const Eigen::Matrix<DataType,3,3> &inertia,
		          const Eigen::Vector<DataType,3>   &centerOfMass);
		      
		/**
		 * @return Returns the mass of this object.
		 */
		DataType mass() const { return this->_mass; }
		
		/**
		 * @return Returns the moment of inertia for this object.
		 */
		Eigen::Matrix<DataType,3,3> inertia() const { return this->_inertia; }
		
		/**
		 * @return Returns the time derivative of the moment of inertia.
		 */
		Eigen::Matrix<DataType,3,3> inertia_derivative() const { return this->_inertiaDerivative; }
		
		/**
		 * @return The pose of this object in some global frame.
		 */
		Pose<DataType> pose() const { return this->_pose; }
		
		/**
		 * @return The name of this object.
		 */
		std::string name() const { return this->_name; }
		
		/**
		 * @return The center of mass for this object in its local reference frame.
		 */
		Eigen::Vector<DataType,3> center_of_mass() const { return this->_centerOfMass; }    // Get the center of mass
		
		/**
		 * @return The linear and angular velocity of this object.
		 */
		Eigen::Vector<DataType,6> twist() const { return this->_twist; }
		
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
		void update_state(const Pose<DataType> &pose,
		                  const Eigen::Vector<DataType,6> &twist);
		
	protected:
	          
		DataType _mass = 0.0;                                                                     ///< How heavy the object is (kg)                                             
	
		Eigen::Matrix<DataType,3,3> _inertia = Eigen::Matrix<DataType,3,3>::Zero();               ///< Inertia in GLOBAL reference frame
		
		Eigen::Matrix<DataType,3,3> _inertiaDerivative = Eigen::Matrix<DataType,3,3>::Zero();     ///< Time derivative of the moment of inertia
		
		Eigen::Matrix<DataType,3,3> _localInertia = Eigen::Matrix<DataType,3,3>::Zero();          ///< Moment of inertia (kg*m^2) in LOCAL frame
				
		Eigen::Vector<DataType,3>   _centerOfMass = {0,0,0};                                      ///< Location for the center of mass in GLOBAL frame
		
		Eigen::Vector<DataType,3>   _localCenterOfMass = {0,0,0};                                 ///< Location for center of mass in LOCAL frame
		
		Eigen::Vector<DataType,6>   _twist = Vector<DataType,6>::Zero();                          ///< Linear and angular velocity of the object.

		Pose<DataType> _pose;                                                                     ///< Pose of the object in GLOBAL reference frame
		
		std::string _name = "unnamed";                                                            ///< Unique identifier for this object.
		
};                                                                                                  // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
RigidBody<DataType>::RigidBody(const std::string                 &name,
			                const DataType                    &mass,
                               const Eigen::Matrix<DataType,3,3> &inertia,
                               const Eigen::Vector<DataType,3>   &centerOfMass)
                               :
                               _name(name),
                               _mass(mass),
                               _localInertia(inertia),
                               _localCenterOfMass(centerOfMass)
{	
	if(mass < 0.0)
	{
		throw std::invalid_argument("[ERROR] [RIGID BODY] Constructor: "
		                            "Mass argument was " + std::to_string(mass) + " but cannot be negative.");
	}
	else if((inertia - inertia.transpose()).norm() > 1e-04)
	{
		throw std::invalid_argument("[ERROR] [RIGID BODY] Constructor: "
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
	this->_mass += other.mass();                                                                   // Add the masses together

	Eigen::Matrix<DataType,3,3> R = pose.rotation();                                               // Get the rotation matrix
	
	Eigen::Vector<DataType,3> t = pose.translation() + R*other.center_of_mass();                   // Transform the center of mass for the other link to THIS coordinate frame
	
	Eigen::Matrix<DataType,3,3> S; S <<   0 , -t(2),  t(1), 
	                                    t(2),    0 , -t(0),
	                                   -t(1),  t(0),    0;                                         // As a skew-symmetric matrix
	
	this->_localInertia += R*other.inertia()*R.transpose() - other.mass()*S*S;                     // From the parallel axis theorem
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Update the pose and velocity of this rigid body                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
void RigidBody<DataType>::update_state(const Pose<DataType> &pose,
                                       const Eigen::Vector<DataType,6> &twist)
{
	this->_pose  = pose;                                                                           // Update the pose of this object
	
	this->_centerOfMass = this->_pose*this->_localCenterOfMass;                                    // Point transformation to global frame
	
	Eigen::Matrix<DataType,3,3> R = pose.rotation();                                               // Get the rotation component as SE(3)
	
	this->_inertia = R*this->_localInertia*R.transpose();                                          // Rotate inertia to global reference frame
	
	this->_twist = twist;                                                                          // Update the linear & angular velocity
	
	Eigen::Vector<DataType,3> w = this->_twist.tail(3);                                            // Needed so we can do cross product
	
	for(int i = 0; i < 3; i++) this->_inertiaDerivative.col(i) = w.cross(this->_inertia.col(i));   // Perform cross product on every column
}
#endif
