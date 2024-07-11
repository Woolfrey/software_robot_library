/**
 * @file   RigidBody.cpp
 * @author Jon Woolfrey
 * @date   July 2023
 * @brief  Source files for the RigidBody class.
 */
 
#include <RigidBody.h>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
RigidBody::RigidBody(const std::string     &name,
                     const double          &mass,
                     const Eigen::Matrix3d &inertia,
                     const Eigen::Vector3d &centerOfMass)
                     : _name(name),
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
 //                     Update the pose and velocity of this rigid body                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
void
RigidBody::update_state(const Pose &pose,
                        const Eigen::Vector<double,6> &twist)
{
     this->_pose = pose;                                                                            // Update the pose of this object
     
     this->_centerOfMass = this->_pose*this->_localCenterOfMass;                                    // Point transformation to global frame
     
     Eigen::Matrix3d R = pose.rotation();                                                           // Get the rotation component as SE(3)
     
     this->_inertia = R*this->_localInertia*R.transpose();                                          // Rotate inertia to global reference frame
     
     this->_twist = twist;                                                                          // Update the linear & angular velocity
     
     Eigen::Vector3d w = this->_twist.tail(3);                                                      // Needed so we can do cross product
     
     for(int i = 0; i < 3; i++) this->_inertiaDerivative.col(i) = w.cross(this->_inertia.col(i));   // Perform cross product on every column
     
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                 Combine the inertia of another rigid body object with this one                //  
///////////////////////////////////////////////////////////////////////////////////////////////////
void
RigidBody::combine_inertia(const RigidBody &other,
                           const Pose      &pose)
{
     this->_mass += other.mass();                                                                   // Add the masses together

     Eigen::Matrix3d R = pose.rotation();                                                           // Get the rotation matrix
     
     Eigen::Vector3d t = pose.translation() + R*other.center_of_mass();                             // Transform the center of mass for the other link to THIS coordinate frame
     
     Eigen::Matrix3d S; S <<   0 , -t(2),  t(1), 
                             t(2),    0 , -t(0),
                            -t(1),  t(0),    0;                                                     // As a skew-symmetric matrix
     
     this->_localInertia += R*other.inertia()*R.transpose() - other.mass()*S*S;                     // From the parallel axis theorem
}
