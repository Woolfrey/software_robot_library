/**
 * @file   RigidBody.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class specifying dynamic properties of a solid object.
 */

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include <Pose.h>

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
          Eigen::Vector<DataType,3> center_of_mass() const { return this->_centerOfMass; }          // Get the center of mass
          
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
          
          Eigen::Vector<DataType,6>   _twist = Eigen::Vector<DataType,6>::Zero();                   ///< Linear and angular velocity of the object.

          Pose<DataType> _pose;                                                                     ///< Pose of the object in GLOBAL reference frame
          
          std::string _name = "unnamed";                                                            ///< Unique identifier for this object.
          
};                                                                                                  // Semicolon needed after a class declaration

using RigidBody_f = RigidBody<float>;
using RigidBody_d = RigidBody<double>;

#endif
