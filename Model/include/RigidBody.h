/**
 * @file    RigidBody.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class for describing the kinematics & dynamics of a single, ridid body.
 * 
 * @details This class contains the kinematic & dynamic properties of a single rigid body.
 *          It can be used to compute the forces and torques due to its dynamic motion.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include "Pose.h"

#include <Eigen/Core>
#include <string>

namespace RobotLibrary { namespace Model {

/**
 * @brief A class representing a solid object in 3D space.
 */
class RigidBody
{
     public:   
          /**
           * @brief Empty constructor.
           */
          RigidBody() {}
          
          /**
           * @brief Full constructor for creating a RigidBody object.
           * @param name A unique identifier.
           * @param mass The weight of this object (kg).
           * @param inertia 3x3 matrix specifying the moment of inertia (kg*m^2)
           * @param centerOfMass The position of the center of mass relative to a reference frame on the object.
           */
          RigidBody(const std::string     &name,
                    const double          &mass,
                    const Eigen::Matrix3d &inertia,
                    const Eigen::Vector3d &centerOfMass);
                
          /**
           * @brief  Returns the mass of this object.
           */
          double mass() const { return this->_mass; }
          
          /**
           * @brief  Returns the moment of inertia for this object.
           */
          Eigen::Matrix3d inertia() const { return this->_inertia; }
          
          /**
           * @brief  Returns the time derivative of the moment of inertia.
           */
          Eigen::Matrix3d inertia_derivative() const { return this->_inertiaDerivative; }
          
          /**
           * @brief  The pose of this object in some global frame.
           */
          Pose pose() const { return this->_pose; }
          
          /**
           * @brief  The name of this object.
           */
          std::string name() const { return this->_name; }
          
          /**
           * @brief  The center of mass for this object in its local reference frame.
           */
          Eigen::Vector3d center_of_mass() const { return this->_centerOfMass; }                    // Get the center of mass
          
          /**
           * @brief  The linear and angular velocity of this object.
           */
          Eigen::Vector<double,6> twist() const { return this->_twist; }
          
          /**
           * @brief Combines the inertial properties of another rigid body with this one.
           * @param other The other rigid body object to be added to this one.
           * @param pose The pose of the other rigid body relative to this one.
           */
            void
            combine_inertia(const RigidBody &other,
                            const Pose      &pose);
                            
          /**
           * @brief Updates the kinematic properties for this object.
           * @param pose The pose of this object in a global reference frame.
           * @param twist The linear and angular velocity of this object.
           */              
          void
          update_state(const Pose &pose,
                       const Eigen::Vector<double,6> &twist);
          
     protected:
               
          double _mass = 0.0;                                                                       ///< How heavy the object is (kg)                                             
     
          Eigen::Matrix3d _inertia = Eigen::Matrix3d::Zero();                                       ///< Inertia in GLOBAL reference frame
          
          Eigen::Matrix3d _inertiaDerivative = Eigen::Matrix3d::Zero();                             ///< Time derivative of the moment of inertia
          
          Eigen::Matrix3d _localInertia = Eigen::Matrix3d::Zero();                                  ///< Moment of inertia (kg*m^2) in LOCAL frame
                    
          Eigen::Vector3d _centerOfMass = {0,0,0};                                                  ///< Location for the center of mass in GLOBAL frame
          
          Eigen::Vector3d _localCenterOfMass = {0,0,0};                                             ///< Location for center of mass in LOCAL frame
          
          Eigen::Vector<double,6> _twist = Eigen::Vector<double,6>::Zero();                         ///< Linear and angular velocity of the object.

          Pose _pose;                                                                               ///< Pose of the object in GLOBAL reference frame
          
          std::string _name = "unnamed";                                                            ///< Unique identifier for this object.
          
};                                                                                                  // Semicolon needed after a class declaration

} }

#endif
