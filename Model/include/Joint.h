/**
 * @file   Joint.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class that defines an actuated joint connecting two rigid bodies together.s
 */

#ifndef JOINT_H_
#define JOINT_H_

#include "Pose.h"                                                                                   // This tells the compiler to search locally

#include <Eigen/Core>
#include <string>
#include <vector>

namespace RobotLibrary {

/**
 * A data structure for joint limits.
 * It could represent position, velocity, acceleration, or torque depending on the context.
 */
struct Limits
{
     double lower;                                                                                  ///< Upper limit
     double upper;                                                                                  ///< Lower limit
};                                                                                                  // Semicolon needed after declaring a data structure

/**
 * A class representing an actuated joint on a mechanism.
 */
class Joint
{
     public:
          /**
           * Minimum constructor which delegates to the full constructor.
           * @param name A unique identifier for this joint.
           * @param type Options are 'revolute', 'prismatic', or 'continuous'.
           * @param axis A unit vector defining the axis of actuation.
           * @param positionLimit A 2D array specifying the upper and lower limits for the joint.
           */
          Joint(const std::string             &name,
                const std::string             &type,
                const Eigen::Vector<double,3> &axis,
                const Limits                  &positionLimit)
          :
          Joint(name, type, axis, Pose(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}
          
          /**
           * Full constructor for a Joint object.
           * @param name A unique identifier for this joint.
           * @param type Options are 'revolute', 'prismatic', or 'continuous'.
           * @param axis A unit vector defining the axis of actuation.
           * @param origin The pose of the joint origin relative to the parent link.
           * @param positionLimit A data structure specifying the upper and lower limits for the joint.
           * @param speedLimit The maximum velocity that the joint actuator can move (rad/s or m/s).
           * @param effortLimit The maximum force or torque the joint actuator can apply.
           * @param damping Viscous friction for the joint (N*s/m^2 or Nm*s/rad^2).
           * @param friction The static friction of the joint.
           */
          Joint(const std::string             &name,
                const std::string             &type,
                const Eigen::Vector<double,3> &axis,
                const Pose                    &origin,
                const Limits                  &positionLimit,
                const double                  &speedLimit,
                const double                  &effortLimit,
                const double                  &damping,
                const double                  &friction);
          
          /**
           * @return Returns true if this joint is not actuated.
           */
          bool is_fixed() const { return this->_isFixed; }
          
          /**
           * @return Returns true if this is a translational joint.
           */
          bool is_prismatic() const { return not this->_isRevolute; }                               // Because I'm lazy
          
          /**
           * @return Returns true if this is a rotational joint.
           */
          bool is_revolute() const { return this->_isRevolute; }
          
          /**
           * @return Returns a unit vector for local axis of actuation.
           */
          Eigen::Vector<double,3> axis() const { return this->_axis; }
          
          /**
           * The relative transform due to the joint position.
           * @param position The joint position (radians or metres)
           * @return The local pose origin.
           */
          Pose position_offset(const double &position);
          
          /**
           * @return Returns the pose of this joint relative to the parent link in a kinematic chain.
           */
          Pose origin() const { return this->_origin; }
          
          /**
           * @return Returns the joint type as a string (prismatic, revolute, or fixed)
           */
          std::string type() const { return this->_type; }
          
          /**
           * @return Returns the name of this joint.
           */
          std::string name() const { return this->_name; }
          
          /**
           * Extends the pose of this joint relative to its parent link in a kinematic chain.
           * @param other The additional transformation before this one.
           */
          void extend_origin(const Pose &other)
          { 
               Pose temp = other;
               this->_origin = temp*this->_origin;
          } 
          
          /**
           * Get the position limits for this joint.
           * @param lower Argument in which the lower limit will be stored.
           * @param upper Argument in which the upper limit will be stored.
           */
          Limits position_limits() const { return this->_positionLimit; }
          
          /**
           * @return The speed limit for the specified joint.
           */
          double speed_limit() const { return this->_speedLimit; }
          
          /**
           * @return The effort (force or torque) limit for the joints.
           */
          double effort_limit() const { return this->_effortLimit; }
          
          /**
           * @return The viscous friction (Ns/m^2) of this joint.
           */
          double damping() const { return this->_damping; }
          
     private:
     
          bool _isRevolute = true;                                                                  ///< Used for logic purposes when computing kinematics. 
          
          bool _isFixed = false;                                                                    ///< Links connected by fixed joints are merged together.
          
          Eigen::Vector<double,3> _axis;                                                            ///< Axis of actuation in LOCAL frame

          double _damping;                                                                          ///< Viscous friction of the joint actuator.

          double _effortLimit;                                                                      ///< The maximum force/torque for the joint actuator.
          
          double _friction;                                                                         ///< Static friction of the joint actuator.
          
          double _speedLimit;                                                                       ///< The maximum velocity of the joint actuator.s

          Limits _positionLimit;                                                                    ///< Lower and upper limits on the joint position
                    
          Pose _origin;                                                                             ///< Pose with respect to joint of parent link
           
          std::string _type = "unknown";                                                            ///< Joint type as a string
          
          std::string _name = "unnamed";                                                            ///< Unique identifier
          
};                                                                                                  // Semicolon needed after a class declaration

}
#endif
