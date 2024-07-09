/**
 * @file   Joint.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class that defines an actuated joint connecting two rigid bodies together.s
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <Eigen/Core>
#include <Pose.h>
#include <string>
#include <vector>

/**
 * A data structure for joint limits.
 */
template <typename DataType>
struct Limits
{
     DataType lower;
     DataType upper;
};                                                                                                  // Semicolon needed after declaring a data structure

template <class DataType>
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
          Joint(const std::string                 &name,
                const std::string                 &type,
                const Eigen::Vector<DataType,3>   &axis,
                const Limits<DataType>            &positionLimit)
          :
          Joint(name, type, axis, Pose<DataType>(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}
          
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
          Joint(const std::string                 &name,
                const std::string                 &type,
                const Eigen::Vector<DataType,3>   &axis,
                const Pose<DataType>              &origin,
                const Limits<DataType>            &positionLimit,
                const DataType                    &speedLimit,
                const DataType                    &effortLimit,
                const DataType                    &damping,
                const DataType                    &friction);
          
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
          Eigen::Vector<DataType,3> axis() const { return this->_axis; }
          
          /**
           * The relative transform due to the joint position.
           * @param position The joint position (radians or metres)
           * @return The local pose origin.
           */
          Pose<DataType> position_offset(const DataType &position);
          
          /**
           * @return Returns the pose of this joint relative to the parent link in a kinematic chain.
           */
          Pose<DataType> origin() const { return this->_origin; }
          
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
          void extend_origin(const Pose<DataType> &other)
          { 
               Pose<DataType> temp = other;
               this->_origin = temp*this->_origin;
          } 
          
          /**
           * Get the position limits for this joint.
           * @param lower Argument in which the lower limit will be stored.
           * @param upper Argument in which the upper limit will be stored.
           */
          Limits<DataType> position_limits() const { return this->_positionLimit; }
          
          /**
           * @return The speed limit for the specified joint.
           */
          DataType speed_limit() const { return this->_speedLimit; }
          
          /**
           * @return The viscous friction (Ns/m^2) of this joint.
           */
          DataType damping() const { return this->_damping; }
          
     private:
     
          bool _isRevolute = true;                                                                  ///< Used for logic purposes when computing kinematics. 
          
          bool _isFixed = false;                                                                    ///< Links connected by fixed joints are merged together.
          
          Eigen::Vector<DataType,3> _axis;                                                          ///< Axis of actuation in LOCAL frame

          DataType _damping;                                                                        ///< Viscous friction of the joint actuator.

          DataType _effortLimit;                                                                    ///< The maximum force/torque for the joint actuator.
          
          DataType _friction;                                                                       ///< Static friction of the joint actuator.
          
          DataType _speedLimit;                                                                     ///< The maximum velocity of the joint actuator.s

          Limits<DataType> _positionLimit;                                                          ///< Lower and upper limits on the joint position
                    
          Pose<DataType> _origin;                                                                   ///< Pose with respect to joint of parent link
           
          std::string _type = "unknown";                                                            ///< Joint type as a string
          
          std::string _name = "unnamed";                                                            ///< Unique identifier
          
};                                                                                                  // Semicolon needed after a class declaration

using Joint_f = Joint<float>;
using Joint_d = Joint<double>;

#endif
