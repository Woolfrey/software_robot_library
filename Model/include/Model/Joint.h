/**
 * @file   Joint.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class that defines an actuated joint connecting two rigid bodies together.s
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <Eigen/Core>
#include <Model/Pose.h>
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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Joint<DataType>::Joint(const std::string               &name,
                       const std::string               &type,
                       const Eigen::Vector<DataType,3> &axis,
                       const Pose<DataType>            &origin,
                       const Limits<DataType>          &positionLimit,
                        const DataType                  &speedLimit,
                        const DataType                  &effortLimit,
                       const DataType                  &damping,
                       const DataType                  &friction)
                       :
                       _name(name),
                       _type(type),
                       _axis(axis.normalized()),                                                    // Ensure unit norm for good measure
                       _origin(origin),
                       _positionLimit(positionLimit),
                       _speedLimit(speedLimit),
                       _effortLimit(effortLimit),
                       _damping(damping),
                       _friction(friction)
{
     using namespace std; // std::to_string, std::logic_error, std::invalid_argument
     
     // Check that the inputs are sound
     if(positionLimit.lower >= positionLimit.upper)
     {
          throw logic_error("[ERROR] [JOINT] Constructor: "
                            "Lower position limit " + to_string(positionLimit.lower) + " is greater than "
                            "upper position limit " + to_string(positionLimit.upper) + " for joint " + this->_name + ".");
     }
     else if(speedLimit <= 0)
     {
          throw invalid_argument("[ERROR] [JOINT] Constructor: "
                                 "Speed limit for " + this->_name + " joint was " + to_string(speedLimit) +
                                 " but it must be positive.");
     }
     else if(effortLimit <= 0)
     {
          throw invalid_argument("[ERROR] [JOINT] Constructor: "
                                 "Force/torque limit for " + this->_name + " joint was " + to_string(effortLimit) + 
                                 " but it must be positive.");
     }
     else if(damping < 0)
     {
          throw invalid_argument("[ERROR] [JOINT] Constructor: "
                                 "Damping for " + this->_name + " joint was " + to_string(damping) +
                                 " but it must be positive.");
     }
     else if(friction < 0)
     {
          throw invalid_argument("[ERROR] [JOINT] Constructor: "
                                 "Friction for " + this->_name + " joint was " + to_string(friction) +
                                 " but it cannot be negative.");
     }
     
     // Make sure the type is correctly specified
          if(this->_type == "revolute" or this->_type == "continuous") this->_isRevolute = true;
     else if(this->_type == "prismatic")                               this->_isRevolute = false;
     else if(this->_type == "fixed")                                   this->_isFixed    = true;
     else
     {
          throw invalid_argument("[ERROR] [JOINT] Constructor: "
                                 "Joint type was " + this->_type + " "
                                 "but expected 'revolute', " + "'continuous', or 'prismatic'.");
     }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Get the local transform due to the joint position                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Joint<DataType>::position_offset(const DataType &position)
{
     // Make sure the values are within the limits
     if(position > this->_positionLimit.upper)
     {
          throw std::runtime_error("[ERROR] [JOINT] position_offset(): "
                                   "Position for the " + this->_name + " joint exceeds upper limit ("
                                   + std::to_string(position) + " > "
                                   + std::to_string(this->_positionLimit.upper) + ").");
     }
     else if(position < this->_positionLimit.lower)
     {
          throw std::runtime_error("[ERROR] [JOINT] position_offset(): "
                                   "Position for the " + this->_name + " joint exceeds lower limit ("
                                   + std::to_string(position) + " < "
                                   + std::to_string(this->_positionLimit.lower) + ").");
     }
     
     // Compute the transform based on the type of joint
     if(this->_isRevolute)
     {
          return Pose<DataType>(Eigen::Vector<DataType,3>::Zero(),
                                Eigen::Quaternion<DataType>(cos(0.5*position),
                                                      sin(0.5*position)*this->_axis(0),
                                                      sin(0.5*position)*this->_axis(1),
                                                      sin(0.5*position)*this->_axis(2)));
     }
     else     return Pose<DataType>(position*this->_axis, Eigen::Quaternion<DataType>(1, 0, 0, 0));
}

#endif
