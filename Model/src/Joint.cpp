/**
 * @file   Joint.h
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the joint class.
 */

#include <Joint.h>

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
