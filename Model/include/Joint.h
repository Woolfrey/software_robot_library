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

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::vector
using namespace std;                                                                                // std::string, std::runtime_error, std::invalid_argument

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
		Joint(const string               &name,
		      const string               &type,
		      const Vector<DataType,3>   &axis,
		      const DataType             positionLimit[2])
		:
		Joint(name, type, axis, Pose<DataType>(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}

		/**
		 * Full constructor for a Joint object.
		 * @param name A unique identifier for this joint.
		 * @param type Options are 'revolute', 'prismatic', or 'continuous'.
		 * @param axis A unit vector defining the axis of actuation.
		 * @param offset The pose of the joint origin relative to the parent link.
		 * @param positionLimit A 2D array specifying the upper and lower limits for the joint.
		 * @param speedLimit The maximum velocity that the joint actuator can move (rad/s or m/s).
		 * @param effortLimit The maximum force or torque the joint actuator can apply.
		 * @param damping Viscous friction for the joint (N*s/m^2 or Nm*s/rad^2)
		 * @param friction The static friction of the joint.
		 */
		Joint(const string               &name,
		      const string               &type,
		      const Vector<DataType,3>   &axis,
		      const Pose<DataType>       &offset,
		      const DataType              positionLimit[2],                                 // Can't use pointers & for arrays for some reason
		      const DataType             &speedLimit,
		      const DataType             &effortLimit,
		      const DataType             &damping,
		      const DataType             &friction);
		
		/**
		 * @return Returns true if this joint is not actuated.
		 */
		bool is_fixed() const { return this->isFixed; }
		
		/**
		 * @return Returns true if this is a translational joint.
		 */
		bool is_prismatic() const { return not this->isRevolute; }                          // Because I'm lazy
		
		/**
		 * @return Returns true if this is a rotational joint.
		 */
		bool is_revolute() const { return this->isRevolute; }
		
		/**
		 * @return Returns a unit vector for local axis of actuation.
		 */
		Vector<DataType,3> axis() const { return this->_axis; }
		
		/**
		 * The relative transform due to the joint position.
		 * @param position The joint position (radians or metres)
		 * @return The local pose offset.
		 */
		Pose<DataType> pose(const DataType &position);
		
		/**
		 * @return Returns the pose of this joint relative to the parent link in a kinematic chain.
		 */
		Pose<DataType> offset() const { return this->_offset; }
		
		/**
		 * @return Returns the joint type as a string (prismatic, revolute, or fixed)
		 */
		string type() const { return this->_type; }
		
		/**
		 * @return Returns the name of this joint.
		 */
		string name() const { return this->_name; }
		
		/**
		 * Extends the pose of this joint relative to its parent link in a kinematic chain.
		 * @param other The additional transformation before this one.
		 */
		void extend_offset(const Pose<DataType> &other)
		{ 
			Pose<DataType> temp = other;
			this->_offset = temp*this->_offset;
		} 
		
		/**
		 * Get the position limits for this joint.
		 * @param lower Argument in which the lower limit will be stored.
		 * @param upper Argument in which the upper limit will be stored.
		 */		
		void position_limits(DataType &lower, DataType &upper) { lower = this->_positionLimit[0];
		                                                         upper = this->_positionLimit[1]; }
		
	private:
	
		bool isRevolute = true;                                                             ///< Used for logic purposes when computing kinematics. 
		
		bool isFixed = false;                                                               ///< Links connected by fixed joints are merged together.
		
		Vector<DataType,3> _axis;                                                           ///< Axis of actuation in LOCAL frame

		DataType _positionLimit[2];                                                         ///< Lower and upper limits on the joint position
		
		DataType _speedLimit;                                                               ///< The maximum velocity of the joint actuator.s
		
		DataType _effortLimit;                                                              ///< The maximum force/torque for the joint actuator.
		
		DataType _damping;                                                                  ///< Viscous friction of the joint actuator.
		
		DataType _friction;                                                                 ///< Static friction of the joint actuator.
		
		Pose<DataType> _offset;                                                             ///< Pose with respect to joint of parent link
		
		string _type = "unknown";                                                           ///< Joint type as a string
		
		string _name = "unnamed";                                                           ///< Unique identifier
		
};                                                                                                  // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Joint<DataType>::Joint(const string               &name,
	               const string               &type,
                       const Vector<DataType,3>   &axis,
                       const Pose<DataType>       &offset,
                       const DataType              positionLimit[2],
      		       const DataType             &speedLimit,
      		       const DataType             &effortLimit,
		       const DataType             &damping,
		       const DataType             &friction)
		       :
		       _name(name),
		       _type(type),
		       _axis(axis.normalized()),                                               // Ensure unit norm for good measure
		       _offset(offset),
		       _positionLimit{positionLimit[0],positionLimit[1]},
		       _speedLimit(speedLimit),
		       _effortLimit(effortLimit),
		       _damping(damping),
		       _friction(friction)
{
	// Check that the inputs are sound
	
	if(positionLimit[0] >= positionLimit[1])
	{
		throw logic_error("[ERROR] [JOINT] Constructor: "
		                  "Lower position limit " + to_string(positionLimit[0]) + " is greater than "
		                  "upper position limit " + to_string(positionLimit[1]) + " for joint " + name + ".");
	}
	else if(speedLimit <= 0)
	{
		throw invalid_argument("[ERROR] [JOINT] Constructor: "
		                       "Speed limit for " + name + " joint was " + to_string(speedLimit) +
		                       " but it must be positive.");
	}
	else if(effortLimit <= 0)
	{
		throw invalid_argument("[ERROR] [JOINT] Constructor: "
		                       "Force/torque limit for " + name + " joint was " + to_string(effortLimit) + 
		                       " but it must be positive.");
	}
	else if(damping < 0)
	{
		throw invalid_argument("[ERROR] [JOINT] Constructor: "
		                       "Damping for " + name + " joint was " + to_string(damping) +
		                       " but it must be positive.");
	}
	else if(friction < 0)
	{
		throw invalid_argument("[ERROR] [JOINT] Constructor: "
		                       "Friction for " + name + " joint was " + to_string(friction) +
		                       " but it cannot be negative.");
	}
	
	// Make sure the type is correctly specified
	     if(this->_type == "revolute" or this->_type == "continuous") this->isRevolute = true;
	else if(this->_type == "prismatic")                               this->isRevolute = false;
	else if(this->_type == "fixed")                                   this->isFixed    = true;
	else
	{
		throw invalid_argument("[ERROR] [JOINT] Constructor: "
		                       "Joint type was " + this->_type +
		                       " but expected 'revolute', " + "'continuous', or 'prismatic'.");
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Get the local transform due to the joint position                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Joint<DataType>::pose(const DataType &position)
{
	if(this->isRevolute)
	{
		return Pose<DataType>(Vector<DataType,3>::Zero(),
		                      Quaternion<DataType>(cos(0.5*position),
				                           sin(0.5*position)*this->_axis(0),
				                           sin(0.5*position)*this->_axis(1),
				                           sin(0.5*position)*this->_axis(2)));
	}
	else  return Pose<DataType>(position*this->_axis, Quaternion<DataType>(1, 0, 0, 0));
}

#endif
