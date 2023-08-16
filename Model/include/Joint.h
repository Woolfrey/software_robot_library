    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
		// Minimum constructor, delegate to full constructor
		Joint(const string               &name,
		      const string               &type,
		      const Vector<DataType,3>   &axis,
		      const DataType             positionLimit[2])
		:
		Joint(name, type, axis, Pose<DataType>(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}

		// Full constructor
		Joint(const string               &name,
		      const string               &type,
		      const Vector<DataType,3>   &axis,
		      const Pose<DataType>       &offset,
		      const DataType              positionLimit[2],                                 // Can't use pointers & for arrays for some reason
		      const DataType             &speedLimit,
		      const DataType             &effortLimit,
		      const DataType             &damping,
		      const DataType             &friction);
		
		// Methods
		
		bool is_fixed() const { return this->isFixed; }
		
		bool is_prismatic() const { return not this->isRevolute; }                          // Because I'm lazy
		
		bool is_revolute() const { return this->isRevolute; }
		
		Vector<DataType,3> axis() const { return this->_axis; }		                    // Axis in LOCAL frame
		
		Pose<DataType> pose(const DataType &position);                                      // Return local transform given position	
		
		Pose<DataType> offset() const { return this->_offset; }                             // Get the pose of the joint relative to pose of joint on parent link
		
		string type() const { return this->_type; }
		
		string name() const { return this->_name; }                                         // Get the name of this joint
		
		unsigned int number() const { return this->_number; }                               // Get the index for this joint in the joint vector
		
		void extend_offset(const Pose<DataType> &other)
		{ 
			Pose<DataType> temp = other;
			this->_offset = temp*this->_offset;                                         // Extend the offset for fixed joints when merging links
		} 
				
		void position_limits(DataType &lower, DataType &upper) { lower = this->_positionLimit[0];
		                                                         upper = this->_positionLimit[1]; }
		
	private:
	
		bool isRevolute = true;
		
		bool isFixed = false;
		
		Vector<DataType,3> _axis;                                                           // Axis of actuation in LOCAL frame

		DataType _positionLimit[2];
		
		DataType _speedLimit;
		
		DataType _effortLimit;
		
		DataType _damping;
		
		DataType _friction;
		
		Pose<DataType> _offset;                                                             // Pose with respect to joint of parent link
		
		string _type = "unknown";
		
		string _name = "unnamed";                                                           // Unique identifier
		
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
