    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A class representing a moveable joint between two rigid bodies                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINT_H_
#define JOINT_H_

#include <Pose.h>
#include <vector>


class Joint
{
	public:
		// Minimum constructor, delegate to full constructor
		Joint(const std::string     &name,
		      const std::string     &type,
		      const Eigen::Vector3f &axis,
		      const float            positionLimit[2])
		:
		Joint(name, type, axis, Pose(), positionLimit, 100*2*M_PI/60, 10.0, 1.0, 0.0) {}

		// Full constructor
		Joint(const std::string     &name,
		      const std::string     &type,
		      const Eigen::Vector3f &axis,
		      const Pose            &origin,
		      const float            positionLimit[2],
		      const float           &velocityLimit,
		      const float           &forceLimit,
		      const float           &damping,
		      const float           &friction);
		
		// Functions
		
		Pose origin()      const { return this->_origin; }                                  // Joint pose relative to preceeding link
	
		std::string name() const { return this->_name; }
		
		std::string type() const { return this->_type; }
		
	private:
	
		Eigen::Vector3f _axis;                                                              // Axis of actuation for this joint

		float _positionLimit[2];
		float _velocityLimit;
		float _forceLimit;
		float _damping;
		float _friction;
		
		Pose _origin;                                                                       // Origin relative to preceding link
		
		std::string _name;                                                                  // Unique identifier
		std::string _type;

};                                                                                                  // Semicolon needed after a class declaration

#endif
