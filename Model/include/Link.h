/**
 * @file   Link.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class that combines a rigid body with an actuated joint.
 */

#ifndef LINK_H_
#define LINK_H_

#include <Joint.h>
#include <Pose.h>
#include <RigidBody.h>                                                                              // Custom class
#include <vector>                                                                                   // std::vector
 
template <class DataType>
class Link : public RigidBody<DataType>
{
	public:
	
		/**
		 * Constructor.
		 * @param rigidBody A RigidBody object that defines its dynamic properties.
		 * @param joint A Joint object that defines control properties for actuation.
		 */ 
		Link(const RigidBody<DataType> &rigidBody,
		     const Joint<DataType>     &joint)
		:
		RigidBody<DataType>(rigidBody),
		_joint(joint) {}
		
		
		/**
		 * Sets a pointer to a preceding link in a kinematic chain.
		 * @param parent A pointer to the preceding Link object.
		 * @return Returns false if there was a problem.
		 */
		bool set_parent_link(Link *parent);
		
		/**
		 * Adds a pointer to a list of proceeding joints in a kinematic chain.
		 * @param child A pointer to the proceeding Link object
		 * @param Return false if there was a problem.
		 */
		bool add_child_link(Link *child);
		
		/**
		 * Updates the kinematic properties of the link.
		 * @param jointPosition The position of the joint (radians for revolute joints, and metres for prismatic joints)
		 * @param jointVelocity The speed with which the joint is moving (rad/s for revolute, m/s for prismatic)
		 * @return Returns false if there was a problem.
		 */
		bool update_state(const DataType &jointPosition,
		                  const DataType &jointVelocity);
		
		/**
		 * Updates the kinematic properties of the link.
		 * @param previousPose The pose of the parent link in the kinematic chain.
		 * @param previousTwist The velocity of the parent link in the kinematic chain.
		 * @param jointPosition The position of the joint (rad for revolute, metres for prismatic)
		 * @param jointVelocity The speed at which the joint is moving (rad/s for revolute, m/s for prismatic)
		 * @return Returns false if there was a problem.
		 */
		bool update_state(const Pose<DataType>            &previousPose,
		                  const Eigen::Vector<DataType,6> &previousTwist,
		                  const DataType                  &jointPosition,
		                  const DataType                  &jointVelocity);
		
		/**
		 * Returns the Joint object associated with this link.
		 */
		Joint<DataType> joint() const { return this->_joint; }
		
		/**
		 * Returns a pointer to the previous joint in the kinematic chain.
		 */
		Link* parent_link() const { return this->_parentLink; }
		
		/**
		 * Returns the number of this link in the kinematic chain.
		 */
		unsigned int number() const { return this->_number; }
		
		/**
		 * Returns an array of pointers to all the proceeding links in a kinematic chain.
		 */
		std::vector<Link*> child_links() const { return this->_childLinks; }
		
		/**
		 * Returns a unit vector for the joint axis in the base frame of the kinematic tree.
		 */
		Eigen::Vector<DataType,3> joint_axis() const { return this->_jointAxis; }
		
		/**
		 * Set the pointer to the proceeding link in a kinematic chain as null.
		 */
		void clear_parent_link() { this->_parentLink = nullptr; }

		/**
		 * Merge the dynamic properties of another link in to this one.
		 * @param otherLink The link to be merged in to this one.
		 */
		void merge(const Link &otherLink);
		
		/**
		 * Set the number of this link in the kinematic tree.
		 * @param number The number to be assigned.
		 */
		void set_number(const unsigned int &number) { this->_number = number; }
		
	private:
	
		Eigen::Vector<DataType,3> _jointAxis = {0,0,1};                                      ///< Axis of joint actuation in global frame
		
		Joint<DataType> _joint;                                                             ///< The joint attached to this link
		
		Link<DataType>* _parentLink = nullptr;                                              ///< Pointer to previous link in chain (null = attached to base)
		
		std::vector<Link<DataType>*> _childLinks = {};                                      ///< Array of proceeding links in a kinematic tree.
		
		unsigned int _number;                                                               ///< The number in the kinematic chain
		
};                                                                                                  // Semicolon needed at the end of class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add a proceeding link in the kinematic chain                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool Link<DataType>::add_child_link(Link *child)
{
	if(child == nullptr)
	{
		std::cerr << "[ERROR] [LINK] connect_link(): Input is a null pointer." << std::endl;
		
		return false;
	}
	
	this->_childLinks.push_back(child);
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Connect a preceding link with this one                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool Link<DataType>::set_parent_link(Link *parent)
{
	if(parent == nullptr)
	{
		std::cerr << "[ERROR] [LINK] connect_link(): Input is a null pointer." << std::endl;
		
		return false;
	}

	this->_parentLink = parent;                                                                 // Set pointer to parent link
	
	return parent->add_child_link(this);                                                        // Add this link as child to the parent
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
void Link<DataType>::merge(const Link &otherLink)
{
	RigidBody<DataType>::combine_inertia(otherLink, otherLink.joint().origin());                // Add the inertia of the other link to this one
	
	for(auto currentLink : otherLink.child_links())                                             // Cycle through all links attached to other
	{
		currentLink->set_parent_link(this);                                                 // Make this link the new parent
		
		currentLink->joint().extend_origin(otherLink.joint().origin());                     // Extend the origin to this link
	}
	
	// Delete the merged link from the list of child links
	for(int i = 0; i < this->_childLinks.size(); i++)
	{
		if(this->_childLinks[i]->name() == otherLink.name())
		{
			this->_childLinks.erase(this->_childLinks.begin()+i);    
			break;
		}
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Update the kinematics of this link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool Link<DataType>::update_state(const DataType &jointPosition, const DataType &jointVelocity)
{
	if(this->_parentLink == nullptr)
	{
		std::cerr << "[ERROR] [LINK] update_state(): The '" << this->_name << "' link has no parent link. "
		             "You need to call the update_state() method that specifies the "
		             "relative pose and twist as arguments." << std::endl;
		
		return false;
	}
	else	return update_state(this->_parentLink->pose(), this->_parentLink->twist(), jointPosition, jointVelocity);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Update the kinematics of this link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template<class DataType> inline
bool Link<DataType>::update_state(const Pose<DataType>     &previousPose,
                                  const Vector<DataType,6> &previousTwist,
                                  const DataType           &jointPosition,
                                  const DataType           &jointVelocity)                            
{
	if(not this->_joint.is_revolute() and not this->_joint.is_prismatic())
	{
		std::cerr << "[ERROR] [LINK] update_state(): "
		          << "The '" << this->_joint.name() << "' was " << this->_joint.type() << " "
		          << "but expected revolute or prismatic." << std::endl;
		
		return false;
	}
	else
	{
		this->_pose = previousPose * this->_joint.origin();
		
		this->_jointAxis = (this->_pose.rotation()*this->_joint.axis()).normalized();
		
		this->_pose *= this->_joint.position_offset(jointPosition);
		
		this->_twist = previousTwist;
		
		this->_twist.head(3) += Eigen::Vector<DataType,3>(previousTwist.tail(3)).cross(this->_pose.translation() - previousPose.translation());

		     if(this->_joint.is_revolute())  this->_twist.tail(3) += jointVelocity * this->_jointAxis; // Add effect of joint motion
		else if(this->_joint.is_prismatic()) this->_twist.head(3) += jointVelocity * this->_jointAxis;

		this->_centerOfMass = this->_pose*this->_localCenterOfMass;

		Eigen::Matrix<DataType,3,3> R = this->_pose.rotation();
		
		this->_inertia = R*this->_localInertia*R.transpose();
		
		Eigen::Vector<DataType,3> w = this->_twist.tail(3);
		for(int i = 0; i < 3; i++) this->_inertiaDerivative.col(i) = w.cross(this->_inertia.col(i));
		
		return true;
	}
}
                  
#endif
