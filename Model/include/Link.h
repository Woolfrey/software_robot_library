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

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;                                                                                // std::cerr, std::vector, std::string
 
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
		bool update_state(const Pose<DataType>     &previousPose,
		                  const Vector<DataType,6> &previousTwist,
		                  const DataType           &jointPosition,
		                  const DataType           &jointVelocity);
		
		/**
		 * Returns the Joint object associated with this link.
		 */
		Joint<DataType> joint() const { return this->_joint; }
		
		/**
		 * Returns a pointer to the previous joint in the kinematic chain.
		 */
		Link* parent_link() const { return this->parentLink; }
		
		/**
		 * Returns the number of this link in the kinematic chain.
		 */
		unsigned int number() const { return this->_number; }
		
		/**
		 * Returns an array of pointers to all the proceeding links in a kinematic chain.
		 */
		vector<Link*> child_links() const { return this->childLinks; }
		
		/**
		 * Returns a unit vector for the joint axis in the base frame of the kinematic tree.
		 */
		Vector<DataType,3> axis() const { return this->_axis; }
		
		/**
		 * Set the pointer to the proceeding link in a kinematic chain as null.
		 */
		void clear_parent_link() { this->parentLink = nullptr; }

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
	
		Joint<DataType> _joint;                                                             ///< The joint attached to this link
		
		Link<DataType>* parentLink = nullptr;                                               ///< Pointer to previous link in chain (null = attached to base)
		
		unsigned int _number;                                                               ///< The number in the kinematic chain
		
		Vector<DataType,3> _axis;                                                           ///< Axis of joint actuation in global frame
		
		vector<Link<DataType>*> childLinks = {};                                            ///< Array of proceeding links in a kinematic tree.
		
};                                                                                                  // Semicolon needed at the end of class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add a proceeding link in the kinematic chain                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool Link<DataType>::add_child_link(Link *child)
{
	if(child == nullptr)
	{
		cerr << "[ERROR] [LINK] connect_link(): Input is a null pointer.\n";
		
		return false;
	}
	
	this->childLinks.push_back(child);
	
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
		cerr << "[ERROR] [LINK] connect_link(): Input is a null pointer.\n";
		
		return false;
	}

	this->parentLink = parent;
	
	return parent->add_child_link(this);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
void Link<DataType>::merge(const Link &otherLink)
{
	RigidBody<DataType>::combine_inertia(otherLink, otherLink.joint().offset());                // Add the inertia of the other link to this one
	
	std::vector<Link*> otherChildLinks = otherLink.child_links();                               // Get the list of child links for the merged link
	
	for(int i = 0; i < otherChildLinks.size(); i++)
	{
		otherChildLinks[i]->set_parent_link(this);                                          // Make this link the parent of the other child links
	
		otherChildLinks[i]->joint().extend_offset(otherLink.joint().offset());              // Extend the offset
	}
	
	// Delete the merged link from the list of child links
	for(int i = 0; i < this->childLinks.size(); i++)
	{
		if(this->childLinks[i]->name() == otherLink.name())
		{
			this->childLinks.erase(this->childLinks.begin()+i);    
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
	if(this->parentLink == nullptr)
	{
		cerr << "[ERROR] [LINK] update_state(): "
		        "The '" << this->_name << "' link has no parent link. "
		        "You need to call the update_state() method that specifies the "
		        "relative pose and twist as arguments.\n";
		
		return false;
	}
	else	return update_state(this->parentLink->pose(), this->parentLink->twist(), jointPosition, jointVelocity);
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
	if(not this->joint().is_revolute() and not this->joint().is_prismatic())
	{
		cerr << "[ERROR] [LINK] update_state(): "
		     << "The '" << this->joint().name() << "' was " << this->joint().type() << " "
		     << "but expected revolute or prismatic.\n";
		
		return false;
	}
	else
	{
		Pose<DataType> wtf = previousPose;                                                          
		
		this->_pose = wtf * this->joint().offset() * this->joint().pose(jointPosition);     // Using 'previousPose' directly won't compile??
		
		this->_axis = this->_pose.rotation() * this->joint().axis();                        // Rotate to global frame
		
		this->_axis.normalize();                                                            // Ensure unit norm
		
		Vector<DataType,3> angularVel = previousTwist.tail(3);                              // Need this to perform cross() 
		
		this->_twist.head(3) = previousTwist.head(3)
			             + angularVel.cross(this->_pose.translation() - previousPose.translation());
			             
		this->_twist.tail(3) = angularVel;
		
		     if(this->_joint.is_revolute())  this->_twist.tail(3) += jointVelocity * this->_axis; // Add joint effect
		else if(this->_joint.is_prismatic()) this->_twist.head(3) += jointVelocity * this->_axis;
	
		return true;
	}
}
                  
#endif
