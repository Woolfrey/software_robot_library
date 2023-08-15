    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //        A class that combines a RigidBody with a Joint to form a Link of a robot chain         //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

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
		Link(const RigidBody<DataType> &rigidBody,
		     const Joint<DataType>     &_joint)
		:
		RigidBody<DataType>(rigidBody),
		joint(_joint) {}
	
		// Properties
		
		Joint<DataType> joint;
		
		// Methods
		
		bool set_parent_link(Link *parent);
		
		bool add_child_link(Link *child);
		
		bool update_state(const Pose<DataType>     &pose,
		                  const Vector<DataType,6> &twist,
		                  const DataType           &jointPosition,
		                  const DataType           &jointVelocity);
		
		Link* parent_link() const { return this->parentLink; }
		
		Pose<DataType> pose() const { return this->_pose; }
		
		unsigned int number() const { return this->_number; }
		
		vector<Link*> child_links() const { return this->childLinks; }
		
		Vector<DataType,3> angular_velocity() const { return this->_angularVelocity; }
		
		Vector<DataType,3> axis() const { return this->_axis; }
		
		void clear_parent_link() { this->parentLink = nullptr; }

		void merge(const Link &otherLink);                                                  // Merge the properties of the other link with this one
		
		void set_number(const unsigned int &number) { this->_number = number; }
		
		void set_angular_velocity(const Vector<DataType,3> &velocity)
		{
			this->_angularVelocity = velocity;
		}
		
		void update_state(const Pose<DataType> &pose, const DataType &jointPosition);
		
	private:
		
		Link<DataType>* parentLink = nullptr;
		
		Pose<DataType> _pose;
		
		unsigned int _number;                                                               // The number in the kinematic chain
		
		Vector<DataType,3> _angularVelocity;
		
		Vector<DataType,3> _axis;
		
		vector<Link<DataType>*> childLinks = {};
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
	RigidBody<DataType>::combine_inertia(otherLink, otherLink.joint.offset());                  // Add the inertia of the other link to this one
	
	std::vector<Link*> otherChildLinks = otherLink.child_links();                               // Get the list of child links for the merged link
	
	for(int i = 0; i < otherChildLinks.size(); i++)
	{
		otherChildLinks[i]->set_parent_link(this);                                          // Make this link the parent of the other child links
	
		//otherChildLinks[i]->joint.extend_offset(otherLink.joint.offset());                  // Extend the offset
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
template<class DataType> inline
void Link<DataType>::update_state(const Pose<DataType> &previousPose, const DataType &jointPosition)
{
	Pose<DataType> wtf = previousPose;
	
	this->_pose = wtf * this->joint.pose(jointPosition);
	
	this->_axis = this->_pose.rotation() * this->joint.axis();
	
	this->_axis.normalize();
}
                  
#endif
