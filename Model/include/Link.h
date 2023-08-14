    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                       A class for building a kinematic tree data structure                    //
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
		
		void merge(const Link &otherLink);                                                        // Merge the properties of the other link with this one

		bool set_parent_link(Link *parent);
		
		bool add_child_link(Link *child);
		
		Link* parent_link() const { return this->parentLink; }
		
		vector<Link*> child_links() const { return this->childLinks; }
		
		Vector<DataType,3> angular_velocity() const { return this->angularVelocity; }
		
		void clear_parent_link() { this->parentLink = nullptr; }
		
	private:
		
		Link* parentLink = nullptr;
		
		vector<Link*> childLinks = {};
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
	
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
void Link<DataType>::merge(const Link &otherLink)
{
	RigidBody<DataType>::combine_inertia(otherLink, otherLink.joint.offset());                            // Add the inertia of the other link to this one
	
	// Set this link as the parent link for all child links of the other link
	std::vector<Link*> otherChildLinks = otherLink.child_links();
	
	for(int i = 0; i < otherChildLinks.size(); i++) otherChildLinks[i]->set_parent_link(this);	
}

#endif
