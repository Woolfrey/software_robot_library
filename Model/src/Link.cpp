/**
 * @file    Link.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class that combines a RobotLibrary::Model::RigidyBody object with a RobotLibrary::Model::Joint object.
 * 
 * @details This class describes a link as a part of a serial link chain. It contains both a joint
 *          and a rigid body to define the kinematic and dynamic properties.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include "Model/Link.h"

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add a proceeding link in the kinematic chain                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool 
Link::add_child_link(Link *child)
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
bool
Link::set_parent_link(Link *parent)
{
     if(parent == nullptr)
     {
          std::cerr << "[ERROR] [LINK] connect_link(): Input is a null pointer." << std::endl;
          
          return false;
     }

     this->_parentLink = parent;                                                                    // Set pointer to parent link
     
     return parent->add_child_link(this);                                                           // Add this link as child to the parent
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Link::merge(const Link &otherLink)
{
     RigidBody ::combine_inertia(otherLink, otherLink.joint().origin());                   // Add the inertia of the other link to this one
     
     for(auto currentLink : otherLink.child_links())                                                // Cycle through all links attached to other
     {
          currentLink->set_parent_link(this);                                                       // Make this link the new parent
          
          currentLink->joint().extend_origin(otherLink.joint().origin());                           // Extend the origin to this link
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
bool
Link::update_state(const double &jointPosition, const double &jointVelocity)
{
     if(this->_parentLink == nullptr)
     {
          std::cerr << "[ERROR] [LINK] update_state(): The '" << this->_name << "' link has no parent link. "
                    << "You need to call the update_state() method that specifies the "
                    << "relative pose and twist as arguments." << std::endl;
          
          return false;
     }
     else return update_state(this->_parentLink->pose(), this->_parentLink->twist(), jointPosition, jointVelocity);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Update the kinematics of this link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool
Link::update_state(const Pose                    &previousPose,
                   const Eigen::Vector<double,6> &previousTwist,
                   const double                  &jointPosition,
                   const double                  &jointVelocity)                            
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
          this->_pose = previousPose * this->_joint.origin();                                       // Update pose of link in base frame
          
          this->_jointAxis = (this->_pose.rotation()*this->_joint.axis()).normalized();             // Update axis of actuation in base frame

          // NOTE: This can throw an error!
          this->_pose *= this->_joint.position_offset(jointPosition);                               // Shouldn't this be on line 222? Did I write this???
 
          this->_twist = previousTwist;                                                             // Propagate linear, angular velocity
          
          this->_twist.head(3) += Eigen::Vector3d(previousTwist.tail(3)).cross(this->_pose.translation() - previousPose.translation()); // Added linear velocity due to angular velocity

               if(this->_joint.is_revolute())  this->_twist.tail(3) += jointVelocity * this->_jointAxis; // Add effect of joint motion
          else if(this->_joint.is_prismatic()) this->_twist.head(3) += jointVelocity * this->_jointAxis;

          this->_centerOfMass = this->_pose*this->_localCenterOfMass;                               // Update center of mass in base frame

          Eigen::Matrix3d R = this->_pose.rotation();                                               // Get rotation as SO(3)
          
          this->_inertia = R*this->_localInertia*R.transpose();                                     // Rotate moment of inertia to base frame
          
          Eigen::Vector3d w = this->_twist.tail(3);                                                 // Need to do this so we can call the cross() method below
          
          for(int i = 0; i < 3; i++) this->_inertiaDerivative.col(i) = w.cross(this->_inertia.col(i)); // Compute time derivative of moment of inertia
          
          return true;
     }
}

} }
