/**
 * @file    Link.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 2.0
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
 
#include <Model/Link.h>

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Add a proceeding link in the kinematic chain                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void 
Link::add_child_link(Link *child)
{
    if (child == nullptr) throw std::invalid_argument("[ERROR] [LINK] connect_link(): Input is a null pointer.");

     _childLinks.push_back(child);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Connect a preceding link with this one                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////
void
Link::set_parent_link(Link *parent)
{
     if (parent == nullptr) throw std::invalid_argument("[ERROR] [LINK] connect_link(): Input is a null pointer.");

     _parentLink = parent;                                                                          // Set pointer to parent link
     
     parent->add_child_link(this);                                                                  // Add this link as child to the parent
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Merge another link with this one                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Link::merge(const Link &otherLink)
{
     RobotLibrary::Model::RigidBody::combine_inertia(otherLink, otherLink.joint().origin());        // Add the inertia of the other link to this one
     
     for (auto currentLink : otherLink.child_links())                                               // Cycle through all links attached to other
     {
          currentLink->set_parent_link(this);                                                       // Make this link the new parent
          
          currentLink->joint().extend_origin(otherLink.joint().origin());                           // Extend the origin to this link
     }
     
     // Delete the merged link from the list of child links
     for(int i = 0; i < _childLinks.size(); i++)
     {
          if(_childLinks[i]->name() == otherLink.name())
          {
               _childLinks.erase(_childLinks.begin()+i);    
               break;
          }
     }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Update the kinematics of this link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
void
Link::update_state(const double &jointPosition,
                   const double &jointVelocity)
{
    if(_parentLink == nullptr)                                                             
    {
        throw std::invalid_argument("[ERROR] [LINK] update_state(): The '" + _name + "' link has no parent link. "
                                    "You need to call the update_state() method that specifies the "
                                    "relative pose and twist as arguments.");
    }
    
    update_state(_parentLink->pose(), _parentLink->twist(), jointPosition, jointVelocity);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Update the kinematics of this link                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
void
Link::update_state(const RobotLibrary::Model::Pose &previousPose,
                   const Eigen::Vector<double,6>   &previousTwist,
                   const double                    &jointPosition,
                   const double                    &jointVelocity)                            
{
    if (not _joint.is_revolute() and not _joint.is_prismatic())
    {
        throw std::logic_error("[ERROR] [LINK] update_state(): "
                               "The '" + _joint.name() + "' was " + _joint.type() + " "
                               "but expected revolute or prismatic.");
    }

    _pose = previousPose * _joint.origin();                                                         // Update pose of link in base frame

    _jointAxis = (_pose.rotation()*_joint.axis()).normalized();                                     // Update axis of actuation in base frame

    // NOTE: This line of code can throw an error:
    _pose *= _joint.position_offset(jointPosition);                                                 // Shouldn't this be on line 222? Did I write this???

    _twist = previousTwist;                                                                         // Propagate linear, angular velocity

    _twist.head(3) += Eigen::Vector3d(previousTwist.tail(3)).cross(_pose.translation() - previousPose.translation()); // Added linear velocity due to angular velocity

         if (_joint.is_revolute())  _twist.tail(3) += jointVelocity * _jointAxis;                   // Add effect of joint motion
    else if (_joint.is_prismatic()) _twist.head(3) += jointVelocity * _jointAxis;

    _centerOfMass = _pose * _localCenterOfMass;                                                     // Update center of mass in base frame

    Eigen::Matrix3d R = _pose.rotation();                                                           // Get rotation as SO(3)

    _inertia = R * _localInertia * R.transpose();                                                   // Rotate moment of inertia to base frame

    Eigen::Vector3d w = _twist.tail(3);                                                             // Need to do this so we can call the cross() method below

    for (int i = 0; i < 3; i++) _inertiaDerivative.col(i) = w.cross(_inertia.col(i));               // Compute time derivative of moment of inertia
}

} } // namespace
