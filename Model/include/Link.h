/**
 * @file   Link.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class that combines a rigid body with an actuated joint.
 */

#ifndef LINK_H_
#define LINK_H_

#include "Joint.h"
#include "Pose.h"
#include "RigidBody.h"

#include <vector>                                                                                   // std::vector

class Link : public RigidBody 
{
     public:
     
          /**
           * Constructor.
           * @param rigidBody A RigidBody object that defines its dynamic properties.
           * @param joint A Joint object that defines control properties for actuation.
           */ 
          Link(const RigidBody  &rigidBody,
               const Joint      &joint)
          : RigidBody (rigidBody),
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
          bool update_state(const double &jointPosition,
                            const double &jointVelocity);
          
          /**
           * Updates the kinematic properties of the link.
           * @param previousPose The pose of the parent link in the kinematic chain.
           * @param previousTwist The velocity of the parent link in the kinematic chain.
           * @param jointPosition The position of the joint (rad for revolute, metres for prismatic)
           * @param jointVelocity The speed at which the joint is moving (rad/s for revolute, m/s for prismatic)
           * @return Returns false if there was a problem.
           */
          bool update_state(const Pose                    &previousPose,
                            const Eigen::Vector<double,6> &previousTwist,
                            const double                  &jointPosition,
                            const double                  &jointVelocity);
          
          /**
           * Returns the Joint object associated with this link.
           */
          Joint  joint() const { return this->_joint; }
          
          /**
           * Returns a pointer to the previous joint in the kinematic chain.
           */
          Link* parent_link() const { return this->_parentLink; }
          
          /**
           * Returns the number of this link in the kinematic chain.
           */
          unsigned int number() const { return this->_number; }
          
          /**
           * Returns an array of pointers to the next links attached to this one in a kinematic chain.
           */
          std::vector<Link*> child_links() const { return this->_childLinks; }
          
          /**
           * Returns a unit vector for the joint axis in the base frame of the kinematic tree.
           */
          Eigen::Vector3d joint_axis() const { return this->_jointAxis; }
          
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
     
          Eigen::Vector3d _jointAxis = {0,0,1};                                                     ///< Axis of joint actuation in global frame
          
          Joint  _joint;                                                                            ///< The joint attached to this link
          
          Link * _parentLink = nullptr;                                                             ///< Pointer to previous link in chain (null = attached to base)
          
          std::vector<Link *> _childLinks = {};                                                     ///< Array of proceeding links in a kinematic tree.
          
          unsigned int _number;                                                                     ///< The number in the kinematic chain
          
};                                                                                                  // Semicolon needed at the end of class declaration

#endif
