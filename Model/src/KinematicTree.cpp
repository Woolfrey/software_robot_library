/**
 * @file    KinematicTree.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   A class for a multi rigid body system of branching serial link structures.
 * 
 * @details This class is used to compute the kinematics and dynamics of branching, serial link structures.
 *          It presumes only open-chain branches. It computes forward kinematics, and inverse dynamics.
 *          It is designed to be embedded in to a control class to obtain things like the Jacobian,
 *          inertia matrix, Coriolis matrix, etc.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Model/KinematicTree.h>
#include <deque>

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
     using namespace Eigen;                                                                         // Eigen::Matrix, Eigen::Vector
     using namespace RobotLibrary::Model;                                                           // ReferenceFrame, Link, Pose
     using namespace std;                                                                           // std::string, std::map
     using namespace tinyxml2;                                                                      // tinyxml2::XMLDocument, tinyxml2::XMLElement

     // Variables used in this scope
     map<string, string>       connectionList;                                                      // For storing parent/child connection between links
     map<string, unsigned int> linkList;                                                            // Search for links by name
     map<string, RigidBody>    rigidBodyList;                                                       // Store urdf links / RigidBody objects
     
     // Check to see if the file exists
     if(not ifstream(pathToURDF.c_str()))
     {
          throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
                              "The file " + pathToURDF + " does not appear to exist.");
     }

     XMLDocument urdf; urdf.LoadFile(pathToURDF.c_str());                                           // Load URDF           
     
     // Make sure a robot is defined
     XMLElement* robot = urdf.FirstChildElement("robot");
     
     if(robot == nullptr)
     {
          throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
                              "There does not appear to be a 'robot' element in the URDF.");
     }

     this->_name = robot->Attribute("name");                                                        // Assign the name

     // Search through every 'link' in the urdf file and convert it to a RigidBody object
     for(auto iterator = robot->FirstChildElement("link"); iterator; iterator = iterator->NextSiblingElement("link"))
     {
          // Default properties
          double mass = 0.0;
          string name = iterator->Attribute("name");
          Vector3d   centreOfMass = {0,0,0};
          Matrix<double,3,3> momentOfInertia; momentOfInertia.setZero();

          // Get the inertia properties if they exist
          XMLElement *inertial = iterator->FirstChildElement("inertial");
          
          if(inertial != nullptr)
          {
               // Need to force a conversion here if double == float
               mass = inertial->FirstChildElement("mass")->DoubleAttribute("value");

               double ixx = inertial->FirstChildElement("inertia")->DoubleAttribute("ixx");
               double ixy = inertial->FirstChildElement("inertia")->DoubleAttribute("ixy");
               double ixz = inertial->FirstChildElement("inertia")->DoubleAttribute("ixz");
               double iyy = inertial->FirstChildElement("inertia")->DoubleAttribute("iyy");
               double iyz = inertial->FirstChildElement("inertia")->DoubleAttribute("iyz");
               double izz = inertial->FirstChildElement("inertia")->DoubleAttribute("izz");
               
               momentOfInertia << ixx, ixy, ixz,
                                  ixy, iyy, iyz,
                                  ixz, iyz, izz;

               XMLElement* origin = inertial->FirstChildElement("origin");
               
               if(origin != nullptr)
               {
                    Eigen::Vector3d xyz = char_to_vector(origin->Attribute("xyz"));
                    Eigen::Vector3d rpy = char_to_vector(origin->Attribute("rpy"));
                    
                    // NOTE: URDF specifies a pose for the center of mass,
                    //       which is a little superfluous, so we will reduce it
                    Pose pose(xyz, AngleAxis<double>(rpy(0),Vector3d::UnitX())
                                            *AngleAxis<double>(rpy(1),Vector3d::UnitY())
                                            *AngleAxis<double>(rpy(2),Vector3d::UnitZ()));
                                                           

                    centreOfMass = pose.translation();                                              // Location for the center of mass
  
                    Matrix<double,3,3> R = pose.rotation();                                         // Get the SO(3) matrix
                    
                    momentOfInertia = R*momentOfInertia*R.transpose();                              // Rotate the inertia to the origin frame
               }
          }
     
          rigidBodyList.emplace(name, RigidBody(name,mass,momentOfInertia,centreOfMass));           // Put it in the list so we can find it later
     }

     // Search through every joint and create a RobotLibrary::Joint object
     unsigned int linkNumber = 0;
     for(auto iterator = robot->FirstChildElement("joint"); iterator; iterator = iterator->NextSiblingElement("joint"))
     {
          string jointName = iterator->Attribute("name");
          string jointType = iterator->Attribute("type");

          // Get the joint properties
          if(jointType == "continuous"
          or jointType == "fixed"
          or jointType == "prismatic"
          or jointType == "revolute")
          {
               // Default properties
               double damping       = 1.0;
               double effortLimit   = 10;
               double friction      = 0.0;
               RobotLibrary::Model::Limits positionLimit = {-M_PI, M_PI};                                                // Data structure in Joint.h
               double velocityLimit = 100*2*M_PI/60;
               Vector3d axis        = {0,0,1};
               Pose origin(Vector3d::Zero(), Quaterniond(1,0,0,0));
               
               // Get the pose of the joint relative to preceeding link
               XMLElement* originElement = iterator->FirstChildElement("origin");

               if(originElement != nullptr)
               {
                    Vector3d xyz = char_to_vector(originElement->Attribute("xyz"));
                    Vector3d rpy = char_to_vector(originElement->Attribute("rpy"));

                    origin = Pose(xyz, AngleAxis<double>(rpy(2),Vector3d::UnitZ())
                                      *AngleAxis<double>(rpy(1),Vector3d::UnitY())
                                      *AngleAxis<double>(rpy(0),Vector3d::UnitX()));
               }

               // Get joint properties for non-fixed type
               if(jointType != "fixed")
               {     
                    // Get the joint limits     
                    if(jointType == "continuous")                                                   // No limits on the joints
                    { 
                         double inf = numeric_limits<double>::infinity();
                         
                         positionLimit.lower =-inf;
                         positionLimit.upper = inf;
                         velocityLimit       = inf;
                         effortLimit         = inf;
                    }
                    else                                                                            // Obtain the joint limits
                    {
                         XMLElement* limit = iterator->FirstChildElement("limit");
                         if(limit != nullptr)
                         {
                              positionLimit.lower = limit->DoubleAttribute("lower");
                              positionLimit.upper = limit->DoubleAttribute("upper");
                              velocityLimit       = limit->DoubleAttribute("velocity");             // NOTE: This is apparently optional so could cause problems in the future
                              effortLimit         = limit->DoubleAttribute("effort");
                         }
                    }
                    
                    // Get the axis of actuation                         
                    XMLElement* axisElement = iterator->FirstChildElement("axis");
                    if(axisElement != nullptr) axis = char_to_vector(axisElement->Attribute("xyz"));
                    
                    // Get the dynamic properties
                    XMLElement* dynamics = iterator->FirstChildElement("dynamics");
                    if(dynamics != nullptr)
                    {
                         damping  = dynamics->DoubleAttribute("damping");
                         friction = dynamics->DoubleAttribute("friction");
                    }
               }
               
               Joint joint(jointName, jointType, axis, origin, positionLimit, velocityLimit, effortLimit, damping, friction);
               
               // Find the "child link", i.e. RigidBody object, and form a RobotLibrary::Link object
               
               string childLinkName = iterator->FirstChildElement("child")->Attribute("link");
                        
               const auto &rigidBody = rigidBodyList.find(childLinkName)->second;
                                      
               this->_fullLinkList.emplace_back(rigidBody, joint);                                  // Create new link, add to vector
                        
               linkList.emplace(childLinkName, linkNumber);                                         // Add the index to the std::map so we can find it later
               
               linkNumber++;                                                                        // Iterate the link/joint number
               
               rigidBodyList.erase(childLinkName);                                                  // Delete the rigid body from the list
               
               // Get the parent link name, store it to search later
               
               string parentLinkName = iterator->FirstChildElement("parent")->Attribute("link");
               
               connectionList.emplace(childLinkName, parentLinkName);                               // Store this so we can search later
          }
     }

     // Find and set the base
     if(rigidBodyList.size() == 1)
     {
          this->base = rigidBodyList.begin()->second;                                               // Get underlying RigidBody object
          
          this->_baseName = rigidBodyList.begin()->first;                                           // Save the name
     }
     else
     {
          string message = "[ERROR] [KINEMATIC TREE] Constructor: More than one candidate for the base:\n";
          
          for(const auto &[name, object] : rigidBodyList) message + "   - " + name + "\n";          // Loop over all elements in the map
          
          throw std::runtime_error(message);
     }

     // Form the connections between all the links
     for(auto &currentLink : this->_fullLinkList)
     {
          std::string parentLinkName = connectionList.find(currentLink.name())->second;             // Get the name of the parent link
          
          if(parentLinkName != this->base.name())                                                   // Not directly attached to the base
          {
               unsigned int parentLinkNum = linkList.find(parentLinkName)->second;                  // Get the number of the parent link
               
               currentLink.set_parent_link(&this->_fullLinkList[parentLinkNum]);                    // Set the parent/child link relationship
          }
     }

     unsigned int activeJointCount = 0;
     
     // Merge the dynamic properties of all the links connected by fixed joints     
     for(auto &currentLink : this->_fullLinkList)
     {
          if(currentLink.joint().is_fixed())                                                        // Joint is fixed, so merge with parent link
          {
               Link *parentLink = currentLink.parent_link();                                        // Pointer to the parent link
          
               if(parentLink == nullptr)                                                            // No parent link, must be attached to the base!
               {
                    this->base.combine_inertia(currentLink,currentLink.joint().origin());           // Combine inertia of this link with the base
                    
                    for(auto childLink : currentLink.child_links())                                 // Cycle through all the child links
                    {
                         childLink->clear_parent_link();                                            // Link has been merged, so sever the connection
                    }
               }
               else parentLink->merge(currentLink);                                                 // Merge this link in to the previous
          
               ReferenceFrame frame = {parentLink, currentLink.joint().origin()};
               
               this->_frameList.emplace(currentLink.name(),frame);                                   // Save this link as a reference frame so we can search it later
          }
          else
          {
               if(currentLink.parent_link() == nullptr) this->_baseLinks.push_back(&currentLink);   // This link is attached directly to the base
               currentLink.set_number(activeJointCount);                                            // Set the number in the kinematic tree
               this->_link.push_back(&currentLink);                                                 // Add this link to the active list
               activeJointCount++;                                                                  // Increment the counter of active links
          }
     }

     this->_numberOfJoints = this->_link.size();
     
     // Resize the relevant matrices, vectors accordingly
     this->_jointPosition.resize(this->_numberOfJoints);
     this->_jointVelocity.resize(this->_numberOfJoints);
     this->_jointInertiaMatrix.resize(this->_numberOfJoints, this->_numberOfJoints);
     this->_jointCoriolisMatrix.resize(this->_numberOfJoints, this->_numberOfJoints);
     this->_jointDampingVector.resize(this->_numberOfJoints);
     this->_jointGravityVector.resize(this->_numberOfJoints);
     this->_jointBaseInertiaMatrix.resize(this->_numberOfJoints, NoChange);
     this->_jointBaseCoriolisMatrix.resize(this->_numberOfJoints, NoChange);
     
     std::cout << "[INFO] [KINEMATIC TREE] Successfully generated the '" << this->_name << "' robot model."
               << " It has " << this->_numberOfJoints << " joints (reduced from " << this->_fullLinkList.size() << ")." << std::endl;

     #ifndef NDEBUG
          std::cout << "\nThe base link is: " << this->_baseName << ".\n";
          
          std::cout << "\nHere is a list of all the joints on the robot:\n";          
          vector<Link*> candidateList = this->_baseLinks;                                           // Start with links connect to base
          while(candidateList.size() > 0)
          {
               Link *currentLink = candidateList.back();                                            // Get the one at the end
               
               candidateList.pop_back();                                                            // Delete it from the search list
               
               std::cout << " - " << currentLink->joint().name() << "\n";
               
               vector<Link*> temp = currentLink->child_links();                                     // All the links attached to the current link
               
               if(temp.size() != 0) candidateList.insert(candidateList.end(), temp.begin(), temp.end());
          }
          
          std::cout << "\nHere are the reference frames on the robot:\n";
          for(const auto &[name, frame] : this->_frameList)
          {
               std::cout << " - " << name << " is on the ";
               
               if(frame.link == nullptr) std::cout << this->base.name() << ".\n";
               else                      std::cout << "'" << frame.link->name() << "' link.\n";
          }
          std::cout << "\n";
          
     #endif
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Update the kinematics and dynamics                               //        
///////////////////////////////////////////////////////////////////////////////////////////////////
bool KinematicTree::update_state(const Eigen::VectorXd &jointPosition,
                                 const Eigen::VectorXd &jointVelocity,
                                 const RobotLibrary::Model::Pose &basePose,
                                 const Eigen::Vector<double,6> &baseTwist)
{
    using namespace RobotLibrary::Model;
    
    if(jointPosition.size() != this->_numberOfJoints || jointVelocity.size() != this->_numberOfJoints)
    {
        std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
                  << "This model has " << this->_numberOfJoints << " joints, but "
                  << "the joint position argument had " << jointPosition.size() << " elements, "
                  << "and the joint velocity argument had " << jointVelocity.size() << " element." << std::endl;
                  
        return false;
    }
    
    this->_jointPosition = jointPosition;    
    this->_jointVelocity = jointVelocity;    
    this->base.update_state(basePose, baseTwist);
    
    // Clear matrices
    this->_jointInertiaMatrix.setZero();
    this->_jointCoriolisMatrix.setZero();
    this->_jointGravityVector.setZero();
    this->_jointBaseInertiaMatrix.setZero();
    this->_jointBaseCoriolisMatrix.setZero();
    
    std::deque<Link*> candidateList(this->_baseLinks.begin(), this->_baseLinks.end());
    
    while(not candidateList.empty())
    {
        Link *currentLink = candidateList.back();
        candidateList.pop_back();
        
        unsigned int k = currentLink->number();
        Link *parentLink = currentLink->parent_link();
        
        bool success = parentLink == nullptr 
                     ? currentLink->update_state(this->base.pose(), this->base.twist(), jointPosition(k), jointVelocity(k))
                     : currentLink->update_state(jointPosition(k), jointVelocity(k));
        
        if(not success)
        {
            std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
                      << "Unable to update the state for the '" << currentLink->name()
                      << "' link (" << currentLink->joint().name() << " joint).\n";
                      
            return false;
        }
        
        Eigen::Matrix<double,6,Eigen::Dynamic> J  = jacobian(currentLink, currentLink->center_of_mass(), k+1);
        Eigen::Matrix<double,3,Eigen::Dynamic> Jv = J.block(0,0,3,k+1);
        Eigen::Matrix<double,3,Eigen::Dynamic> Jw = J.block(3,0,3,k+1);
        
        double mass = currentLink->mass();
        
        for(int i = 0; i < k+1; ++i)
        {
            for(int j = i; j < k+1; ++j)
            {
                this->_jointInertiaMatrix(i,j) += mass * Jv.col(i).dot(Jv.col(j))
                                                + Jw.col(i).dot(currentLink->inertia() * Jw.col(j));
            }
        }
        
        Eigen::Matrix<double,6,Eigen::Dynamic> Jdot = time_derivative(J);
        
        this->_jointCoriolisMatrix.block(0,0,k+1,k+1) += mass * Jv.transpose() * Jdot.block(0,0,3,k+1)
                                                       + Jw.transpose() * (currentLink->inertia_derivative() * Jw + currentLink->inertia() * Jdot.block(3,0,3,k+1));
        
        this->_jointGravityVector.head(k+1) -= mass * Jv.transpose() * this->_gravityVector;
        this->_jointDampingVector[k] = currentLink->joint().damping() * this->_jointVelocity[k];
        
        using namespace RobotLibrary::Math;
        
        this->_jointBaseInertiaMatrix.block(0,0,k+1,3) += mass * Jv.transpose();
        this->_jointBaseInertiaMatrix.block(0,3,k+1,3) += Jw.transpose() * this->base.inertia()
                                                        - mass * (SkewSymmetric(currentLink->center_of_mass() - this->base.pose().translation()) * Jv).transpose();
        
        this->_jointBaseCoriolisMatrix.block(0,3,k+1,3) += Jw.transpose() * this->base.inertia_derivative()
                                                         - mass * (SkewSymmetric(currentLink->twist().head(3)) * Jv).transpose();
        
        std::vector<Link*> temp = currentLink->child_links();
        if(not temp.empty()) candidateList.insert(candidateList.begin(), temp.begin(), temp.end());
    }
    
    for(int i = 1; i < this->_numberOfJoints; ++i)
    {
        for(int j = 0; j < i; ++j) this->_jointInertiaMatrix(i,j) = this->_jointInertiaMatrix(j,i);
    }
    
    return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the Jacobian to the specified reference frame                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,Eigen::Dynamic>
KinematicTree::jacobian(ReferenceFrame *frame)
{
    if(frame == nullptr)
    {
        throw std::runtime_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                                 "Pointer to reference frame was empty.");
    }

    else
    {
        return jacobian(frame->link,
                       (frame->link->pose()*frame->relativePose).translation(),
                        this->_numberOfJoints);
    }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 6, Eigen::Dynamic>
KinematicTree::jacobian(RobotLibrary::Model::Link *link,                                            // Starting point in the kinematic tree
                        const Eigen::Vector3d &point,                                               // A point in the given joint frame
                        const unsigned int &numberOfColumns)                                        // Must be less than or equal to number of joints in model
{
     // Whitney, D. E. (1972)
     // "The mathematics of coordinated control of prosthetic arms and manipulators"
     // Journal of Dynamic Systems, Measurement, and Control, pp. 303-309
     
     Eigen::Matrix<double,6,Eigen::Dynamic> J(6,numberOfColumns); J.setZero();
     
     if(link->number() > numberOfColumns)
     {
          throw std::logic_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                                 "Cannot compute a Jacobian with " + std::to_string(numberOfColumns) +
                                 "columns using " + std::to_string(link->number()+1) + " joints.");
     }
     else if(numberOfColumns > this->_numberOfJoints)
     {
          throw std::logic_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                                 "A Jacobian matrix with " + std::to_string(numberOfColumns) + " columns was requested, "
                                 "but this model has only " + std::to_string(this->_numberOfJoints) + " joints.");
     }
     else
     {
          while(link != nullptr)
          {
               unsigned int i = link->number();
          
               if(link->joint().is_revolute())
               {
                    // J_i = [ a_i x r_i ]
                    //       [    a_i    ]
               
                    J.block(0,i,3,1) = link->joint_axis().cross(point - link->pose().translation()); // Linear component
                    J.block(3,i,3,1) = link->joint_axis();                                           // Angular component
               }
               else // prismatic
               {
                    // J_i = [ a_i ]
                    //       [  0  ]
                    
                    J.block(0,i,3,1) = link->joint_axis();                                          // Linear component
                    J.block(3,i,3,1).setZero();                                                     // Angular component
               }
               
               link = link->parent_link();                                                          // Move to next joint toward the base     
          }
     }
     return J;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the time derivative of a given Jacobian                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 6, Eigen::Dynamic>
KinematicTree::time_derivative(const Eigen::Matrix<double,6,Eigen::Dynamic> &jacobianMatrix)
{

     // An algorithm for computing the time derivative is provided in this reference,
     // but it's not the same as this one:
     // Angeles, J. (2003) "Fundamentals of Robotic Mechanical Systems: Theory, Methods, and Algorithms"
     // Springer
     
     Eigen::Matrix<double,6,Eigen::Dynamic> Jdot(6,jacobianMatrix.cols());
     
     Jdot.setZero();

     for(int i = 0; i < jacobianMatrix.cols(); ++i)
     {
          for(int j = 0; j <= i; ++j)
          {
               // Compute dJ(i)/dq(j)
               if(this->_link[j]->joint().is_revolute())
               {
                    double qdot = this->_jointVelocity(j);                                          // Makes things a little easier
                    
                    // qdot_j * ( a_j x (a_i x r_i) )
                    Jdot(0,i) += qdot*(jacobianMatrix(4,j)*jacobianMatrix(2,i) - jacobianMatrix(5,j)*jacobianMatrix(1,i));
                    Jdot(1,i) += qdot*(jacobianMatrix(5,j)*jacobianMatrix(0,i) - jacobianMatrix(3,j)*jacobianMatrix(2,i));
                    Jdot(2,i) += qdot*(jacobianMatrix(3,j)*jacobianMatrix(1,i) - jacobianMatrix(4,j)*jacobianMatrix(0,i));
                    
                    if(this->_link[i]->joint().is_revolute())                                       // J_i = [a_i x r_i; a_i]
                    {
                         // qdot_j * ( a_j x a_i )
                         Jdot(3,i) += qdot*(jacobianMatrix(4,j)*jacobianMatrix(5,i) - jacobianMatrix(5,j)*jacobianMatrix(4,i));
                         Jdot(4,i) += qdot*(jacobianMatrix(5,j)*jacobianMatrix(3,i) - jacobianMatrix(3,j)*jacobianMatrix(5,i));
                         Jdot(5,i) += qdot*(jacobianMatrix(3,j)*jacobianMatrix(4,i) - jacobianMatrix(4,j)*jacobianMatrix(3,i));
                    }     
               }
               
               // Compute dJ(j)/dq(i)
               if(i != j and this->_link[j]->joint().is_revolute())                                 // J_j = [a_j x r_j; a_j]
               {
                    double qdot = this->_jointVelocity(i);                                          // Makes things a little easier
                    
                    if(this->_link[i]->joint().is_revolute())                                       // J_i = [a_i x r_i; a_i]
                    {
                         // qdot_i * ( a_i x (a_j x r_j) )
                         Jdot(0,j) += qdot*(jacobianMatrix(4,j)*jacobianMatrix(2,i) - jacobianMatrix(5,j)*jacobianMatrix(1,i));
                         Jdot(1,j) += qdot*(jacobianMatrix(5,j)*jacobianMatrix(0,i) - jacobianMatrix(3,j)*jacobianMatrix(2,i));
                         Jdot(2,j) += qdot*(jacobianMatrix(3,j)*jacobianMatrix(1,i) - jacobianMatrix(4,j)*jacobianMatrix(0,i));
                    }
                    else // this->link[i]->joint().is_prismatic()                                   // J_i = [a_i ; 0]
                    {
                         // qdot_i * ( a_i x (a_j x r_j) )
                         Jdot(0,j) += qdot*(jacobianMatrix(1,i)*jacobianMatrix(2,j) - jacobianMatrix(2,i)*jacobianMatrix(1,j));
                         Jdot(1,j) += qdot*(jacobianMatrix(2,i)*jacobianMatrix(0,j) - jacobianMatrix(0,i)*jacobianMatrix(2,j));
                         Jdot(2,j) += qdot*(jacobianMatrix(0,i)*jacobianMatrix(1,j) - jacobianMatrix(1,i)*jacobianMatrix(0,j));
                    }
               }
          }
     }
          
     return Jdot;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //             Get the partial derivative of a Jacobian with respect to a given joint            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double, 6, Eigen::Dynamic>
KinematicTree::partial_derivative(const Eigen::Matrix<double,6,Eigen::Dynamic> &jacobianMatrix,
                                  const unsigned int &jointNumber)
{

     // E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
     // Proceedings. 1991 IEEE International Conference on Robotics and Automation,
     // 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
     
     unsigned int numberOfColumns = jacobianMatrix.cols();
     
     if(jointNumber >= numberOfColumns)
     {
          throw std::invalid_argument("[ERROR] [KINEMATIC TREE] partial_derivative(): "
                                      "Cannot take the derivative with respect to joint number "
                                      + std::to_string(jointNumber) + " for a Jacobian with only "
                                      + std::to_string(numberOfColumns) + " columns.");
     }

     Eigen::Matrix<double,6,Eigen::Dynamic> dJ(6,numberOfColumns);                                  // Value to be returned
     dJ.setZero();
     
     int j = jointNumber;                                                                           // Makes things a little easier
     
     for(int i = 0; i < numberOfColumns-1; ++i)
     {
          if(this->_link[i]->joint().is_revolute())                                                 // J_i = [a_i x r_i ; a_i]
          {
               if(this->_link[j]->joint().is_revolute())                                            // J_i = [a_j x r_j; a_j]
               {
                    if (j < i)
                    {
                         // a_j x (a_i x a_i)
                         dJ(0,i) = jacobianMatrix(4,j)*jacobianMatrix(2,i) - jacobianMatrix(5,j)*jacobianMatrix(1,i);
                         dJ(1,i) = jacobianMatrix(5,j)*jacobianMatrix(0,i) - jacobianMatrix(3,j)*jacobianMatrix(2,i);
                         dJ(2,i) = jacobianMatrix(3,j)*jacobianMatrix(1,i) - jacobianMatrix(4,j)*jacobianMatrix(0,i);

                         // a_j x a_i
                         dJ(3,i) = jacobianMatrix(4,j)*jacobianMatrix(5,i) - jacobianMatrix(5,j)*jacobianMatrix(4,i);
                         dJ(4,i) = jacobianMatrix(5,j)*jacobianMatrix(3,i) - jacobianMatrix(3,j)*jacobianMatrix(5,i);
                         dJ(5,i) = jacobianMatrix(3,j)*jacobianMatrix(4,i) - jacobianMatrix(4,j)*jacobianMatrix(3,i);
                    }
                    else
                    {
                         // a_i x (a_j x a_j)
                         dJ(0,i) = jacobianMatrix(4,i)*jacobianMatrix(2,j) - jacobianMatrix(5,i)*jacobianMatrix(1,j);
                         dJ(1,i) = jacobianMatrix(5,i)*jacobianMatrix(0,j) - jacobianMatrix(3,i)*jacobianMatrix(2,j);
                         dJ(2,i) = jacobianMatrix(3,i)*jacobianMatrix(1,j) - jacobianMatrix(4,i)*jacobianMatrix(0,j);
                    }
               }
               else if(this->_link[j]->joint().is_prismatic() and j > i)                            // J_j = [a_j ; 0]
               {
                    // a_j x a_i
                    dJ(0,i) = jacobianMatrix(1,j)*jacobianMatrix(2,i) - jacobianMatrix(2,j)*jacobianMatrix(1,i);
                    dJ(1,i) = jacobianMatrix(2,j)*jacobianMatrix(0,i) - jacobianMatrix(0,j)*jacobianMatrix(2,i);
                    dJ(2,i) = jacobianMatrix(0,j)*jacobianMatrix(1,i) - jacobianMatrix(1,j)*jacobianMatrix(0,i);
               }
          }
          else if(this->_link[i]->joint().is_prismatic()                                            // J_i = [a_i ; 0]
              and this->_link[j]->joint().is_revolute()                                             // J_j = [a_j x r_j; a_j]
              and j < i)
          {
               // a_j x a_i
               dJ(0,i) = jacobianMatrix(4,j)*jacobianMatrix(2,i) - jacobianMatrix(5,j)*jacobianMatrix(1,i);
               dJ(1,i) = jacobianMatrix(5,j)*jacobianMatrix(0,i) - jacobianMatrix(3,j)*jacobianMatrix(2,i);
               dJ(2,i) = jacobianMatrix(3,j)*jacobianMatrix(1,i) - jacobianMatrix(4,j)*jacobianMatrix(0,i);
          }
     }
     
     return dJ;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Compute the Jacobian to a given frame                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,6,Eigen::Dynamic>
KinematicTree::jacobian(const std::string &frameName)
{
     return jacobian(find_frame(frameName));                                                        // Too easy lol ᕙ(▀̿̿ĺ̯̿̿▀̿ ̿) ᕗ
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the pose of a reference frame on the robot                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Pose
KinematicTree::frame_pose(const std::string &frameName)
{
     RobotLibrary::Model::ReferenceFrame *frame = find_frame(frameName);                            // Search the model
     
     return frame->link->pose()*frame->relativePose;                                                // Return pose relative to base/global frame
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Return a pointer to a link                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Link*
KinematicTree::link(const unsigned int &linkNumber)
{
   if(linkNumber >= this->_link.size())
   {
        throw std::runtime_error("[ERROR] [KINEMATIC TREE] link(): "
                                 "There are only " + std::to_string(this->_link.size()) + " in this model, "
                                 "but you tried to access link " + std::to_string(linkNumber+1) + ".");
   }
   return this->_link[linkNumber];
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Find a reference frame on the tree by name                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::ReferenceFrame*
KinematicTree::find_frame(const std::string &frameName)
{
   auto container = this->_frameList.find(frameName);                                               // Find the frame in the list
   
   if(container != this->_frameList.end()) return &container->second;                               // Return the ReferenceFrame struct
   else
   {    
        std::string message = "[ERROR] [KINEMATIC TREE] find_frame(): "
                              "Unable to find '" + frameName + "' in the model. "
                              "Here is a list of known reference frames:";
        
        for(auto frame = this->_frameList.begin(); frame != this->_frameList.end(); frame++)
        {
             message += "\n- " + frame->first;
        }
        
        throw std::runtime_error(message);
   }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Convert a char of numbers to an Eigen::Vector3d object                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d
KinematicTree::char_to_vector(const char* character)
{
    std::string numberAsString;                                                                     // So we can concatenate individual char together
    
    std::vector<double> numberAsVector;                                                             // Temporary storage

    int startPoint = 0;
                                                                     
    for(int i = 0; i < std::strlen(character); ++i)
    {
        if(character[i] == ' ')                                                                     // Find the space in the char array
        {
            for(int j = startPoint; j < i; ++j)
            {
                numberAsString += character[j];                                                     // Add the character to the string
            }
            
            numberAsVector.push_back(std::stod(numberAsString));                                    // Convert to double (override double)

            startPoint = i+1;                                                                       // Advance the start point

            numberAsString.clear();                                                                 // Clear the string to get the new number
        }
    }

    for(int i = startPoint; i < std::strlen(character); ++i) numberAsString += character[i];        // Get the last number in the char array

    numberAsVector.push_back(std::stof(numberAsString));

    return Eigen::Vector3d(numberAsVector.data());                                                  // Return the extracted data
}

} } // namespace
