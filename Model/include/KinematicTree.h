    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>                                                                                  // Custom class for describing a moveable connection between links
#include <Link.h>                                                                                   // Custom class combining a rigid body and joint
#include <map>                                                                                      // map
#include <tinyxml2.h>                                                                               // For parsing urdf files

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;                                                                                // std::string, std::map
using namespace tinyxml2;                                                                           // tinyxml2::XMLDocument, tinyxml2::XMLElement

template <typename DataType>
struct ReferenceFrame
{
	Link<DataType> *link = nullptr;                                                             // The link it is attached to
	Pose<DataType> relativePose;                                                                // The relative pose with respect to said link
};

template <class DataType>
class KinematicTree
{
	public:
		KinematicTree(const string &pathToURDF);                                            // Constructor from URDF

		// Methods
		
		bool update_state(const Vector<DataType, Dynamic> &jointPos,
		                  const Vector<DataType, Dynamic> &jointVel)
		{
			
			return update_state(jointPos, jointVel, this->_basePose, Vector<DataType,6>::Zero());
		}
		                  
		bool update_state(const Vector<DataType, Dynamic> &jointPosition,
		                  const Vector<DataType, Dynamic> &jointVelocity,
		                  const Pose<DataType>            &basePose,
		                  const Vector<DataType, 6>       &baseTwist);

		unsigned int number_of_joints() const { return this->numJoints; }
				        
		Matrix<DataType,Dynamic,Dynamic> joint_inertia_matrix() const { return this->jointInertiaMatrix; }
		
		Matrix<DataType,Dynamic,Dynamic> joint_coriolis_matrix() const { return this->jointCoriolisMatrix; } 

                Matrix<DataType,6,Dynamic> jacobian(const string &frameName); 
		                 
		Matrix<DataType,6,Dynamic> time_derivative(const Matrix<DataType, 6, Dynamic> &J); // Time derivative of a Jacobian
		
		Matrix<DataType,6,Dynamic> partial_derivative(const Matrix<DataType, 6, Dynamic> &J,
		                                                const unsigned int &jointNumber);   // Partial derivative of a Jacobian

		Pose<DataType> frame_pose(const string &frameName);                                 // Return the pose for a reference frame on the robot
				
		Vector<DataType,Dynamic> joint_gravity_vector() const { return this->jointGravityVector; }
		
		Vector<DataType,Dynamic> joint_velocities() const { return this->_jointVelocity; }
		
		string name() const { return this->_name; }                                         // Return the name of this robot
		
	private:
		
		// Properties

		map<string, ReferenceFrame<DataType>> referenceFrameList;
		
		Matrix<DataType, Dynamic, Dynamic> jointInertiaMatrix,
		                                   jointCoriolisMatrix;
		                                   
		Pose<DataType> _basePose;

		RigidBody<DataType> _base;
		
		unsigned int numJoints;

		Vector<DataType, 3> gravityVector = {0,0,-9.81};
		
		Vector<DataType, 6> _baseTwist;
			
		Vector<DataType, Dynamic> _jointPosition,
		                          _jointVelocity;

		Vector<DataType, Dynamic> jointGravityVector;
		
		vector<Link<DataType>> link;                                                        // Vector of link objects
		
		vector<Link<DataType>*> baseLinks;                                                  // Links attached directly to the base
		
		string _name;                                                                       // The name of this robot
		
		// Methods

		Matrix<DataType, 6, Dynamic> jacobian(Link<DataType> *link,
		                                      const Vector<DataType,3> &point,
		                                      const unsigned int &numberOfColumns);

		Vector<DataType, 3> char_to_vector(const char* character);                          // Used to parse urdf properties	  
                       
};                                                                                                  // Semicolon needed after class declarations

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
KinematicTree<DataType>::KinematicTree(const string &pathToURDF)
{
	// Variables used in this scope
	map<string, string>              connectionList;
	map<string, unsigned int>        linkList;
	map<string, RigidBody<DataType>> rigidBodyList;
	
	// Check to see if the file exists
	if(not ifstream(pathToURDF.c_str()))
	{
		throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
		                    "The file " + pathToURDF + " does not appear to exist.");
	}

	XMLDocument urdf; urdf.LoadFile(pathToURDF.c_str());                                        // Load URDF           
	
	// Make sure a robot is defined
	XMLElement* robot = urdf.FirstChildElement("robot");
	
	if(robot == nullptr)
	{
		throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
		                    "There does not appear to be a 'robot' element in the URDF.");
	}
	
	this->_name = robot->Attribute("name");                                                     // Assign the name
	
	// Search through every 'link' in the urdf file and convert it to RobotLibrary::RigidBody
	for(auto iterator = robot->FirstChildElement("link"); iterator; iterator = iterator->NextSiblingElement("link"))
	{
		// Default properties
		DataType mass = 0.0;
		string name = iterator->Attribute("name");
		Vector<DataType,3> centreOfMass = {0,0,0};
		Matrix<DataType,3,3> momentOfInertia; momentOfInertia.setZero();
		
		// Get the inertia properties if they exist
		XMLElement *inertial = iterator->FirstChildElement("inertial");
		
		if(inertial != nullptr)
		{
			// Need to force a conversion here if DataType == float
			mass = (DataType)inertial->FirstChildElement("mass")->DoubleAttribute("value");

			DataType ixx = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("ixx");
			DataType ixy = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("ixy");
			DataType ixz = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("ixz");
			DataType iyy = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("iyy");
			DataType iyz = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("iyz");
			DataType izz = (DataType)inertial->FirstChildElement("inertia")->DoubleAttribute("izz");
			
			momentOfInertia << ixx, ixy, ixz,
			                   ixy, iyy, iyz,
			                   ixz, iyz, izz;
			
			XMLElement* origin = inertial->FirstChildElement("origin");
			
			if(origin != nullptr)
			{
				Vector<DataType, 3> xyz = char_to_vector(origin->Attribute("xyz"));
				Vector<DataType, 3> rpy = char_to_vector(origin->Attribute("rpy"));
				
				// NOTE: URDF specifies a pose for the center of mass,
				//       which is a little superfluous...
				
				Pose<DataType> pose(xyz, AngleAxis<DataType>(rpy(0),Vector<DataType,  3>::UnitX())
				                        *AngleAxis<DataType>(rpy(1),Vector<DataType,  3>::UnitY())

				                        *AngleAxis<DataType>(rpy(2),Vector<DataType,  3>::UnitZ()));
				                                       

				centreOfMass = pose.position();                                     // Location for the center of mass
				
				Matrix<DataType,3,3> R = pose.rotation();                           // Get the SO(3) matrix
				
				momentOfInertia = R*momentOfInertia*R.transpose();                  // Rotate the inertia to the origin frame
			}
		}
	
		rigidBodyList.emplace(name, RigidBody<DataType>(name,mass,momentOfInertia,centreOfMass)); // Put it in the list so we can find it later
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
			DataType damping          = 1.0;
			DataType effortLimit      = 10;
			DataType friction         = 0.0;
			DataType positionLimit[]  = {-M_PI, M_PI};
			DataType velocityLimit    = 100*2*M_PI/60;
			Vector<DataType,  3> axis = {0,0,1};
			Pose<DataType> origin(Vector<DataType,3>::Zero(), Quaternion<DataType>(1,0,0,0));
			
			// Get the pose of the joint relative to preceeding link
			XMLElement* originElement = iterator->FirstChildElement("origin");

			if(originElement != nullptr)
			{
				Vector<DataType,3> xyz = char_to_vector(originElement->Attribute("xyz"));
				Vector<DataType,3> rpy = char_to_vector(originElement->Attribute("rpy"));

				origin = Pose<DataType>(xyz, AngleAxis<DataType>(rpy(2),Vector<DataType,3>::UnitZ())
						       *AngleAxis<DataType>(rpy(1),Vector<DataType,3>::UnitY())
						       *AngleAxis<DataType>(rpy(0),Vector<DataType,3>::UnitX()));
			}

			// Get joint properties for non-fixed type
			if(jointType != "fixed")
			{	
				// Get the joint limits	
				if(jointType == "continuous")                                       // No limits on the joints
				{
					DataType inf = numeric_limits<DataType>::infinity();
					
					positionLimit[0] =-inf;
					positionLimit[1] = inf;
					velocityLimit    = inf;
					effortLimit      = inf;
				}
				else                                                                // Obtain the joint limits
				{
					XMLElement* limit = iterator->FirstChildElement("limit");
					if(limit != nullptr)
					{
						positionLimit[0] = (DataType)limit->DoubleAttribute("lower");
						positionLimit[1] = (DataType)limit->DoubleAttribute("upper");
						velocityLimit    = (DataType)limit->DoubleAttribute("velocity"); // NOTE: This is apparently optional so could cause problems in the future
						effortLimit      = (DataType)limit->DoubleAttribute("effort");
					}
				}
				
				// Get the axis of actuation					
				XMLElement* axisElement = iterator->FirstChildElement("axis");
				if(axisElement != nullptr) axis = char_to_vector(axisElement->Attribute("xyz"));
				
				// Get the dynamic properties
				XMLElement* dynamics = iterator->FirstChildElement("dynamics");
				if(dynamics != nullptr)
				{
					damping  = (DataType)dynamics->DoubleAttribute("damping");
					friction = (DataType)dynamics->DoubleAttribute("friction");
				}
			}
			
			Joint<DataType> tempJoint(jointName, jointType, axis, origin, positionLimit, velocityLimit, effortLimit, damping, friction);
			
			// Find the "child link", i.e. RigidBody object, and form a RobotLibrary::Link object
			
			string childLinkName = iterator->FirstChildElement("child")->Attribute("link");
			
			auto container = rigidBodyList.find(childLinkName);
			
			if(container == rigidBodyList.end())
			{
				throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
                                                    "Unable to find '" + childLinkName + "' in the list of rigid bodies.");
                        }
                        
                        this->link.emplace_back(container->second, tempJoint);                      // Create new link, add to vector
                        
			linkList.emplace(childLinkName, linkNumber);                                // Add the index to the std::map so we can find it later
			
			linkNumber++;                                                               // Iterate the link/joint number
			
			rigidBodyList.erase(childLinkName);                                         // Delete the rigid body from the list
			
			// Get the parent link name, store it to search later
			
			string parentLinkName = iterator->FirstChildElement("parent")->Attribute("link");
			
			connectionList.emplace(childLinkName, parentLinkName);                      // Store this so we can search later
		}
	}
	
	// Find and set the base
	if(rigidBodyList.size() == 1)
	{
		auto container = rigidBodyList.begin();
		this->_base = container->second;
	}
	else
	{
		string message = "[ERROR] [KINEMATIC TREE] Constructor: More than one candidate for the base:\n";
		for(auto iterator = rigidBodyList.begin(); iterator != rigidBodyList.end(); iterator++) message + "   - " + iterator->first + "\n";
		throw std::runtime_error(message);
	}
	
	// Form the connections between all the links
	for(int i = 0; i < this->link.size(); i++)
	{
		// Get the name for the parent of this link
		auto container = connectionList.find(this->link[i].name());
		
		if(container == connectionList.end())
		{
			throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
			                    "Could not find the '" + this->link[i].name() + "' link in the connection list for some reason.");
		}
			
		std::string parentLinkName = container->second;
		
		// Set the parent and child
		if(parentLinkName != this->_base.name())
		{
			auto otherContainer = linkList.find(parentLinkName);
			
			if(otherContainer == linkList.end())
			{
				throw runtime_error("[ERROR] [KINEMATIC TREE] Constructor: "
					            "Could not find the '" + parentLinkName + "' link in the link list");
			}
			
			unsigned int j = otherContainer->second;                                    // Get the number
			
			this->link[i].set_parent_link(&this->link[j]);                              // Set parent/child relationship
		}
	}
	
	unsigned int prevNumJoints = this->link.size();
	unsigned int count = 0;
	
	// Merge the dynamic properties of all the links connected by fixed joints
	for(int i = 0; i < this->link.size(); i++)
	{
		if(this->link[i].joint.is_fixed())
		{     
			Link<DataType> *parentLink = this->link[i].parent_link();                        // Pointer to the parent link of this one
			          
			if(parentLink == nullptr)                                                        // It must be the base!
			{
				this->_base.combine_inertia(this->link[i],this->link[i].joint.offset()); // Combine the inertia of this link with the base
				
				std::vector<Link<DataType>*> childLinks = this->link[i].child_links();   // Get all the links attached to this one
				          
				for(int j = 0; j < childLinks.size(); j++)                               // Get all the links following this one
				{
					childLinks[j]->clear_parent_link();                              // Clear parent; it is attached to base
					
					this->baseLinks.push_back(childLinks[j]);                        // Add this to list of links attached to base
				}
			}
			else parentLink->merge(this->link[i]);                                           // Merge this link in to the other
		
			// Save this link as a reference frame so we can search it later
			ReferenceFrame<DataType> frame;
			frame.link = parentLink;                                                         // The frame is on the parent link
			frame.relativePose = this->link[i].joint.offset();	                         // Pose of the frame relative to parent link
			this->referenceFrameList.emplace(this->link[i].name(), frame);	                 // Add to list
		}
		else
		{
			this->link[i].set_number(count);                                                 // Give this link/joint a number
			count++;                                                                         // Count up the number of 
		}
	}
	
	this->numJoints = count;
	
	// Resize the relevant matrices, vectors accordingly
	this->_jointPosition.resize(this->numJoints);
	this->_jointVelocity.resize(this->numJoints);
	this->jointInertiaMatrix.resize(this->numJoints, this->numJoints);
	this->jointCoriolisMatrix.resize(this->numJoints, this->numJoints);
	this->jointGravityVector.resize(this->numJoints);
	
	std::cout << "[INFO] [KINEMATIC TREE] Successfully created the '" << this->_name << "' robot."
	          << " It has " << this->numJoints << " joints (reduced from " << prevNumJoints << ").\n";
	
/*	Debugging stuff:
	std::cout << "\nHere is a list of all the joints on the robot:\n";		
	vector<Link<DataType>*> candidateList = this->baseLinks;                                         // Start with links connect to base
	while(candidateList.size() > 0)
	{
		Link<DataType> *currentLink = candidateList.back();                                      // Get the one at the end
		
		candidateList.pop_back();                                                                // Delete it from the search list
		
		std::cout << " - " << currentLink->joint.name() << std::endl;
		
		vector<Link<DataType>*> temp = currentLink->child_links();                               // All the links attached to the current link
		
		if(temp.size() != 0) candidateList.insert(candidateList.end(), temp.begin(), temp.end());
	}
	
	std::cout << "\nHere are the reference frames on the robot:\n";
	for(auto iterator = this->referenceFrameList.begin(); iterator != this->referenceFrameList.end(); iterator++)
	{	
		std::cout << " - '" << iterator->first << "' is on the ";
		
		if(iterator->second.link == nullptr) std::cout << this->_base.name() << ".\n";
		else                                 std::cout << "'" << iterator->second.link->name() << "' link.\n";
	}
*/
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Update the kinematics and dynamics                               //        
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool KinematicTree<DataType>::update_state(const Vector<DataType, Dynamic> &jointPosition,
                                           const Vector<DataType, Dynamic> &jointVelocity,
                                           const Pose<DataType>            &basePose,
                                           const Vector<DataType, 6>       &baseTwist)
{
	if(jointPosition.size() != this->numJoints
	or jointVelocity.size() != this->numJoints)
	{
		cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
		     << "This model has " << this->numJoints << " joints, but "
		     << "the joint position argument had " << jointPosition.size() << " elements, "
		     << "and the joint velocity argument had " << jointVelocity.size() << " element.\n";
		
		return false;
	}
	
	this->_jointPosition = jointPosition;
	this->_jointVelocity = jointVelocity;
	
	this->_basePose  = basePose;
	this->_baseTwist = baseTwist;
	
	// Need to clear for new loop
	this->jointInertiaMatrix.setZero();
	this->jointCoriolisMatrix.setZero();
	this->jointGravityVector.setZero();
	
	vector<Link<DataType>*> candidateList = this->baseLinks;                                    // Start with links attached to base
	
	while(candidateList.size() > 0)
	{
		Link<DataType> *currentLink = candidateList.back();                                 // Get last link in the list...
		
		candidateList.pop_back();                                                           // ... and remove it

		unsigned int k = currentLink->number();                                             // The number of this link/joint
		
		Link<DataType> *parentLink = currentLink->parent_link();                            // Get the parent of this one
	
		Pose<DataType> previousPose;
		
		if(parentLink == nullptr) previousPose = this->_basePose;
		else                      previousPose = parentLink->pose();
		
		// Propagate the forward kinematics
		currentLink->update_state(previousPose, jointPosition(k));
		
		// Get the links attached to this one and add them to the list
		
		vector<Link<DataType>*> temp = currentLink->child_links();
		
		if(temp.size() > 0) candidateList.insert(candidateList.begin(), temp.begin(), temp.end());
		
		// Debugging:
		if(parentLink == nullptr) std::cout << this->_base.name();
		else                      std::cout << parentLink->name();
		
		std::cout << " <--- (" << currentLink->joint.name() << ") <--- " << currentLink->name() << std::endl;
		
		std::cout << std::endl;
		
		std::cout << currentLink->pose().as_matrix() << std::endl;
		
				std::cout << std::endl;
		
	}
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Matrix<DataType, 6, Dynamic>
KinematicTree<DataType>::jacobian(Link<DataType> *link,                                             // Starting point in the kinematic tree
                                  const Vector<DataType, 3> &point,                                 // A point in the given joint frame
                                  const unsigned int &numberOfColumns)                              // Must be less than or equal to number of joints in model
{
/*
	Matrix<DataType, Dynamic, Dynamic> J(6,numberOfColumns); J.setZero();                       // Value to be returned
	
	if(joint->number() > numberOfColumns)
	{
		throw runtime_error("[ERROR] [KINEMATIC TREE] jacobian(): "
		                    "Cannot compute a Jacobian with " + to_string(numberOfColumns) +
		                    "columns starting from joint number " + to_string(joint->number()+1) + ").");
	}
	else
	{
		Joint *currentJoint = joint;                                                        // Used to iterate toward the base
				
		while(currentJoint != nullptr)
		{
			unsigned int i = currentJoint->number();
		
			if(currentJoint->is_revolute())
			{
				// J_i = [ a_i x r_i ]
				//       [    a_i    ]
			
				J.block(0,i,3,1) = currentJoint->axis().cross(point - currentJoint->pose().position()); // Linear component
				J.block(3,i,3,1) = currentJoint->axis();                                                // Angular component
			}
			else // prismatic
			{
				// J_i = [ a_i ]
				//       [  0  ]
				
				J.block(0,i,3,1) = currentJoint->axis();                            // Linear component
				J.block(3,i,3,1).setZero();                                         // Angular component
			}
			
			currentJoint = currentJoint->parent_link()->parent_joint();                 // Move to next joint toward the base	
		}
	}
	
	return J;
*/
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the time derivative of a given Jacobian                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Matrix<DataType, 6, Dynamic>
KinematicTree<DataType>::time_derivative(const Matrix<DataType, 6, Dynamic> &J)
{	
/*
	Matrix<DataType, Dynamic, Dynamic> Jdot = Eigen::Matrix<DataType,6,Dynamic>::Zero();

	for(int i = 0; i < J.cols(); i++)
	{
		for(int j = 0; j <= i; j++)
		{
			// Compute dJ(i)/dq(j)
			if(this->joint[j].is_revolute())
			{
				DataType qdot = this->_jointVelocity(j);                            // Makes things a little easier
				
				// qdot_j * ( a_j x (a_i x r_i) )
				Jdot(0,i) += qdot*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
				Jdot(1,i) += qdot*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
				Jdot(2,i) += qdot*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				
				if(this->joint[i].is_revolute())			            // J_i = [a_i x r_i; a_i]
				{
					// qdot_j * ( a_j x a_i )
					Jdot(3,i) += qdot*(J(4,j)*J(5,i) - J(5,j)*J(4,i));
					Jdot(4,i) += qdot*(J(5,j)*J(3,i) - J(3,j)*J(5,i));
					Jdot(5,i) += qdot*(J(3,j)*J(4,i) - J(4,j)*J(3,i));
				}	
			}
			
			// Compute dJ(j)/dq(i)
			if(i != j && this->joint[j].is_revolute())			            // J_j = [a_j x r_j; a_j]
			{
				DataType qdot = this->_jointVelocity(i);                            // Makes things a little easier
				
				if(this->joint[i].is_revolute())			            // J_i = [a_i x r_i; a_i]
				{
					// qdot_i * ( a_i x (a_j x r_j) )
					// Jdot(0,j) += this->qdot(i)*(J(4,i)*J(2,j) - J(5,i)*J(1,j));
					// Jdot(1,j) += this->qdot(i)*(J(5,i)*J(0,j) - J(3,i)*J(2,j));
					// Jdot(2,j) += this->qdot(i)*(J(3,i)*J(1,j) - J(4,i)*J(0,j));
					Jdot(0,j) += qdot*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
					Jdot(1,j) += qdot*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
					Jdot(2,j) += qdot*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				}
				else // this->link[i].is_prismatic()			            // J_i = [a_i ; 0]
				{
					// qdot_i * ( a_i x (a_j x r_j) )
					Jdot(0,j) += qdot*(J(1,i)*J(2,j) - J(2,i)*J(1,j));
					Jdot(1,j) += qdot*(J(2,i)*J(0,j) - J(0,i)*J(2,j));
					Jdot(2,j) += qdot*(J(0,i)*J(1,j) - J(1,i)*J(0,j));
				}
			}
		}
	}
	
	return Jdot;
*/
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //             Get the partial derivative of a Jacobian with respect to a given joint            //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Matrix<DataType, 6, Dynamic>
KinematicTree<DataType>::partial_derivative(const Matrix<DataType, 6, Dynamic> &J,
                                            const unsigned int &jointNumber)
{
/*
	// E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
	// Proceedings. 1991 IEEE International Conference on Robotics and Automation,
	// 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
	
	if(jointNumber >= J.cols())
	{
		throw invalid_argument("[ERROR] [KINEMATIC TREE] partial_derivative(): "
		                       "Joint number must be strictly less than the number of columns "
		                       "(" + to_string(jointNumber) + " >= "+ to_string(J.cols()) + ").");
	}

	Matrix<DataType, 6, Dynamic> dJ(6,J.cols()); dJ.setZero();                                  // Value to be returned
	
	int j = jointNumber;                                                                        // Makes things a little easier
	
	for(int i = 0; i < J.cols(); i++)
	{
		if(this->joint[i].is_revolute())					            // J_i = [a_i x r_i ; a_i]
		{
			if(this->joint[j].is_revolute()) 			                    // J_i = [a_j x r_j; a_j]
			{
				if (j < i)
				{
					// a_j x (a_i x a_i)
					dJ(0,i) = J(4,j)*J(2,i) - J(5,j)*J(1,i);
					dJ(1,i) = J(5,j)*J(0,i) - J(3,j)*J(2,i);
					dJ(2,i) = J(3,j)*J(1,i) - J(4,j)*J(0,i);

					// a_j x a_i
					dJ(3,i) = J(4,j)*J(5,i) - J(5,j)*J(4,i);
					dJ(4,i) = J(5,j)*J(3,i) - J(3,j)*J(5,i);
					dJ(5,i) = J(3,j)*J(4,i) - J(4,j)*J(3,i);
				}
				else
				{
					// a_i x (a_j x a_j)
					dJ(0,i) = J(4,i)*J(2,j) - J(5,i)*J(1,j);
					dJ(1,i) = J(5,i)*J(0,j) - J(3,i)*J(2,j);
					dJ(2,i) = J(3,i)*J(1,j) - J(4,i)*J(0,j);
				}
			}
			else if(this->joint[j].is_prismatic() and j > i)	                    // J_j = [a_j ; 0]
			{
				// a_j x a_i
				dJ(0,i) = J(1,j)*J(2,i) - J(2,j)*J(1,i);
				dJ(1,i) = J(2,j)*J(0,i) - J(0,j)*J(2,i);
				dJ(2,i) = J(0,j)*J(1,i) - J(1,j)*J(0,i);
			}
		}
		else if(this->joint[i].is_prismatic()					            // J_i = [a_i ; 0]
		    and this->joint[j].is_revolute()				                    // J_j = [a_j x r_j; a_j]
	            and j < i)
		{
			// a_j x a_i
			dJ(0,i) = J(4,j)*J(2,i) - J(5,j)*J(1,i);
			dJ(1,i) = J(5,j)*J(0,i) - J(3,j)*J(2,i);
			dJ(2,i) = J(3,j)*J(1,i) - J(4,j)*J(0,i);
		}
	}
	
	return dJ;
*/
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Compute the Jacobian to a given frame                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Matrix<DataType, 6, Dynamic>
KinematicTree<DataType>::jacobian(const string &frameName)
{
/*
    auto container = this->referenceFrameList.find(frameName);

    if(container == this->referenceFrameList.end())
    {
        throw runtime_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                            "Could not find the '" + frameName + "' frame in the list.");
    }
    else
    {
        Pose<DataType> framePose = container->second.link->parent_joint()->pose()                   // Pose for the *origin* of the associated link
                                 * container->second.relativePose;                                  // Pose of this frame relative to link/joint
        
        return jacobian(container->second.link->parent_joint(),
                        framePose.position(),
                        container->second.link->parent_joint()->number()+1);
    }
*/
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the a reference frame on the robot                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> KinematicTree<DataType>::frame_pose(const string &frameName)
{
	auto container = this->referenceFrameList.find(frameName);
	
	if(container == this->referenceFrameList.end())
	{
		throw runtime_error("[ERROR] [KINEMATIC TREE] get_reference_frame(): "
		                    "Could not find '" + frameName + "' in the list.");
	}
	else
	{
		return container->second.link->pose()                                               // Pose for the *origin* of the associated link
		      *container->second.relativePose;                                              // Pose of the frame relative to the link/joint
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Convert a char of numbers to an Vector<DataType,  3> object                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Vector<DataType,3>
KinematicTree<DataType>::char_to_vector(const char* character)
{
	string numberAsString;                                                                      // So we can concatenate individual char together
	vector<DataType> numberAsVector;                                                            // Temporary storage
	
	int startPoint = 0;                                                                         
	for(int i = 0; i < strlen(character); i++)
	{
		if(character[i] == ' ')                                                             // Find the space in the char array
		{
			for(int j = startPoint; j < i; j++) numberAsString += character[j];         // Add the character to the string
				
			numberAsVector.push_back(stof(numberAsString));                             // Convert the string to a DataType, put in vector
			
			startPoint = i+1;                                                           // Advance the start point
			
			numberAsString.clear();                                                     // Clear the string to get the new number
		}
	}
	
	// Get the last number in the char array
	for(int i = startPoint; i < strlen(character); i++) numberAsString += character[i];
	
	numberAsVector.push_back(stof(numberAsString));

	return Vector<DataType,3>(numberAsVector.data());                                           // Return the extracted data
}

#endif
