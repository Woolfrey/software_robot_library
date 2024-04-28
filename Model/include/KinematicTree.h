/**
 * @file   KinematicTree.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class representing multiple rigid bodies connected in series by actuated joints.
 */
 
#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>                                                                                  // Custom class for describing a moveable connection between links
#include <Link.h>                                                                                   // Custom class combining a rigid body and joint
#include <map>                                                                                      // map
#include <Math.h>                                                                                   // Useful functions
#include <tinyxml2.h>                                                                               // For parsing urdf files

template <typename DataType>
struct ReferenceFrame
{
	Link<DataType> *link = nullptr;                                                                // The link it is attached to
	Pose<DataType> relativePose;                                                                   // Pose with respect to local link frame
};

template <class DataType>
class KinematicTree
{
	public:
		/**
		 * Constructor for a kinematic tree.
		 * @param pathToURDF The location of a URDF file that specifies are robot structure.
		 */
		KinematicTree(const std::string &pathToURDF);                                             // Constructor from URDF
		
		/**
		 * Updates the forward kinematics and inverse dynamics. Used for fixed base structures.
		 * @param jointPosition A vector of the joint positions.
		 * @param jointVelocity A vector of the joint velocities.
		 * @return Returns false if there is a problem.
		 */
		bool update_state(const Eigen::Vector<DataType, Eigen::Dynamic> &jointPosition,
		                  const Eigen::Vector<DataType, Eigen::Dynamic> &jointVelocity)
		{
			return update_state(jointPosition, jointVelocity, this->base.pose(), Eigen::Vector<DataType,6>::Zero());
		}
		
		/**
		 * Updates the forward kinematics and inverse dynamics. Used for floating base structures.
		 * @param jointPosition A vector of all the joint positions.
		 * @param jointVelocity A vector of all the joint velocities.
		 * @param basePose The transform of the base relative to some global reference frame.
		 * @param baseTwist The velocity of the base relative to some global reference frame.
		 */
		bool update_state(const Eigen::Vector<DataType, Eigen::Dynamic> &jointPosition,
		                  const Eigen::Vector<DataType, Eigen::Dynamic> &jointVelocity,
		                  const Pose<DataType>                          &basePose,
		                  const Eigen::Vector<DataType, 6>              &baseTwist);

		/**
		 * Query how many controllable joints there are in this model.
		 * @return Returns what you asked for.
		 */
		unsigned int number_of_joints() const { return this->_numberOfJoints; }
		
		/**
		 * Get the coupled inertia matrix between the actuated joints and the base.
		 * @return An nx6 Eigen::Matrix object.
		 */
		Eigen::Matrix<DataType, Eigen::Dynamic, 6>
		joint_base_inertia_matrix() const { return this->_jointBaseInertiaMatrix; }
		
		/**
		 * Get the coupled inertia matrix between the base and actuated joints.
		 * @return A 6xn Eigen::Matrix object.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic>
		base_joint_inertia_matrix() const { return this->_jointBaseInertiaMatrix.transpose(); }
		
		/**
		 * Get the Coriolis matrix pertaining to coupled inertia between the actuated joints and base.
		 * @return An nx6 Eigen::Matrix object.
		 */
		// Should be centripetal forces only???
		Eigen::Matrix<DataType, Eigen::Dynamic, 6>
		joint_base_coriolis_matrix() const { return this->_jointBaseCoriolisMatrix; }
		
		/**
		 * Get the Coriolis matrix pertaining to coupled inertia between the base and actuated joints.
		 * @return A 6xn Eigen::Matrix object.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic>
		base_joint_coriolis_matrix() const { return -this->_jointBaseCoriolisMatrix.transpose(); }
		
		/**
		 * Get the inertia matrix in the joint space of the model / robot.
		 * @return Returns an nxn Eigen::Matrix object.
		 */	        
		Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
		joint_inertia_matrix() const { return this->_jointInertiaMatrix; }
		
		/**
		 * Get the matrix pertaining to centripetal and Coriolis torques in the joints of the model.
		 * @return Returns an nxn Eigen::Matrix object.
		 */
		Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
		joint_coriolis_matrix() const { return this->_jointCoriolisMatrix; } 

		/**
		 * Get the matrix that maps joint motion to Cartesian motion of the specified frame.
		 * @return Returns a 6xn Eigen::Matrix object.
		 */
          Eigen::Matrix<DataType, 6, Eigen::Dynamic> jacobian(const std::string &frameName); 

		/**
		 * Compute the time derivative for a given Jacobian matrix.
		 * @param J The Jacobian for which to take the time derivative.
		 * @return A 6xn matrix for the time derivative.
		 */
		Eigen::Matrix<DataType, 6, Eigen::Dynamic>
		time_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix);
		
		/**
		 * Compute the partial derivative for a Jacobian with respect to a given joint.
		 * @param J The Jacobian with which to take the derivative
		 * @param jointNumber The joint (link) number for which to take the derivative.
		 * @return A 6xn matrix for the partial derivative of the Jacobian.
		 */
		Eigen::Matrix<DataType, 6, Eigen::Dynamic>
		partial_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix,
		                   const unsigned int &jointNumber);

		/**
		 * Get the pose of a specified reference frame on the kinematic tree.
		 * @return Returns a RobotLibrary::Pose object.
		 */
		Pose<DataType> frame_pose(const std::string &frameName);
			
		/**
		 * Get the joint torques needed to oppose gravitational acceleration.
		 * @return Returns an nx1 Eigen::Vector object.
		 */	
		Eigen::Vector<DataType,Eigen::Dynamic> joint_gravity_vector() const { return this->_jointGravityVector; }
		
		/** 
		 * Get the current joint velocities of all the joints in the model.
		 * @return Returns an nx1 Eigen::Vector object.
		 */
		Eigen::Vector<DataType,Dynamic> joint_velocities() const { return this->_jointVelocity; }
		
		/**
		 * Get the name of this model.
		 * @return Returns a std::string object.
		 */
		std::string name() const { return this->_name; }
		
		/**
		 * Returns a pointer to a reference frame on this model.
		 * @param name In the URDF, the name of the parent link attached to a fixed joint
		 * @return A ReferenceFrame data structure.
		 */
		ReferenceFrame<DataType>* find_frame(const std::string &frameName)
		{
               auto container = this->_frameList.find(frameName);                                   // Find the frame in the list
               
               if(container != this->_frameList.end()) return &container->second;                    // Return the ReferenceFrame struct
               else
               {    
                    throw std::runtime_error("[ERROR] [KINEMATIC TREE] find_frame(): "
                                             "Unable to find the frame '" + frameName + "' in the model.");
               }
          }
          
          /**
           * Get the joint position vector in the underlying model.
           * @return An nx1 Eigen::Vector object of all the joint positions.
           */
          Eigen::Vector<DataType,Eigen::Dynamic> joint_positions() const { return this->_jointPosition; }
          
          /**
           * Return a pointer to a link on the structure.
           * @param The number of the link in the model.
           * @return A RobotLibrary::Link object
           */
          Link<DataType>* link(const unsigned int &linkNumber)
          {
               if(linkNumber >= this->_link.size())
               {
                    throw std::runtime_error("[ERROR] [KINEMATIC TREE] link(): "
                                             "There are only " + std::to_string(this->_link.size()) + " in this model, "
                                             "but you requested " + std::to_string(linkNumber+1) + ".");
               }
               return this->_link[linkNumber];
          }

		RigidBody<DataType> base;                                                                 ///< Specifies the dynamics for the base.
		
	private:
		
		Eigen::Matrix<DataType,Eigen::Dynamic,6> _jointBaseCoriolisMatrix;                        ///< Inertial coupling between base and links
		
		Eigen::Matrix<DataType,Eigen::Dynamic,6> _jointBaseInertiaMatrix;                         ///< Inertial coupling between base and links
		
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _jointCoriolisMatrix;               ///< As it says on the label.
		
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _jointInertiaMatrix;                ///< As it says on the label.

		Eigen::Vector<DataType, 3> _gravityVector = {0,0,-9.81};                                  ///< 3x1 vector for the gravitational acceleration.
			
		Eigen::Vector<DataType,Eigen::Dynamic> _jointPosition;                                    ///< A vector of all the joint positions.

		Eigen::Vector<DataType,Eigen::Dynamic> _jointVelocity;                                    ///< A vector of all the joint velocities.

		Eigen::Vector<DataType,Eigen::Dynamic> _jointGravityVector;                               ///< A vector of all the gravitational joint torques.
		 
		std::map<std::string, ReferenceFrame<DataType>> _frameList;                               ///< A dictionary of reference frames on the kinematic tree.
		
		std::vector<Link<DataType>> _fullLinkList;                                                ///< An array of all the links in the model, including fixed joints.
		
		std::vector<Link<DataType>*> _link;                                                       ///< An array of all the actuated links.
		
		std::vector<Link<DataType>*> _baseLinks;                                                  ///< Array of links attached directly to the base.
		
		std::string _name;                                                                        ///< A unique name for this model.
		
		unsigned int _numberOfJoints;                                                             ///< The number of actuated joint in the kinematic tree.
		
		/**
		 * Compute a matrix that relates joint motion to Cartesian motion for a frame on the robot.
		 * @param frame A pointer to the reference frame on the model.
		 * @return A 6xn Eigen::Matrix object.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic> jacobian(ReferenceFrame<DataType> *frame)
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
                                    this->_numberOfJoints+1);
               }
          }
          
		/**
		 * Computes the Jacobian to a given point on a given link.
		 * @param link A pointer to the link for the Jacobian
		 * @param point A point relative to the link with which to compute the Jacobian
		 * @param numberOfColumns Number of columns for the Jacobian (can be used to speed up calcs)
		 * @return A 6xn Jacobian matrix.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic>
		jacobian(Link<DataType> *link,
		         const Eigen::Vector<DataType,3> &point,
		         const unsigned int &numberOfColumns);

		/**
		 * Converts a char array to a 3x1 vector. Used in the constructor.
		 * @param character A char array
		 * @return Returns a 3x1 Eigen vector object.
		 */
		Eigen::Vector<DataType,3> char_to_vector(const char* character);  
                       
};                                                                                                  // Semicolon needed after class declarations

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
KinematicTree<DataType>::KinematicTree(const std::string &pathToURDF)
{
	using namespace Eigen;                                                                         // Eigen::Matrix, Eigen::Vector
	using namespace std;                                                                           // std::string, std::map
	using namespace tinyxml2;                                                                      // tinyxml2::XMLDocument, tinyxml2::XMLElement

	// Variables used in this scope
	map<string, string>              connectionList;                                               // For storing parent/child connection between links
	map<string, unsigned int>        linkList;                                                     // Search for links by name
	map<string, RigidBody<DataType>> rigidBodyList;                                                // Store urdf links / RigidBody objects
	
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
				//       which is a little superfluous, so we will reduce it
				
				Pose<DataType> pose(xyz, AngleAxis<DataType>(rpy(0),Vector<DataType, 3>::UnitX())
				                        *AngleAxis<DataType>(rpy(1),Vector<DataType, 3>::UnitY())
				                        *AngleAxis<DataType>(rpy(2),Vector<DataType, 3>::UnitZ()));
				                                       

				centreOfMass = pose.translation();                                              // Location for the center of mass
				
				Matrix<DataType,3,3> R = pose.rotation();                                       // Get the SO(3) matrix
				
				momentOfInertia = R*momentOfInertia*R.transpose();                              // Rotate the inertia to the origin frame
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
			DataType damping               = 1.0;
			DataType effortLimit           = 10;
			DataType friction              = 0.0;
			Limits<DataType> positionLimit = {-M_PI, M_PI};                                      // Data structure in Joint.h
			DataType velocityLimit         = 100*2*M_PI/60;
			Vector<DataType,3> axis        = {0,0,1};
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
				if(jointType == "continuous")                                                   // No limits on the joints
				{ 
					DataType inf = numeric_limits<DataType>::infinity();
					
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
						positionLimit.lower = (DataType)limit->DoubleAttribute("lower");
						positionLimit.upper = (DataType)limit->DoubleAttribute("upper");
						velocityLimit       = (DataType)limit->DoubleAttribute("velocity");   // NOTE: This is apparently optional so could cause problems in the future
						effortLimit         = (DataType)limit->DoubleAttribute("effort");
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
			
			Joint<DataType> joint(jointName, jointType, axis, origin, positionLimit, velocityLimit, effortLimit, damping, friction);
			
			// Find the "child link", i.e. RigidBody object, and form a RobotLibrary::Link object
			
			string childLinkName = iterator->FirstChildElement("child")->Attribute("link");
                        
                        const auto &rigidBody = rigidBodyList.find(childLinkName)->second;
			                       
                        this->_fullLinkList.emplace_back(rigidBody, joint);                         // Create new link, add to vector
                        
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
		this->base = rigidBodyList.begin()->second;
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
			Link<DataType> *parentLink = currentLink.parent_link();                              // Pointer to the parent link
		
			if(parentLink == nullptr)                                                            // No parent link, must be attached to the base!
			{
				this->base.combine_inertia(currentLink,currentLink.joint().origin());           // Combine inertia of this link with the base
				
				for(auto childLink : currentLink.child_links())                                 // Cycle through all the child links
				{
					childLink->clear_parent_link();                                            // Link has been merged, so sever the connection
				}
			}
			else parentLink->merge(currentLink);                                                 // Merge this link in to the previous
		
			ReferenceFrame<DataType> frame = {parentLink, currentLink.joint().origin()};
			
			this->_frameList.emplace(currentLink.name(),frame);			                    // Save this link as a reference frame so we can search it later
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
	this->_jointGravityVector.resize(this->_numberOfJoints);
	this->_jointBaseInertiaMatrix.resize(this->_numberOfJoints, NoChange);
	this->_jointBaseCoriolisMatrix.resize(this->_numberOfJoints, NoChange);
	
	std::cout << "[INFO] [KINEMATIC TREE] Successfully generated the '" << this->_name << "' robot model."
	          << " It has " << this->_numberOfJoints << " joints (reduced from " << this->_fullLinkList.size() << ").\n";
	
     #ifndef NDEBUG
	     std::cout << "\nHere is a list of all the joints on the robot:\n";		
	     vector<Link<DataType>*> candidateList = this->_baseLinks;                                 // Start with links connect to base
	     while(candidateList.size() > 0)
	     {
		     Link<DataType> *currentLink = candidateList.back();                                  // Get the one at the end
		     
		     candidateList.pop_back();                                                            // Delete it from the search list
		     
		     std::cout << " - " << currentLink->joint().name() << "\n";
		     
		     vector<Link<DataType>*> temp = currentLink->child_links();                           // All the links attached to the current link
		     
		     if(temp.size() != 0) candidateList.insert(candidateList.end(), temp.begin(), temp.end());
	     }
	     
	     std::cout << "\nHere are the reference frames on the robot:\n";
	     for(const auto &[name, frame] : this->_frameList)
	     {
		     std::cout << " - " << name << " is on the ";
		     
		     if(frame.link == nullptr) std::cout << this->base.name() << ".\n";
		     else                      std::cout << "'" << frame.link->name() << "' link.\n";
	     }
     #endif
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Update the kinematics and dynamics                               //        
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool KinematicTree<DataType>::update_state(const Eigen::Vector<DataType, Eigen::Dynamic> &jointPosition,
                                           const Eigen::Vector<DataType, Eigen::Dynamic> &jointVelocity,
                                           const Pose<DataType>                          &basePose,
                                           const Eigen::Vector<DataType, 6>              &baseTwist)
{
	if(jointPosition.size() != this->_numberOfJoints
	or jointVelocity.size() != this->_numberOfJoints)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
		          << "This model has " << this->_numberOfJoints << " joints, but "
		          << "the joint position argument had " << jointPosition.size() << " elements, "
		          << "and the joint velocity argument had " << jointVelocity.size() << " element." << std::endl;
		
		return false;
	}
	
	this->_jointPosition = jointPosition;                                                          // Transfer joint position values
	
	this->_jointVelocity = jointVelocity;                                                          // Transfer joint velocity values

	this->base.update_state(basePose, baseTwist);                                                  // Update the state for the base
			
	// Need to clear for new loop
	this->_jointInertiaMatrix.setZero();
	this->_jointCoriolisMatrix.setZero();
	this->_jointGravityVector.setZero();
	this->_jointBaseInertiaMatrix.setZero();
	this->_jointBaseCoriolisMatrix.setZero();

	std::vector<Link<DataType>*> candidateList = this->_baseLinks;                                 // Start with links attached to base
	
	while(candidateList.size() > 0)
	{
		Link<DataType> *currentLink = candidateList.back();                                       // Get last link in list...
		
		candidateList.pop_back();                                                                 // ... and remove it
		
		unsigned int k = currentLink->number();                                                   // The number of this link/joint
		
		Link<DataType> *parentLink = currentLink->parent_link();                                  // Get the parent of this one
	
		// Propagate the forward kinematics
		bool success = false;
		if(parentLink == nullptr)                                                                 // No parent link; must be connected to base
		{
			success = currentLink->update_state(this->base.pose(), this->base.twist(),
			                                    jointPosition(k),  jointVelocity(k));
		}
		else	success = currentLink->update_state(jointPosition(k), jointVelocity(k));

		if(not success)
		{
			std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
				     << "Unable to to update the state for the '" << currentLink->name()
				     << "' link (" << currentLink->joint().name() << " joint).\n";
			
			return false;
		}
		
		// Compute the inverse dynamics

		Eigen::Matrix<DataType,6,Eigen::Dynamic> J = jacobian(currentLink, currentLink->center_of_mass(), k+1); // Jacobian to centre of mass
		
		Eigen::Matrix<DataType,3,Eigen::Dynamic> Jv = J.block(0,0,3,k+1);                         // Linear component
		
		Eigen::Matrix<DataType,3,Eigen::Dynamic> Jw = J.block(3,0,3,k+1);                         // Angular component
		
		DataType mass = currentLink->mass();                                                      // As it says
		
		// Compute the upper-right triangle of the inertia matrix
		for(int i = 0; i < k+1; i++)
		{
			for(int j = i; j < k+1; j++)
			{
				this->_jointInertiaMatrix(i,j) += mass * Jv.col(i).dot(Jv.col(j))
				                                       + Jw.col(i).dot(currentLink->inertia()*Jw.col(j));
			}
		}
		
		Eigen::Matrix<DataType,6,Eigen::Dynamic> Jdot = time_derivative(J);                       // As it says
		
		this->_jointCoriolisMatrix.block(0,0,k+1,k+1) += mass * Jv.transpose()*Jdot.block(0,0,3,k+1)
		                                                      + Jw.transpose()*(currentLink->inertia_derivative()*Jw + currentLink->inertia()*Jdot.block(3,0,3,k+1));
		
		this->_jointGravityVector.head(k+1) -= mass*Jv.transpose()*this->_gravityVector;          // We need to NEGATE gravity		
		
		// Compute inertial coupling between the base and links
		this->_jointBaseInertiaMatrix.block(0,0,k+1,3) += mass * Jv.transpose();
		this->_jointBaseInertiaMatrix.block(0,3,k+1,3) += Jw.transpose()*this->base.inertia()
		                                                - mass * (SkewSymmetric<DataType>(currentLink->center_of_mass() - this->base.pose().translation()) * Jv).transpose();
		
		this->_jointBaseCoriolisMatrix.block(0,3,k+1,3) += Jw.transpose()*this->base.inertia_derivative()
		                                                 - mass * (SkewSymmetric<DataType>(currentLink->twist().head(3))*Jv).transpose();
		
		// Get the links attached to this one and add them to the list
		std::vector<Link<DataType>*> temp = currentLink->child_links();
		if(temp.size() > 0) candidateList.insert(candidateList.begin(),temp.begin(),temp.end());
	}
	
	// Complete the lower-left triangle of the inertia matrix
	for(int i = 1; i < this->_numberOfJoints; i++)
	{
		for(int j = 0; j < i; j++) this->_jointInertiaMatrix(i,j) = this->_jointInertiaMatrix(j,i);
	}
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Matrix<DataType, 6, Eigen::Dynamic>
KinematicTree<DataType>::jacobian(Link<DataType> *link,                                             // Starting point in the kinematic tree
                                  const Eigen::Vector<DataType, 3> &point,                          // A point in the given joint frame
                                  const unsigned int &numberOfColumns)                              // Must be less than or equal to number of joints in model
{
     // Whitney, D. E. (1972)
     // "The mathematics of coordinated control of prosthetic arms and manipulators"
     // Journal of Dynamic Systems, Measurement, and Control, pp. 303-309
     
	Eigen::Matrix<DataType,6,Eigen::Dynamic> J(6,numberOfColumns); J.setZero();
	
	if(link->number() > numberOfColumns)
	{
		throw std::logic_error("[ERROR] [KINEMATIC TREE] jacobian(): "
		                       "Cannot compute a Jacobian with " + std::to_string(numberOfColumns) +
		                       "columns using " + std::to_string(link->number()+1) + " joints.");
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
template <class DataType> inline
Eigen::Matrix<DataType, 6, Eigen::Dynamic>
KinematicTree<DataType>::time_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix)
{

     // An algorithm for computing the time derivative is provided in this reference,
     // but it's not the same as this one:
     // Angeles, J. (2003) "Fundamentals of Robotic Mechanical Systems: Theory, Methods, and Algorithms"
     // Springer
     
	Eigen::Matrix<DataType,6,Eigen::Dynamic> Jdot(6,jacobianMatrix.cols());
	Jdot.setZero();

	for(int i = 0; i < jacobianMatrix.cols(); i++)
	{
		for(int j = 0; j <= i; j++)
		{
			// Compute dJ(i)/dq(j)
			if(this->_link[j]->joint().is_revolute())
			{
				DataType qdot = this->_jointVelocity(j);                                        // Makes things a little easier
				
				// qdot_j * ( a_j x (a_i x r_i) )
				Jdot(0,i) += qdot*(jacobianMatrix(4,j)*jacobianMatrix(2,i) - jacobianMatrix(5,j)*jacobianMatrix(1,i));
				Jdot(1,i) += qdot*(jacobianMatrix(5,j)*jacobianMatrix(0,i) - jacobianMatrix(3,j)*jacobianMatrix(2,i));
				Jdot(2,i) += qdot*(jacobianMatrix(3,j)*jacobianMatrix(1,i) - jacobianMatrix(4,j)*jacobianMatrix(0,i));
				
				if(this->_link[i]->joint().is_revolute())			                         // J_i = [a_i x r_i; a_i]
				{
					// qdot_j * ( a_j x a_i )
					Jdot(3,i) += qdot*(jacobianMatrix(4,j)*jacobianMatrix(5,i) - jacobianMatrix(5,j)*jacobianMatrix(4,i));
					Jdot(4,i) += qdot*(jacobianMatrix(5,j)*jacobianMatrix(3,i) - jacobianMatrix(3,j)*jacobianMatrix(5,i));
					Jdot(5,i) += qdot*(jacobianMatrix(3,j)*jacobianMatrix(4,i) - jacobianMatrix(4,j)*jacobianMatrix(3,i));
				}	
			}
			
			// Compute dJ(j)/dq(i)
			if(i != j && this->_link[j]->joint().is_revolute())	                              // J_j = [a_j x r_j; a_j]
			{
				DataType qdot = this->_jointVelocity(i);                                        // Makes things a little easier
				
				if(this->_link[i]->joint().is_revolute())			                         // J_i = [a_i x r_i; a_i]
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
template <class DataType> inline
Eigen::Matrix<DataType, 6, Eigen::Dynamic>
KinematicTree<DataType>::partial_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix,
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

	Eigen::Matrix<DataType,6,Eigen::Dynamic> dJ(6,numberOfColumns);                                // Value to be returned
	dJ.setZero();
	
	int j = jointNumber;                                                                           // Makes things a little easier
	
	for(int i = 0; i < numberOfColumns; i++)
	{
		if(this->_link[i]->joint().is_revolute())					                         // J_i = [a_i x r_i ; a_i]
		{
			if(this->_link[j]->joint().is_revolute()) 			                              // J_i = [a_j x r_j; a_j]
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
			else if(this->_link[j]->joint().is_prismatic() and j > i)	                         // J_j = [a_j ; 0]
			{
				// a_j x a_i
				dJ(0,i) = jacobianMatrix(1,j)*jacobianMatrix(2,i) - jacobianMatrix(2,j)*jacobianMatrix(1,i);
				dJ(1,i) = jacobianMatrix(2,j)*jacobianMatrix(0,i) - jacobianMatrix(0,j)*jacobianMatrix(2,i);
				dJ(2,i) = jacobianMatrix(0,j)*jacobianMatrix(1,i) - jacobianMatrix(1,j)*jacobianMatrix(0,i);
			}
		}
		else if(this->_link[i]->joint().is_prismatic()			                              // J_i = [a_i ; 0]
		    and this->_link[j]->joint().is_revolute()				                         // J_j = [a_j x r_j; a_j]
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
template <class DataType> inline
Eigen::Matrix<DataType,6,Eigen::Dynamic> KinematicTree<DataType>::jacobian(const std::string &frameName)
{
     auto container = this->_frameList.find(frameName);                                             // Find the given frame in the dictionary
     
     if(container == this->_frameList.end())
     {
          throw std::runtime_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                                   "Could not find a frame called '" + frameName + "' in the model.");
     }
     
     return jacobian(&container->second);                                                           // Get pointer to frame, pass onward
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the pose of a reference frame on the robot                         //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> KinematicTree<DataType>::frame_pose(const std::string &frameName)
{
     auto container = this->_frameList.find(frameName);                                             // Search for the given frame in the list
     
     if(container == this->_frameList.end())
     {
          throw std::runtime_error("[ERROR] [KINEMATIC TREE] frame_pose(): "
                                   "Unable to find a frame called '" + frameName + "' in the model.");
     }
    
	return (container->second).link->pose()*(container->second).relativePose;                      // NOTE: container->second is a ReferenceFrame data structure.
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Convert a char of numbers to an Eigen::Vector<DataType,3> object               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Eigen::Vector<DataType,3> KinematicTree<DataType>::char_to_vector(const char* character)
{
	std::string numberAsString;                                                                    // So we can concatenate individual char together
	std::vector<DataType> numberAsVector;                                                          // Temporary storage
	
	int startPoint = 0;                                                                         
	for(int i = 0; i < std::strlen(character); i++)
	{
		if(character[i] == ' ')                                                                   // Find the space in the char array
		{
			for(int j = startPoint; j < i; j++) numberAsString += character[j];                  // Add the character to the string
			
			numberAsVector.push_back((DataType)std::stod(numberAsString));                       // Convert to double (override DataType)
			
			startPoint = i+1;                                                                    // Advance the start point
			
			numberAsString.clear();                                                              // Clear the string to get the new number
		}
	}
	
	for(int i = startPoint; i < std::strlen(character); i++) numberAsString += character[i];       // Get the last number in the char array
	
	numberAsVector.push_back(std::stof(numberAsString));

	return Eigen::Vector<DataType,3>(numberAsVector.data());                                       // Return the extracted data
}

#endif
