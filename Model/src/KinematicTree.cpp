#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
	// NOTE: We should probably write a "check_urdf" function to ensure it is valid
	
	std::string errorMessage = "[ERROR] [KINEMATIC TREE] Constructor: ";
	
	// Check to see if the file exists
	if(not std::ifstream(pathToURDF.c_str()))
	{
		throw std::runtime_error(errorMessage + "The file " + pathToURDF + " does not appear to exist.");
	}

	tinyxml2::XMLDocument urdf; urdf.LoadFile(pathToURDF.c_str());                              // Load URDF           
	
	// Make sure a robot is defined
	tinyxml2::XMLElement* robot = urdf.FirstChildElement("robot");
	if(robot == nullptr)
	{
		throw std::runtime_error(errorMessage + "There does not appear to be a 'robot' element in the URDF.");
	}
	
	// Search through every link in the urdf and conver to RobotLibrary::Link object
	for(auto link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
	{
		// Default properties
		float mass = 0.0;
		std::string linkName = link->Attribute("name");
		Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();
		Pose centreOfMass(Eigen::Vector3f::Zero(), Eigen::Quaternionf(1,0,0,0));
		
		// Get inertia properties if they exist
		tinyxml2::XMLElement* inertial = link->FirstChildElement("inertial");
		if(inertial != nullptr)
		{
			mass = inertial->FloatAttribute("mass");
			
			float ixx = inertial->FloatAttribute("ixx");
			float ixy = inertial->FloatAttribute("ixy");
			float ixz = inertial->FloatAttribute("ixz");
			float iyy = inertial->FloatAttribute("iyy");
			float iyz = inertial->FloatAttribute("iyz");
			float izz = inertial->FloatAttribute("izz");
			
			momentOfInertia << ixx, ixy, ixz,
			                   ixy, iyy, iyz,
			                   ixz, iyz, izz;
			
			tinyxml2::XMLElement* origin = inertial->NextSiblingElement("origin");
			if(origin != nullptr)
			{
				Eigen::Vector3f xyz = char_to_vector3f(origin->Attribute("xyz"));
				Eigen::Vector3f rpy = char_to_vector3f(origin->Attribute("rpy"));
				
				centreOfMass = Pose(xyz, Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
				                        *Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
				                        *Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ()));
			}
		}
		
		RigidBody rigidBody(mass,momentOfInertia,centreOfMass);                             // Temporary object
		
		this->linkList.emplace(linkName, Link(linkName, rigidBody));                        // Add to the list
	}
	
	// Search through every joint
	for(auto joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
	{
		std::string jointName = joint->Attribute("name");
		std::string jointType = joint->Attribute("type");

		// Get the joint properties
		if(jointType == "continuous"
		or jointType == "fixed"
		or jointType == "prismatic"
		or jointType == "revolute")
		{
			// Default properties
			float mass            = 0.0;
			float forceLimit      = 10;
			float damping         = 1.0;
			float friction        = 0.0;
			float positionLimit[] = {-M_PI, M_PI};
			float velocityLimit   = 100*2*M_PI/60;
			Eigen::Vector3f axis  = {1,0,0};
			Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();
			Pose origin(Eigen::Vector3f::Zero(), Eigen::Quaternionf(1,0,0,0));
			
			// Get the pose of the joint relative to preceeding link
			tinyxml2::XMLElement* originElement = joint->FirstChildElement("origin");
			if(originElement != nullptr)
			{
				Eigen::Vector3f xyz = char_to_vector3f(originElement->Attribute("xyz"));
				Eigen::Vector3f rpy = char_to_vector3f(originElement->Attribute("rpy"));
				
				origin = Pose(xyz, Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
						  *Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
						  *Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ()));
			}

			// Get joint properties for non-fixed type
			if(jointType != "fixed")
			{	
				// Get the joint limits	
				if(jointType == "continuous")                                       // No limits on the joints
				{
					float inf = std::numeric_limits<float>::infinity();
					positionLimit[0] =-inf;
					positionLimit[1] = inf;
					velocityLimit    = inf;
					forceLimit       = inf;
				}
				else                                                                // Obtain the joint limits
				{
					tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
					if(limit != nullptr)
					{
						positionLimit[0] = limit->FloatAttribute("lower");
						positionLimit[1] = limit->FloatAttribute("upper");
						velocityLimit    = limit->FloatAttribute("velocity"); // NOTE: This is apparently option so could cause problems in the future
						forceLimit       = limit->FloatAttribute("effort");
					}
				}
				
				// Get the axis of actuation					
				tinyxml2::XMLElement* axisElement = joint->FirstChildElement("axis");
				if(axisElement != nullptr) axis = char_to_vector3f(axisElement->Attribute("xyz"));
				
				// Get the dynamic properties
				tinyxml2::XMLElement* dynamics = joint->FirstChildElement("dynamics");
				if(dynamics != nullptr)
				{
					damping  = dynamics->FloatAttribute("damping");
					friction = dynamics->FloatAttribute("friction");
				}
			}
			
			Joint j0int(jointName, jointType, axis, origin, positionLimit,
			            velocityLimit, forceLimit, damping, friction);                  // Create Joint object
			            
			this->jointList.emplace(jointName, j0int);                                  // Add to the list
			
			// Find the parent link
			std::string parentName = joint->FirstChildElement("parent")->Attribute("link"); // Get the name of the parent joint
			
			auto temp = this->linkList.find(parentName);                                // Find it in the list
			
			if(temp == this->linkList.end()) throw std::runtime_error(errorMessage + "Could not find the parent link in the list.");
			
			this->jointList.at(jointName).set_parent_link(&temp->second);               // Set the pointer to the parent link
			
			// Find the child link
			std::string childName = joint->FirstChildElement("child")->Attribute("link"); // Get the name of the child joint
			
			temp = this->linkList.find(childName);                                      // Find it in the list
			
			if(temp == this->linkList.end()) throw std::runtime_error(errorMessage + "Could not find the child link in the list.");
			
			this->jointList.at(jointName).set_child_link(&temp->second);                // Set pointer to child link
		}
		else
		{
			errorMessage += "The " + jointName + " joint was " + jointType + " but "
			              + "RobotLibrary::KinematicTree can only handle continuous, "
			              + "fixed, prismatic, or revolute joints.";
			                
			throw std::invalid_argument(errorMessage);
		}
	}
	
	std::cout << "There are " << this->jointList.size() << " joints and " << this->linkList.size() << " links.\n";

	// FIRST PASS: For a sequence of links and joints A <-- a <-- B <-- b
	// If joint 'a' is fixed:
	// - Merge link 'B' in to link 'A'
	// - Update the joint origin of 'b' relative to 'A'
	// So the new sequence is (A+B) <-- b
	// At the same time, assign the base link
	for(auto iterator = this->jointList.begin(); iterator != this->jointList.end(); iterator++)
	{
		Joint *b = &iterator->second;                                                       // This current joint
		Link  *B = b->parent_link();                                                        // The parent link
		Joint *a = B->attached_joint();                                                     // The joint attached to the parent
		
		if(a == nullptr) 								    // Link B must be the base since it has no preceding joint
		{
			if(this->baseLink == nullptr) this->baseLink = B;                           // Pointer to the base
			else if(this->baseLink->name() != B->name())
			{
				throw std::runtime_error(errorMessage + "More than one base link candidate. The current base link is " + this->baseLink->name() + ", but also found " + B->name() + ".");
			}
		}
		else
		{
			if(std::string(a->type()) == "fixed")
			{
				Link *A = a->parent_link();                                         // Parent of joint A
				
				std::cout << "The " << B->name() << " link needs to be merged with the "
				          << A->name() << " link.\n";
				          
				A->merge(B);                                                        // Merge the links
				
				b->set_parent_link(A);                                              // Set Link 'A' as the new parent of joint 'b'
				
				Pose temp = a->origin();
				
				b->offset_origin(temp);                                             // Offset the origin of joint 'b' relative to link 'A"
				
				this->jointList.erase(a->name());                                   // Remove joint 'a' from the list
				
				this->linkList.erase(B->name());                                    // Erase link 'B' from the list
			}
		}	
	}
	
	std::cout << "There are now " << this->jointList.size() << " joints and " << this->linkList.size() << " links.\n";
	
	// SECOND PASS: Merge all links with fixed joints A <-- a <-- B
	// (assuming tree does not extend beyong B)
	// At the same time, list joints attached to the base link
	for(auto iterator = this->jointList.begin(); iterator != this->jointList.end(); iterator++)
	{
		Joint *a = &iterator->second;                                                       // Get as a pointer
		
		Link *A = a->parent_link();
		
		Link *B = a->child_link();
		
		if(std::string(a->type()) == "fixed")
		{
			
			std::cout << B->name() << " needs to be merged in to " << A->name() << std::endl;
			
			A->merge(B);
			
			this->linkList.erase(B->name());
			
			// this->jointList.erase(a->name()); // THIS CAUSES  A SEGMENTATION FAULT???
		}
	}                                
	
	std::cout << "I believe '" << this->baseLink->name() << "' is the base link.\n";

	std::cout << "All done.\n";
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Update the kinematics and dynamics                               //        
///////////////////////////////////////////////////////////////////////////////////////////////////
bool KinematicTree::update_state(const Eigen::VectorXf &jointPos,
                                 const Eigen::VectorXf &jointVel,
                                 const Pose &basePose,
                                 const Eigen::Matrix<float,6,1> &baseTwist)
{
	if(jointPos.size() != this->numJoints
	or jointVel.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
		          << "This model has " << this->numJoints << " joints, but "
		          << "the joint position argument had " << jointPos.size() << " elements, "
		          << "and the joint velocity argument had " << jointVel.size() << " element.\n";
		
		return false;
	}
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Convert a char of numbers to an Eigen::Vector3f object                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f KinematicTree::char_to_vector3f(const char* character)
{
	std::string        numberAsString;                                                          // So we can concatenate individual char together
	std::vector<float> numberAsVector;                                                          // Temporary storage
	
	int startPoint = 0;                                                                         
	for(int i = 0; i < strlen(character); i++)
	{
		if(character[i] == ' ')                                                             // Find the space in the char array
		{
			for(int j = startPoint; j < i; j++) numberAsString += character[j];         // Add the character to the string
				
			numberAsVector.push_back(std::stof(numberAsString));                        // Convert the string to a float, put in vector
			
			startPoint = i+1;                                                           // Advance the start point
			
			numberAsString.clear();                                                     // Clear the string to get the new number
		}
	}
	
	// Get the last number in the char array
	for(int i = startPoint; i < strlen(character); i++) numberAsString += character[i];
	numberAsVector.push_back(std::stof(numberAsString));

	return Eigen::Vector3f(numberAsVector.data());                                              // Return the extracted data
}

