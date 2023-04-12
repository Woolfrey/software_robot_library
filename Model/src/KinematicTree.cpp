#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
	// NOTE: We should probably write a "check_urdf" function to ensure it is valid

	// Variables used in this scope
	std::map<std::string, std::string> connectionMap;                                           // Store connection between links and joints
	std::map<std::string, std::shared_ptr<Link>>    linkMap;                                    // 
	std::map<std::string, RigidBody>                rigidBodyMap;                               // 'links' in urdf converted to RigidBody
	
	std::string errorMessage = "[ERROR] [KINEMATIC TREE] Constructor: ";
	
	// Check to see if the file exists
	if(not std::ifstream(pathToURDF.c_str()))
	{
		errorMessage += "The file " + pathToURDF + " does not appear to exist.";
		throw std::runtime_error(errorMessage);
	}

	tinyxml2::XMLDocument urdf; urdf.LoadFile(pathToURDF.c_str());                              // Load URDF           
	
	// Make sure a robot is defined
	tinyxml2::XMLElement* robot = urdf.FirstChildElement("robot");
	if(robot == nullptr)
	{
		errorMessage += "There does not appear to be a robot element in the URDF.";
		throw std::runtime_error(errorMessage);
	}
	
	// Search through every 'link' in the URDF and convert to RigidBody object
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
		
		rigidBodyMap.emplace(linkName, RigidBody(mass,momentOfInertia,centreOfMass, linkName)); // Add to list
	}
	
	// Search through every joint and combine with rigid body to form a link
	for(auto joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
	{
		std::string jointName      = joint->Attribute("name");
		std::string jointType      = joint->Attribute("type");
		std::string parentLinkName = joint->FirstChildElement("parent")->Attribute("link");
		std::string childLinkName  = joint->FirstChildElement("child")->Attribute("link");
		
		connectionMap.emplace(jointName, parentLinkName);                                   // Store the connections so we can find them later
		
//		std::cout << parentLinkName << " <-> " << jointName << " (" << jointType << ") <-> " << childLinkName << std::endl;
		
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
			
			// Create the RobotLibrary::Joint object
			Joint j0int(jointName, jointType, axis, origin, positionLimit, velocityLimit, forceLimit, damping, friction);
			
			// Find its attached link
			auto rigidBody = rigidBodyMap.find(childLinkName);                          // Get the attached rigid body
			if(rigidBody == rigidBodyMap.end())
			{
				errorMessage += "Could not find the rigid body object named " + childLinkName + " in the list.";
				              
				throw std::runtime_error(errorMessage);
			}
			
			linkMap.emplace(childLinkName, std::make_shared<Link>(j0int, rigidBody->second, childLinkName)); // Add them to the map
			
			rigidBodyMap.erase(childLinkName);                                          // Erase the rigid body from the list
		}
		else
		{
			errorMessage += "The " + jointName + " joint was " + jointType +
			                " but the KinematicTree class can only handle continuous, "
			                "fixed, prismatic, or revolute joints.";
			                
			throw std::invalid_argument(errorMessage);
		}
	}
		
	// Find the base
	if(rigidBodyMap.size() != 1)
	{
		errorMessage += "There is more than 1 rigid body without a joint attached:\n";
		for(auto iterator = rigidBodyMap.begin(); iterator != rigidBodyMap.end(); iterator++)
		{
			errorMessage += iterator->first + "\n";
		}
		throw std::runtime_error(errorMessage);
	}
	else
	{
		this->base = rigidBodyMap.begin()->second;
		
		std::cout << "I believe that '" << this->base.name() << "' is the base.\n";
	}
	
	// Assign pointers based on link connections and condense all fixed joints
	for(auto link = linkMap.begin(); link != linkMap.end(); link++)
	{
		std::string jointName = link->second->joint.name();
		std::string jointType = link->second->joint.type();
		
		// Find the preceding link from the connection map
		auto connection = connectionMap.find(jointName);
	 	if(connection == connectionMap.end())
	 	{
	 		errorMessage += "Could not find " + jointName + " in the connection map.";
	 		
	 		throw std::runtime_error(errorMessage);
	 	}

		std::string precedingLinkName  = connection->second;      

		// Assign the preceding link as a pointer
		auto precedingLink = linkMap.find(precedingLinkName);
		if(precedingLinkName != this->base.name())
		{
			if(precedingLink == linkMap.end())
			{
				errorMessage += "Could not find " + precedingLinkName + " in the link map.";
				
				throw std::runtime_error(errorMessage);
			}
			
			if(not link->second->set_preceding_link(precedingLink->second))             // Set the pointer to the preceding link
			{
				std::cerr << "[ERROR] [KINEMATIC TREE] Constructor() : "
				          << " Could not set preceding link.\n";
			}
			
			precedingLink->second->add_proceeding_link(link->second);                   // Make this link a proceeding link of its predecessor
		}
		
		// Now merge all links with fixed joints to the predecessor
		if(link->second->joint.type() == "fixed")
		{
			if(precedingLinkName == this->base.name())
			{
				
			}
			else
			{
				
			}
		}
	}
	
	std::cout << "All done.\n";
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

