#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
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
	std::map<std::string, RigidBody> rigidBodyList;
	
	for(auto link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
	{
		// Default values
		float mass = 0.0;
		Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();
		Pose centreOfMass(Eigen::Vector3f::Zero(), Eigen::Quaternionf(1,0,0,0));
		
		// Get inertia properties if they exist
		tinyxml2::XMLElement* inertial = link->NextSiblingElement("inertial");
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
		
		const char* name = link->Attribute("name");
	
		rigidBodyList.emplace(name, RigidBody(mass,momentOfInertia,centreOfMass, name));    // Add to list
	}

	// Search through every joint and combine with rigid body to form a link
	std::map<std::string, Link> linkList;                                                       // Make a map so we can search later
	std::map<std::string, std::array<std::string,2>> connectionList;                            // Need to store connections between links
	
	for(auto joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
	{
		const char* jointName = joint->Attribute("name");                                   // As it says
		const char* childLinkName = joint->FirstChildElement("child")->Attribute("link");
		std::string jointType = (std::string)joint->Attribute("type");                      // Force a string so we can compare
		
		// Ensure that the child link exists, otherwise terminate
		if(childLinkName == nullptr)
		{
			errorMessage += "The " + (std::string)jointName + " joint has no child link.";
			throw std::runtime_error(errorMessage);
		}
		
		const char* parentLinkName = joint->FirstChildElement("parent")->Attribute("link");
		if(parentLinkName == nullptr)
		{
			errorMessage += "The " + (std::string)jointName + " joint has no parent link.";
			throw std::runtime_error(errorMessage);
		}
		
		// Add connected link names to a list so we can find them later
		std::array<std::string,2> temp = {parentLinkName, childLinkName};
		connectionList.emplace(jointName, temp);
		
		// Get the joint properties
		if(jointType == "continuous"
		or jointType == "fixed"
		or jointType == "prismatic"
		or jointType == "revolute")
		{
			// Get the pose of the joint relative to preceeding link
			Pose origin(Eigen::Vector3f::Zero(), Eigen::Quaternionf(1,0,0,0));          // Default pose
			tinyxml2::XMLElement* originElement = joint->FirstChildElement("origin");
			if(originElement != nullptr)
			{
				Eigen::Vector3f xyz = char_to_vector3f(originElement->Attribute("xyz"));
				Eigen::Vector3f rpy = char_to_vector3f(originElement->Attribute("rpy"));
				
				origin = Pose(xyz, Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
						  *Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
						  *Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ()));
			}

			// Properties to be assigned
			float mass = 0.0;
			float forceLimit      = 10;
			float damping         = 1.0;
			float friction        = 0.0;
			float positionLimit[] = {-M_PI, M_PI};
			float velocityLimit   = 100*2*M_PI/60;
			Eigen::Vector3f axis  = {1,0,0};
			Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();

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
					tinyxml2::XMLElement* limit;
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
			
			// Combine the Joint with the connected RigidBody to form a Link object
			auto rigidBody = rigidBodyList.find(std::string(childLinkName));            // Get the attached rigid body
			if(rigidBody == rigidBodyList.end())
			{
				errorMessage += "Could not find the rigid body object named "
				              + (std::string)childLinkName + " in the list???";
				              
				throw std::runtime_error(errorMessage);
			}
			
			Joint j0int(jointName,jointType,axis,origin,positionLimit,velocityLimit,forceLimit,damping,friction);
			
			linkList.emplace(childLinkName,Link(j0int,rigidBody->second));              // Add Link object to the list
			
			rigidBodyList.erase(std::string(childLinkName));                            // Erase the rigid body from the list
		}
		else
		{
			errorMessage += "The " + (std::string)jointName + " joint was " + jointType +
			                " but the KinematicTree class can only handle continuous, "
			                "fixed, prismatic, or revolute joints.";
			                
			throw std::invalid_argument(errorMessage);
		}
	}
	
	// Find the base
	if(rigidBodyList.size() != 1)
	{
		errorMessage += "There is more than 1 rigid body without a joint attached:\n";
		for(auto iterator = rigidBodyList.begin(); iterator != rigidBodyList.end(); iterator++)
		{
			errorMessage += iterator->first + "\n";
		}
		throw std::runtime_error(errorMessage);
	}
	else
	{
		this->base = rigidBodyList.begin()->second;
		std::cout << "I believe that '" << this->base.name() << "' is the base link.\n";
	}
	
	// Condense the KinematicTree by adding every Link with a fixed joint to its parent
	for(auto link = linkList.begin(); link != linkList.end(); link++)
	{
		std::string jointName = link->second.joint.name();                                  // Get the name of the joint attached to this link
	
		// Find the connected links from the list
		auto joint = connectionList.find(jointName);
		
		if(joint == connectionList.end())
		{
			errorMessage += "Could not find the " + jointName + " joint ( "
			              + link->second.joint.type() + ") in the list.";
			throw std::runtime_error(errorMessage);
		}
		
		std::string precedingLinkName  = joint->second[0];
		std::string proceedingLinkName = joint->second[1];
		
		std::cout << precedingLinkName << " <-> "
		          << jointName << " (" << link->second.joint.type() << ") <-> "
		          << proceedingLinkName << std::endl;
		
		// First add the proceeding links as references
		auto proceedingLink = linkList.find(proceedingLinkName);
		if(proceedingLink == linkList.end())
		{
			errorMessage += "Could not find the " + proceedingLinkName + " link in the list.";
			throw std::runtime_error(errorMessage);
		}
		else if(not link->second.add_proceeding_link(proceedingLink->second))
		{
			errorMessage += "Could not add the " + proceedingLinkName + " as the proceeding link "
			                " of the " + link->first + " link.";
			throw std::runtime_error(errorMessage);
		}
		
		// Now set the preceding link
		auto precedingLink = linkList.find(precedingLinkName);
		if(precedingLinkName == this->base.name())
		{
			// Merge these links with the base
		}
		else if(precedingLink == linkList.end())
		{
			errorMessage += "Could not find the " + precedingLinkName + " link "
			                "for the " + jointName + " joint in the list.";
			throw std::runtime_error(errorMessage);
		}
		else if(not link->second.set_preceding_link(precedingLink->second))
		{
			errorMessage += "Could not set the " + precedingLinkName + " as the preceding link "
			                " of the " + link->first + " link.";
			throw std::runtime_error(errorMessage);
		}
	}
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

