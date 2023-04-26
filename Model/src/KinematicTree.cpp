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
			                                                     
			// Find the parent link
			std::string parentName = joint->FirstChildElement("parent")->Attribute("link"); // Get the name of the parent joint
			
			auto temp = this->linkList.find(parentName);                                // Find it in the list
			
			if(temp == this->linkList.end()) throw std::runtime_error(errorMessage + "Could not find the parent link in the list.");
			
			j0int.set_parent_link(&temp->second);                                       // Set the pointer to the parent link
			
			// Find the child link
			std::string childName = joint->FirstChildElement("child")->Attribute("link"); // Get the name of the child joint
			
			temp = this->linkList.find(childName);                                      // Find it in the list
			
			if(temp == this->linkList.end()) throw std::runtime_error(errorMessage + "Could not find the child link in the list.");
			
			j0int.set_child_link(&temp->second);                                        // Set the pointer to the child link
	
			temp->second.attach_joint(&j0int);                                          // Reference this joint in the child link
			
			this->jointList.emplace(jointName, j0int);                                  // Add to the joint list
		}
		else
		{
			errorMessage += "The " + jointName + " joint was " + jointType + " but "
			              + "RobotLibrary::KinematicTree can only handle continuous, "
			              + "fixed, prismatic, or revolute joints.";
			                
			throw std::invalid_argument(errorMessage);
		}
	}
	
	// Find the base
	for(auto iterator = this->linkList.begin(); iterator != this->linkList.end(); iterator++)
	{
		if(iterator->second.attached_joint() == nullptr)
		{
			std::cout << "I believe that '" << iterator->first << "' is the base.\n";
		}
	}
	
	// For a sequence of links and joints  A <-- a <-- B <-- b
	// if joint 'a' is fixed, merge link 'B' in to link 'A',
	// update the origin of joint 'b' relative to 'A',
	// then delete  joint 'a' from the list
	for(auto iterator = this->jointList.begin(); iterator != this->jointList.end(); iterator++)
	{
		/////////////////// This block of code works fine //////////////////////////////////
		Joint *b = &iterator->second;
		
		Link *B = b->parent_link();
		
		Link *C = b->child_link();
		
		std::cout << B->name() << " <-- (" << b->name() << ") <-- " << C->name() << std::endl;
		////////////////////////////////////////////////////////////////////////////////////
		
		
		
		/*
		////////////////////// This has memory leaks //////////////////////////////////////
		Joint *a = B->attached_joint();
		
		if(a != nullptr)
		{
			Link *A = a->parent_link();
			
			std::cout << A->name() << " <-- (" << a->name() << ") <-- "
			          << B->name() << " <-- (" << b->name() << ") <-- "
			          << C->name() << std::endl;
		}
		///////////////////////////////////////////////////////////////////////////////////
		*/
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

