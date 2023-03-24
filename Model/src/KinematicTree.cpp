#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
	std::string errorMessage = "[ERROR] [KINEMATIC TREE] Constructor: ";
	
	if(not std::ifstream(pathToURDF.c_str()))
	{
		errorMessage += "The file " + pathToURDF + " does not appear to exist.";
		
		throw std::runtime_error(errorMessage);
	}
	else
	{
		tinyxml2::XMLDocument urdf;
		urdf.LoadFile(pathToURDF.c_str());
		
		tinyxml2::XMLElement* robot = urdf.FirstChildElement("robot");
		if(robot == nullptr)
		{
			errorMessage += "There does not appear to be a robot element in the URDF.";
			
			throw std::runtime_error(errorMessage);
		}
		
		std::map<const char*, RigidBody> linkList;                                          // Create a map so we can find which link its attached to
		
		// Search through every link
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
		
			linkList.emplace(link->Attribute("name"), RigidBody(mass,momentOfInertia,centreOfMass)); // Add to list
		}
		
		if(linkList.empty())
		{
			errorMessage += "Could not find any links in the given URDF.";
			throw std::runtime_error(errorMessage);
		}
			
		// Search through every joint
		for(auto joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
		{	
			const char* jointName = joint->Attribute("name");
			const char* proceedingLink = joint->FirstChildElement("child")->Attribute("link");
			
			if(proceedingLink != nullptr)
			{	
			
				//if(auto search = linkList.find(proceedingLink))
				//{
					std::cout << "The " << proceedingLink << " link is attached to the " << jointName << " joint.\n";
				//}
				//else
				//{
				//	std::cout << "Could not find the attached link..." << std::endl;
				//}
			
			}
			else
			{
				errorMessage += "The " + std::to_string(*jointName) + " joint has no child link. Is this correct?";
				throw std::runtime_error(errorMessage);
			}
		}
	}
}

			/*
			// Extract joint friction information
			tinyxml2::XMLElement* dynamics = joint->FirstChildElement("dynamics");
			float damping  = 1.0;
			float friction = 0.0;
			
			if(dynamics == nullptr)
			{
				// Compiler is throwing a warning on these lines:
//				damping  = dynamics->FloatAttribute("damping");
//				friction = dynamics->FloatAttribute("friction");
			}
		
			// Extract joint limit information
			tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
//			float positionLimit[2] = {limit->FloatAttribute("lower"), limit->FloatAttribute("upper")};
//			float velocityLimit    = limit->FloatAttribute("velocity");
//			float forceLimit       = limit->FloatAttribute("effort");
		
			// Extract joint pose information
			tinyxml2::XMLElement* origin = joint->FirstChildElement("origin");
			Eigen::Vector3f pos;
			Eigen::Quaternionf quat;
			if(origin == nullptr)
			{
				pos.setZero();
				quat.setIdentity();
			}
			else
			{
				pos = char_to_vector3f(origin->Attribute("xyz"));
				
				Eigen::Vector3f rpy = char_to_vector3f(origin->Attribute("rpy"));
				
				quat = Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
				     * Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
				     * Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ());				
			}
			
			// Create the joint object
			Joint jointObject(jointName,
			                  type,
			                  axisVector,
			                  Pose(pos,quat),
			                  positionLimit,
			                  velocityLimit,
			                  forceLimit,
			                  damping,
			                  friction);
			
			// Get the link attached to it
			RigidBody rigidBody;
			
			tinyxml2::XMLElement* proceedingLink = joint->FirstChildElement("child");
			if(proceedingLink == nullptr)
			{
				std::string message = "[ERROR] [KINEMATIC TREE] Constructor: Joint ";
				message.push_back(*jointName);                                      // c++ is really annoying sometimes
				message += " has no 'child' link attached to it!";
				                      
				throw std::runtime_error(message);
			}
			else
			{
				// Variables used in this scope
				float mass;
				Eigen::Matrix3f momentOfInertia;
				Eigen::Vector3f com;
				Eigen::Vector3f rpy;
				
				tinyxml2::XMLElement* inertial = proceedingLink->FirstChildElement("inertial");
				if(inertial != nullptr)
				{
					mass = inertial->FloatAttribute("mass");
					
					float ixx  = inertial->FloatAttribute("ixx");
					float ixy  = inertial->FloatAttribute("ixy");
					float ixz  = inertial->FloatAttribute("ixz");
					float iyy  = inertial->FloatAttribute("iyy");
					float iyz  = inertial->FloatAttribute("iyz");
					float izz  = inertial->FloatAttribute("izz");
					
					momentOfInertia << ixx, ixy, ixz,
						           ixy, iyy, iyz,
						           ixz, iyz, izz;

					tinyxml2::XMLElement* origin = proceedingLink->FirstChildElement("inertial");
					com = (origin == nullptr) ? Eigen::Vector3f::Zero() : char_to_vector3f(origin->Attribute("xyz"));
					rpy = (origin == nullptr) ? Eigen::Vector3f::Zero() : char_to_vector3f(origin->Attribute("rpy"));	
				}
				else
				{
					mass = 0.1;
					momentOfInertia = 1e-03*Eigen::Matrix3f::Identity();
					com.setZero();
					rpy.setZero();
					
				}
				
				Pose centreOfMass(com, Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
				                     * Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
				                     * Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ()));	
				
				RigidBody rigidBody(mass,momentOfInertia, centreOfMass);
				
				// Add the joint and link to create a RobotLibrary::Link object
				
				this->link.emplace_back(jointObject,rigidBody);
			}
			*/

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Convert a char of numbers to an Eigen::Vector3f object                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f KinematicTree::char_to_vector3f(const char* character)
{
	// I don't know why it's 5 ¯\_(ツ)_/¯
	if(strlen(character) == 5)
	{
		return Eigen::Vector3f(std::atof(&character[1]),
		                       std::atof(&character[2]),
		                       std::atof(&character[3]));
	}
	else
	{
		std::cout << "[ERROR] [KINEMATIC TREE] char_to_vector3f(): "
		          << "Expected a char of length 5 but it was " << strlen(character) << "." << std::endl;

		return Eigen::Vector3f(1,0,0);
	}
}

