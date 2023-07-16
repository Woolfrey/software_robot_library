#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
	// Variables used in this scope
	std::map<std::string, unsigned int> linkList;                                               // So we can find links later and connect them to joints
	
	std::string errorMessage = "[ERROR] [KINEMATIC TREE] Constructor: ";                        // This is used in multiple places below
	
	// Check to see if the file exists
	if(not std::ifstream(pathToURDF.c_str()))
	{
		throw std::runtime_error(errorMessage + "The file " + pathToURDF + " does not appear to exist.");
	}

	tinyxml2::XMLDocument urdf; urdf.LoadFile(pathToURDF.c_str());                              // Load URDF           
	
	// Make sure a robot is defined
	tinyxml2::XMLElement* robot = urdf.FirstChildElement("robot");
	if(robot == nullptr) throw std::runtime_error(errorMessage + "There does not appear to be a 'robot' element in the URDF.");
	
	this->_name = robot->Attribute("name");                                                     // Assign the name
	
	// Search through every link in the urdf and convert to RobotLibrary::Link object
	
	unsigned int linkNumber = 0;
	
	for(auto linkIterator = robot->FirstChildElement("link"); linkIterator; linkIterator = linkIterator->NextSiblingElement("link"))
	{
		// Default properties
		float mass = 0.0;
		Eigen::Vector3f centerOfMass = {0,0,0};
		std::string linkName = linkIterator->Attribute("name");
		Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();
		
		// Get inertia properties if they exist
		tinyxml2::XMLElement* inertial = linkIterator->FirstChildElement("inertial");
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
				
				// NOTE: URDF specifies a pose for the center of mass,
				//       which is a little superfluous...
				
				Pose pose(xyz, Eigen::AngleAxisf(rpy(0),Eigen::Vector3f::UnitX())
				                                       *Eigen::AngleAxisf(rpy(1),Eigen::Vector3f::UnitY())
				                                       *Eigen::AngleAxisf(rpy(2),Eigen::Vector3f::UnitZ()));
				                                       
				centerOfMass = pose.position();                                     // Location for the center of mass
				
				Eigen::Matrix3f R = pose.rotation();                                // Get the SO(3) matrix
				
				momentOfInertia = R*momentOfInertia*R.transpose();                  // Rotate the inertia to the origin frame
			}
		}
		
		this->link.emplace_back(Link(linkName, RigidBody(mass,momentOfInertia,centerOfMass))); // Create Link object and add it to the list
		
		linkList.emplace(linkName, linkNumber);
		
		linkNumber++;
	}
	
	// Search through every joint
	
	int jointNumber = 0;
	
	for(auto jointIterator = robot->FirstChildElement("joint"); jointIterator; jointIterator = jointIterator->NextSiblingElement("joint"))
	{
		std::string jointName = jointIterator->Attribute("name");
		std::string jointType = jointIterator->Attribute("type");

		// Get the joint properties
		if(jointType == "continuous"
		or jointType == "fixed"
		or jointType == "prismatic"
		or jointType == "revolute")
		{
			// Default properties
			float mass            = 0.0;
			float effortLimit     = 10;
			float damping         = 1.0;
			float friction        = 0.0;
			float positionLimit[] = {-M_PI, M_PI};
			float velocityLimit   = 100*2*M_PI/60;
			Eigen::Vector3f axis  = {0,0,1};
			Pose origin(Eigen::Vector3f::Zero(), Eigen::Quaternionf(1,0,0,0));
			
			// Get the pose of the joint relative to preceeding link
			tinyxml2::XMLElement* originElement = jointIterator->FirstChildElement("origin");
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
					effortLimit      = inf;
				}
				else                                                                // Obtain the joint limits
				{
					tinyxml2::XMLElement* limit = jointIterator->FirstChildElement("limit");
					if(limit != nullptr)
					{
						positionLimit[0] = limit->FloatAttribute("lower");
						positionLimit[1] = limit->FloatAttribute("upper");
						velocityLimit    = limit->FloatAttribute("velocity"); // NOTE: This is apparently optional so could cause problems in the future
						effortLimit      = limit->FloatAttribute("effort");
					}
				}
				
				// Get the axis of actuation					
				tinyxml2::XMLElement* axisElement = jointIterator->FirstChildElement("axis");
				if(axisElement != nullptr) axis = char_to_vector3f(axisElement->Attribute("xyz"));
				
				// Get the dynamic properties
				tinyxml2::XMLElement* dynamics = jointIterator->FirstChildElement("dynamics");
				if(dynamics != nullptr)
				{
					damping  = dynamics->FloatAttribute("damping");
					friction = dynamics->FloatAttribute("friction");
				}
			}
			
			this->joint.emplace_back(Joint(jointName, jointType, axis, origin, positionLimit, velocityLimit, effortLimit, damping, friction));
		
			this->joint.back().set_number(jointNumber);
			
			jointNumber++;
		
			// Find and set the parent link
			
			std::string parentLinkName = jointIterator->FirstChildElement("parent")->Attribute("link");
			
			auto linkIterator = linkList.find(parentLinkName);
			
			if(linkIterator == linkList.end())
			{
				throw std::runtime_error(errorMessage + "Could not find '" + parentLinkName + "' in the link list.");
			}
			else
			{
				unsigned int number = linkIterator->second;
				
				if(not this->joint.back().set_parent_link(&this->link[number]))     // Allows us to traverse backward to base link
				{
					errorMessage += "Unable to set the parent link for the '"
					              + this->joint.back().name() + "' joint.";
					              
					throw std::runtime_error(errorMessage);
				}
				
				// NOTE: We will eventually remove 'fixed' joints, and 
				//       setting any child joints of the link here will
				//       mess things up...
			} 		
		
			// Find and set the child link
			
			std::string childLinkName = jointIterator->FirstChildElement("child")->Attribute("link");
			
			linkIterator = linkList.find(childLinkName);
			
			if(linkIterator == linkList.end())
			{
				throw std::runtime_error(errorMessage + "Could not find '" + childLinkName + "' in the link list.");
			}
			else
			{
				unsigned int number = linkIterator->second;
				
				if(not this->joint.back().set_child_link(&this->link[number]))       // Allows use to traverse forward through a branch          
				{
					errorMessage += "Unable to set the child link for the '"
					              + this->joint.back().name() + "' joint.";
					
					throw std::runtime_error(errorMessage);
				}
				else if(not this->link[number].set_parent_joint(&this->joint.back())) // Allows us to traverse backward to base link
				{
					errorMessage += "Unable to set the parent joint for the '"
					              + this->link[number].name() + "' link.";
					
					throw std::runtime_error(errorMessage);
				}
			}
		}
		else
		{
			errorMessage += "The " + jointName + " joint was " + jointType + " but "
			              + "RobotLibrary::KinematicTree can only handle continuous, "
			              + "fixed, prismatic, or revolute joints.";
			                
	 		throw std::invalid_argument(errorMessage);
		}
	}
	
	int prevNumJoints = this->joint.size();
	
	// NOTE: I know this code below is horribly inefficient, but I have spent *FAR* too long
	// trying to figure out something smarter... And at least efficiency isn't critical in the constructor...

	// FIRST PASS: Merge links connected by fixed joints

	for(int i = 0; i < this->joint.size(); i++)
	{
		if(this->joint[i].is_fixed())
		{
			Link *parentLink = this->joint[i].parent_link();
		
			Link *childLink = this->joint[i].child_link();
			
			parentLink->merge(childLink);                                               // Merge the links connected by the fixed joint
			
			// Save the child link as a reference frame
			// so we can search for it later
			ReferenceFrame frame;
			frame.link = parentLink;
			frame.relativePose = this->joint[i].offset();	
			this->referenceFrameList.emplace(childLink->name(), frame);		
		}
	}
	
	// SECOND PASS: For a sequence Link <-- Fixed Joint <-- Merged Link <-- Joint
	//              Set the beginning link as the parent of the end joint
	
	for(int i = 0; i < this->joint.size(); i++)
	{
		Link *parentLink = this->joint[i].parent_link();
		
		Joint *previousJoint = parentLink->parent_joint();
		
		if(previousJoint != nullptr and previousJoint->is_fixed())
		{
			Link *rootLink = previousJoint->parent_link();
			
			this->joint[i].set_parent_link(rootLink);
			
			
		}
	}
	
	// THIRD PASS: Renumber joints, delete fixed ones, set the base
	int i = 0;
	while(i < this->joint.size())
	{
		if(this->joint[i].is_fixed())
		{
			this->joint.erase(this->joint.begin() + i);                                 // Remove the joint
		}
		else
		{
			this->joint[i].set_number(i);                                               // Set new number
			
			Link *parentLink = this->joint[i].parent_link();        
			                      
			parentLink->add_child_joint(&this->joint[i]);
			
			Link *childLink = this->joint[i].child_link();
			
			childLink->set_parent_joint(&this->joint[i]);                               // Need to reassign after deleting joints
			
			Joint *previousJoint = parentLink->parent_joint();
			
			// Set the base link
			if(previousJoint == nullptr)
			{
				if(this->base == nullptr)
				{
					this->base = parentLink;                                    // This must be the base!
				}
				else if(this->base->name() != parentLink->name())
				{
					errorMessage += "More than one candidate base link found: '"
					              + this->base->name() + "' and '"
					              + parentLink->name() + "'.\n";
					              
					throw std::runtime_error(errorMessage);
				}
			}
				
			i++;
		}
	}
	
	this->numJoints = this->joint.size();                                                       // ACTUAL number of actuated joints
	
	// Resize the relevant tensors
	std::cout << this->numJoints <<  " So far so good\n";
	this->jointInertiaMatrix.resize(this->numJoints,this->numJoints);
	this->jointCoriolisMatrix.resize(this->numJoints,this->numJoints);
	this->jointGravityVector.resize(this->numJoints);
	
	std::cout << "[INFO] [KINEMATIC TREE] Successfully created the '" << this->_name << "' robot."
	          << " It has " << this->numJoints << " joints (reduced from " << prevNumJoints << ").\n";
	          
	// Debugging stuff:
	
	if(this->base == nullptr)
	{
		std::cerr << "\n[FLAGRANT SYSTEM ERROR] The base link has not been set!\n";
	}
	else	std::cout << "\nI believe that '" << this->base->name() << "' is the base link.\n";
	
	std::cout << "\nHere is the list of joints:\n";
	for(int i = 0; i < this->joint.size(); i++)
	{
		std::cout << i+1 << ". " <<  this->joint[i].name() << std::endl;
	}
	
	std::cout << "\nHere are the reference frames on the robot:\n";
	for(auto iterator = this->referenceFrameList.begin(); iterator != this->referenceFrameList.end(); iterator++)
	{	
		std::cout << " - '" << iterator->first << "' is on the '" << iterator->second.link->name() << "' link.\n";
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              Update the kinematics and dynamics                               //        
///////////////////////////////////////////////////////////////////////////////////////////////////
bool KinematicTree::update_state(const Eigen::VectorXf &jointPosition,
                                 const Eigen::VectorXf &jointVelocity,
                                 const Pose &basePose,
                                 const Eigen::Matrix<float,6,1> &baseTwist)
{
	if(jointPosition.size() != this->numJoints
	or jointVelocity.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
		          << "This model has " << this->numJoints << " joints, but "
		          << "the joint position argument had " << jointPosition.size() << " elements, "
		          << "and the joint velocity argument had " << jointVelocity.size() << " element.\n";
		
		return false;
	}
	
	this->_jointPosition = jointPosition;
	
	this->_jointVelocity = jointVelocity;
	
	// Update the base properties
	
	this->_basePose  = basePose;
	
	this->_baseTwist = baseTwist;
	
	this->base->set_angular_velocity(this->_baseTwist.tail(3));                                 // We need this for the inverse dynamics algorithm
	
	std::vector<Joint*> candidateList = this->base->child_joints();
	
	while(candidateList.size() > 0)
	{
		//////////////////////////// Forward Kinematics ///////////////////////////////////
	
		// Get all the current and previous links and joints in the chain
		Joint *currentJoint = candidateList.back();                                         // Current link in the tree
		
		Link *currentLink = currentJoint->child_link();                                     // The link attached to this joint
		
		Link *previousLink = currentJoint->parent_link();
		
		Joint *previousJoint = previousLink->parent_joint();
		
		candidateList.pop_back();                                                           // Remove the current joint from the candidate list
		
		unsigned int jointNumber = currentJoint->number();
		
		// Update the pose for the current joint relative to base frame
		Pose jointOrigin;
		
		if(previousJoint == nullptr) jointOrigin = this->_basePose;
		else                         jointOrigin = previousJoint->pose();
		
		if(not currentJoint->update_state(jointOrigin, jointPosition(jointNumber)))
		{
			std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): Unable to update "
			          << "the state for the " << currentJoint->name() << " joint.\n";
			
			return false;
		}
		
		// Propagate the angular velocity
		Eigen::Vector3f angularVelocity = previousLink->angular_velocity();                 // We also need this for inverse dynamics
			
		if(currentJoint->is_revolute())
		{
			angularVelocity += jointVelocity(jointNumber)*currentJoint->axis();
			
			currentLink->set_angular_velocity(angularVelocity);
		}
		
		///////////////////////////// Inverse Dynamics ////////////////////////////////////
		
		float mass = currentLink->mass();
		
		Eigen::Vector3f com = currentJoint->pose() * currentLink->com();                    // Center of mass of current link in base frame
		
		Eigen::Matrix3f R = currentJoint->pose().rotation();

		Eigen::Matrix3f I = R*currentLink->inertia()*R.transpose();                         // Rotate the inertia from local frame to base frame
		
		//Eigen::Matrix3f Idot = angularVelocity.cross(I); // THIS DOESN'T WORK? FUCK YOU (ノಠ益ಠ)ノ彡┻━┻
		
		Eigen::Matrix3f Idot;
		for(int i = 0; i < 3; i++) Idot.col(i) = angularVelocity.cross(I.col(i));
		
		Eigen::MatrixXf J = jacobian(currentJoint, com, jointNumber+1);                     // Get Jacobian to center of mass of this current link/joint
		
		Eigen::MatrixXf Jdot = time_derivative(J);                                          // As it says
		
		Eigen::MatrixXf Jv = J.block(0,0,3,jointNumber);                                    // Linear component
		
		Eigen::MatrixXf Jw = J.block(3,0,3,jointNumber);                                    // Angular component
		
		this->jointInertiaMatrix.block(0,0,jointNumber,jointNumber) += mass*Jv.transpose()*Jv + Jw.transpose()*I*Jw;
		                                                
		this->jointCoriolisMatrix.block(0,0,jointNumber,jointNumber) += mass*Jv.transpose()*Jdot.block(0,0,3,jointNumber)
		                                                              + Jw.transpose()*(Idot*Jw + I*Jdot.block(3,0,3,jointNumber));
		
		this->jointGravityVector.head(jointNumber) -= mass*Jv.transpose()*this->gravityVector; // We need to NEGATE gravity
		
		////////////////////////////////////////////////////////////////////////////////////
		
		std::vector<Joint*> temp = currentLink->child_joints();
		
		if(temp.size() != 0) candidateList.insert(candidateList.end(), temp.begin(), temp.end()); // Add them to the search list
	}
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf KinematicTree::jacobian(Joint *joint,                                               // Starting point in the kinematic tree
                                        const Eigen::Vector3f &point,                               // A point in the given joint frame
                                        const unsigned int &numberOfColumns)                        // Must be less than or equal to number of joints in model
{

	Eigen::MatrixXf J(6,numberOfColumns); J.setZero();                                          // Value to be returned
	
	if(joint->number() > numberOfColumns)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] jacobian(): Joint number is greater than "
		          << "the number of columns (" << joint->number() << " > "
		          << numberOfColumns << ".\n";
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
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the time derivative of a given Jacobian                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf KinematicTree::time_derivative(const Eigen::MatrixXf &J)
{
	if(J.rows() != 6)
	{
		throw std::invalid_argument("[ERROR] [KINEMATIC TREE] time_derivative(): Expected the Jacobian matrix to have 6 rows but it only had " + std::to_string(J.rows()) + ".");
	}
	
	Eigen::MatrixXf Jdot(6,J.cols()); Jdot.setZero();                                           // Value to be returned

	for(int i = 0; i < J.cols(); i++)
	{
		for(int j = 0; j <= i; j++)
		{
			// Compute dJ(i)/dq(j)
			if(this->joint[j].is_revolute())
			{
				float qdot = this->_jointVelocity(j);                               // Makes things a little easier
				
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
				float qdot = this->_jointVelocity(i);                               // Makes things a little easier
				
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
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //             Get the partial derivative of a Jacobian with respect to a given joint            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf partial_derivative(const Eigen::MatrixXf &J, const unsigned int &jointNum)
{
	// E. D. Pohl and H. Lipkin, "A new method of robotic rate control near singularities,"
	// Proceedings. 1991 IEEE International Conference on Robotics and Automation,
	// 1991, pp. 1708-1713 vol.2, doi: 10.1109/ROBOT.1991.131866.
	
	if(J.rows() != 6)
	{
		throw std::invalid_argument("[ERROR] [KINEMATIC TREE] partial_derivative(): Expected the input Jacobian to have 6 rows but it had " + std::to_string(J.rows()) + ".");
	}
	else if(jointNum > J.cols())
	{
		throw std::invalid_argument("[ERROR] [KINEMATIC TREE] partial_derivative(): Cannot take the partial derivative with respect to joint " + std::to_string(jointNum) + " as the Jacobian has only " + std::to_string(J.cols()) + " columns.");
	}

	Eigen::MatrixXf dJ(6,J.cols()); dJ.setZero();                                              // Value to be returned
/*	
	for(int i = 0; i < J.cols(); i++)
	{
		if(this->joint[i].is_revolute())					           // J_i = [a_i x r_i ; a_i]
		{
			if(this->joint[jointNum].is_revolute()) 			           // J_i = [a_j x r_j; a_j]
			{
				if (jointNum < i)
				{
					// a_j x (a_i x a_i)
					dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
					dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
					dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);

					// a_j x a_i
					dJ(3,i) = J(4,jointNum)*J(5,i) - J(5,jointNum)*J(4,i);
					dJ(4,i) = J(5,jointNum)*J(3,i) - J(3,jointNum)*J(5,i);
					dJ(5,i) = J(3,jointNum)*J(4,i) - J(4,jointNum)*J(3,i);
				}
				else
				{
					// a_i x (a_j x a_j)
					dJ(0,i) = J(4,i)*J(2,jointNum) - J(5,i)*J(1,jointNum);
					dJ(1,i) = J(5,i)*J(0,jointNum) - J(3,i)*J(2,jointNum);
					dJ(2,i) = J(3,i)*J(1,jointNum) - J(4,i)*J(0,jointNum);
				}
			}
			else if(this->joint[jointNum].is_prismatic() && jointNum > i)	            // J_j = [a_j ; 0]
			{
				// a_j x a_i
				dJ(0,i) = J(1,jointNum)*J(2,i) - J(2,jointNum)*J(1,i);
				dJ(1,i) = J(2,jointNum)*J(0,i) - J(0,jointNum)*J(2,i);
				dJ(2,i) = J(0,jointNum)*J(1,i) - J(1,jointNum)*J(0,i);
			}
		}
		else if(this->joint[i].is_prismatic()					           // J_i = [a_i ; 0]
			&& this->joint[jointNum].is_revolute()				           // J_j = [a_j x r_j; a_j]
			&& jointNum < i)
		{
			// a_j x a_i
			dJ(0,i) = J(4,jointNum)*J(2,i) - J(5,jointNum)*J(1,i);
			dJ(1,i) = J(5,jointNum)*J(0,i) - J(3,jointNum)*J(2,i);
			dJ(2,i) = J(3,jointNum)*J(1,i) - J(4,jointNum)*J(0,i);
		}
	}
*/	
	return dJ;
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
