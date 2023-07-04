#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
KinematicTree::KinematicTree(const std::string &pathToURDF)
{
	// NOTE: We should probably write a "check_urdf" function to ensure it is valid!
	
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
	if(robot == nullptr)
	{
		throw std::runtime_error(errorMessage + "There does not appear to be a 'robot' element in the URDF.");
	}
	
	// Search through every link in the urdf and convert to RobotLibrary::Link object
	
	unsigned int linkNumber = 0;
	
	for(auto link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
	{
		// Default properties
		float mass = 0.0;
		Eigen::Vector3f centerOfMass = {0,0,0};
		std::string linkName = link->Attribute("name");
		Eigen::Matrix3f momentOfInertia = Eigen::Matrix3f::Zero();
		
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
	
	unsigned int jointNumber = 0;
	
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
			
			this->joint.emplace_back(Joint(jointName, jointType, axis, origin, positionLimit, velocityLimit, effortLimit, damping, friction, jointNumber));
			
			jointNumber++;                                                              // Increment the joint number
			
			// Find the parent link
			std::string parentLinkName = jointIterator->FirstChildElement("parent")->Attribute("link");
			
			auto temp = linkList.find(parentLinkName);
			
			if(temp == linkList.end()) throw std::runtime_error(errorMessage + "Could not find the parent link of the " + jointName + " joint in the link list.");
			
			if(not this->joint.back().set_parent_link(&this->link[temp->second])) throw std::runtime_error(errorMessage + "Unable to set " + parentLinkName + " as the parent link of the " + jointName + " joint.");
			
			// Find the child link
			
			std::string childLinkName = jointIterator->FirstChildElement("child")->Attribute("link");
			
			temp = linkList.find(childLinkName);
			
			if(temp == linkList.end()) throw std::runtime_error(errorMessage + "Could not find the child link of the " + jointName + " joint in the link list.");			
			
			if(not this->joint.back().set_child_link(&this->link[temp->second])) throw std::runtime_error(errorMessage + "Unable to set " + parentLinkName + " as the parent link of the " + jointName + " joint.");
		}
		else
		{
			errorMessage += "The " + jointName + " joint was " + jointType + " but "
			              + "RobotLibrary::KinematicTree can only handle continuous, "
			              + "fixed, prismatic, or revolute joints.";
			                
	 		throw std::invalid_argument(errorMessage);
		}
	}
		
	int previousNumJoints = this->joint.size();

	// FIRST PASS: For a sequence of links and joints A <-- a <-- B <-- b
	// If joint 'a' is fixed, combine links so we have (A + B) <-- b
	
	// At the same time, assign the base link
		
//	for(auto iterator = this->jointList.begin(); iterator != this->jointList.end(); iterator++)
	for(int i = 0; i < this->joint.size(); i++)
	{
		Joint *b = &this->joint[i];                                                         // Current joint
		Link  *B = b->parent_link();                                                        // Its parent link
		Joint *a = B->joint();                                                              // The joint attached to the parent

		if(a == nullptr) 								    // Link B must be the base since it has no preceding joint
		{
			if(this->base == nullptr) this->base = B;                                   // Pointer to the base
			
			else if(this->base->name() != B->name()) throw std::runtime_error(errorMessage + "More than one base link candidate. The current base link is " + this->base->name() + ", but also found " + B->name() + ".");
		}
		else
		{
			if(std::string(a->type()) == "fixed")
			{
				Link *A = a->parent_link();                                         // Parent of joint A
				     
				A->merge(B);                                                        // Merge the links
				
				b->set_parent_link(A);                                              // Set Link 'A' as the new parent of joint 'b'
				
				Pose temp = a->offset();                                            // Pose of joint relative to parent joint
				
				b->extend_offset(temp);                                             // Offset the origin of joint 'b' relative to link 'A"
				
				ReferenceFrame frame; frame.link = A; frame.relativePose = temp;
				
				this->referenceFrameList.emplace(B->name(), frame);                 // Add it to the list before erasing
				
				this->joint.erase(this->joint.begin()+i);
				
				linkList.erase(B->name());                                          // Erase link 'B' from the list
			}
		}	
	}
	
	// SECOND PASS: Merge all links with fixed joints A <-- a <-- B (assuming tree does not extend beyong B)
	
	// At the same time, list joints attached to the base link
	for(int i = 0; i < this->joint.size(); i++)
	{
		if(this->joint[i].type() == "fixed")
		{
			Link *parentLink = this->joint[i].parent_link();
			Link *childLink  = this->joint[i].child_link();
			
			parentLink->merge(childLink);                                               // Merge the child link with the parent link
			
			ReferenceFrame frame;
			frame.link = parentLink;
			frame.relativePose = childLink->joint()->offset();
			this->referenceFrameList.emplace(childLink->name(),frame);                  // Add the child link as a reference frame
		
			this->joint.erase(this->joint.begin()+i);
		}
	}
	
	// Now connect the links so we can traverse the kinematic tree
	// (There's probably a smarter way to put this in the above code but I'm tired)
	
	for(int i = 0; i < this->joint.size(); i++)
	{
		Link *parentLink = joint[i].parent_link();
		
		Link *childLink = joint[i].child_link();
		
		parentLink->add_next_link(childLink);
		
		childLink->set_previous_link(parentLink);
		
		joint[i].set_number(i);                                                               // New number for the joint?
	}
	
	this->numJoints = this->joint.size();
		
	this->_name = robot->Attribute("name");
	
	std::cout << "[INFO] [KINEMATIC TREE] Successfully parsed the `" << this->_name << "' model from the URDF. "
	          << "There are " << this->numJoints << " joints (reduced from " << previousNumJoints << ").\n";
	          
	std::cout << "\nHere is a list of all the joints:\n";
	for(int i = 0; i < this->numJoints; i++) std::cout << this->joint[i].name() << std::endl;
	
	std::cout << "\nHere is a list of all the reference frames on the robot:\n";
	for(auto iterator = this->referenceFrameList.begin(); iterator != this->referenceFrameList.end(); iterator++)
	{
		std::cout << "The " << iterator->first << " frame is on the " << iterator->second.link->name() << " link.\n";
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
	
	this->base->angularVelocity = this->_baseTwist.tail(3);                                     // We need this for the inverse dynamics algorithm
	
	std::vector<Link*> candidateList = this->base->next_links();
	
	std::cout << "\nTraversing the kinematic tree...\n";
	
	while(candidateList.size() > 0)
	{
	        ///////////////////////////// Forward Kinematics ///////////////////////////////////
	        
		Link *currentLink = candidateList.back();                                           // Get the link at the end of the list
		
		candidateList.pop_back();                                                           // Remove it from the list
		
		Joint *currentJoint = currentLink->joint();                                         // Get the joint attached to this link
		
		unsigned int jointNum = currentJoint->number();                                     // Get the joint index
		
		Pose jointOrigin;
		
		if(currentJoint->parent_link()->joint() == nullptr) jointOrigin = this->_basePose;
		else                                             jointOrigin = currentJoint->parent_link()->joint()->pose();

		if(not currentJoint->update_state(jointOrigin, jointPosition(jointNum)))
		{
			std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): Unable to update the "
			          << "state for the " << currentJoint->name() << " joint.\n";
			
			return false;
		}
		
		currentLink->angularVelocity = currentLink->previous_link()->angularVelocity;       // Propagate the angular velocity
		
		if(currentJoint->is_revolute())
		{
			currentLink->angularVelocity += jointVelocity(jointNum)*currentJoint->axis();  // Add on effect of joint
		}
	
		///////////////////////////// Inverse Dynamics ////////////////////////////////////
		
		/*
		float mass = currentLink->mass();
		
		Pose com = currentJoint->pose() * currentLink->com();                               // Pose to the center of mass of the current link
		
		Eigen::Matrix3f R = com.rotation();                                                 // Orientation as SO(3)                                               
		
		Eigen::Matrix3f I = R*currentLink.inertia()*R.transpose();                          // Rotate inertia to global frame
		
		Eigen::Matrix3f Idot = currentLink->angularVelocity.cross(I);                       // Time derivative
		
		Eigen::MatrixXf J = jacobian(currentJoint, com.position(), jointNum);                  // Get Jacobian to center of mass
		
		Eigen::MatrixXf Jdot = time_derivative(J);                                          // As it says
		
		Eigen::MatrixXf Jv = J.block(0,0,3,index);                                          // Linear component
		
		Eigen::MatrixXf Jw = J.block(3,0,3,index);                                          // Angular component
		
		this->jointInertiaMatrix.block(0,0,index,index) += mass*Jv.transpose()*Jv + Jw.transpose()*I*Jw;
		                                                
		this->jointCoriolisMatrix.block(0,0,index,index) += mass*Jv.transpose()*Jdot.block(0,0,3,index)
		                                                  + Jw.transpose()*(Idot*Jw + I*Jdot.block(3,0,3,index));
		
		this->jointGravityVector.head(index) += mass*Jv.transpose()*this->gravityVector;
		
		std::vector<Link*> temp = currentLink->next_links();                                // Get the links following this one
		
		if(temp.size() != 0) candidateList.insert(candidateList.end(), temp.begin(), temp.end()); // Add them to the search list
	
		std::cout << "This is the " << currentLink->name() << " link.\n";
		*/
	}
	
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the Jacobian to a given point                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf jacobian(Joint *joint, const Eigen::Vector3f &point, const unsigned int &numberOfJoints)
{

	Eigen::MatrixXf J(6,numberOfJoints);
/*	
	Joint *currentJoint = joint;
	
	while(currentJoint != nullptr)
	{
		const unsigned int i = joint->index();
		
		if(currentJoint->is_revolute())
		{
			// J_i = [ a_i x r_i ]
			//       [    a_i    ]
		
			J.block(0,i,3,1) = currentJoint->axis().cross(point - currentJoints->pose().position()); // Linear component
			J.block(3,i,3,1) = currentJoint->axis();                                                 // Angular component
		}
		else // prismatic
		{
			// J_i = [ a_i ]
			//       [  0  ]
			
			J.block(0,i,3,1) = currentJoint->axis();                                    // Linear component
			J.block(3,i,3,1).setZero();                                                 // Angular component
		}
		
		currentJoint = currentJoint->parent_link()->joint();                                // Get the next link down the chain
	}
*/
	return J;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the time derivative of a given Jacobian                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf time_derivative(const Eigen::MatrixXf &J)
{
	if(J.rows() != 6)
	{
		throw std::invalid_argument("[ERROR] [KINEMATIC TREE] time_derivative(): Expected the Jacobian matrix to have 6 rows but it only had " + std::to_string(J.rows()) + ".");
	}
	
	Eigen::MatrixXf Jdot(6,J.cols()); Jdot.setZero();                                           // Value to be returned
/*
	for(int i = 0; i < J.cols(); i++)
	{
		for(int j = 0; j <= i; j++)
		{
			// Compute dJ(i)/dq(j)
			if(this->joint[j].is_revolute())				            // J_j = [a_j x r_j; a_j]
			{
				// qdot_j * ( a_j x (a_i x r_i) )
				Jdot(0,i) += this->qdot(j)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
				Jdot(1,i) += this->qdot(j)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
				Jdot(2,i) += this->qdot(j)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				
				if(this->joint[i].is_revolute())			            // J_i = [a_i x r_i; a_i]
				{
					// qdot_j * ( a_j x a_i )
					Jdot(3,i) += this->qdot(j)*(J(4,j)*J(5,i) - J(5,j)*J(4,i));
					Jdot(4,i) += this->qdot(j)*(J(5,j)*J(3,i) - J(3,j)*J(5,i));
					Jdot(5,i) += this->qdot(j)*(J(3,j)*J(4,i) - J(4,j)*J(3,i));
				}	
			}
			
			// Compute dJ(j)/dq(i)
			if(i != j && this->joint[j].is_revolute())			            // J_j = [a_j x r_j; a_j]
			{
				if(this->joint[i].is_revolute())			            // J_i = [a_i x r_i; a_i]
				{
					// qdot_i * ( a_i x (a_j x r_j) )
					// Jdot(0,j) += this->qdot(i)*(J(4,i)*J(2,j) - J(5,i)*J(1,j));
					// Jdot(1,j) += this->qdot(i)*(J(5,i)*J(0,j) - J(3,i)*J(2,j));
					// Jdot(2,j) += this->qdot(i)*(J(3,i)*J(1,j) - J(4,i)*J(0,j));
					Jdot(0,j) += this->qdot(i)*(J(4,j)*J(2,i) - J(5,j)*J(1,i));
					Jdot(1,j) += this->qdot(i)*(J(5,j)*J(0,i) - J(3,j)*J(2,i));
					Jdot(2,j) += this->qdot(i)*(J(3,j)*J(1,i) - J(4,j)*J(0,i));
				}
				else // this->link[i].is_prismatic()			            // J_i = [a_i ; 0]
				{
					// qdot_i * ( a_i x (a_j x r_j) )
					Jdot(0,j) += this->qdot(i)*(J(1,i)*J(2,j) - J(2,i)*J(1,j));
					Jdot(1,j) += this->qdot(i)*(J(2,i)*J(0,j) - J(0,i)*J(2,j));
					Jdot(2,j) += this->qdot(i)*(J(0,i)*J(1,j) - J(1,i)*J(0,j));
				}
			}
		}
	}
*/
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
