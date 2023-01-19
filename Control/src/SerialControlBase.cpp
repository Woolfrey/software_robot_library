#include <SerialControlBase.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialControlBase::SerialControlBase(const std::vector<RigidBody>  &links,
                                     const std::vector<Joint> &joints,
				     const float &controlFrequency) :
				     SerialLink(links,joints),
				     hertz(controlFrequency),
				     dt(1/this->hertz)
{
	this->gainFormat << 1.0,  0.0,  0.0,  0.00,  0.00,  0.00,
	                    0.0,  1.0,  0.0,  0.00,  0.00,  0.00,
		            0.0,  0.0,  1.0,  0.00,  0.00,  0.00,
			    0.0,  0.0,  0.0,  0.10,  0.01,  0.00,
			    0.0,  0.0,  0.0,  0.01,  0.10,  0.00,
			    0.0,  0.0,  0.0,  0.00,  0.00,  0.10;

	this->Kc = 1.0*this->gainFormat;
	this->Dc = 0.1*this->gainFormat;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Set the gains for feedback control in Cartesian space                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialControlBase::set_cartesian_gains(const float &stiffness, const float &damping)
{
	if(stiffness < 0 or damping < 0)
	{
		std::cerr << "[ERROR] [SERIAL-LINK CONTROL] set_cartesian_gains(): "
		          << "Gains cannot be negative! "
		          << "The stiffness gain was " << stiffness << " "
			  << "and the damping gain was " << damping << "." << std::endl;
		
		return false;
	}
	else
	{
		this->Kc = stiffness*this->gainFormat;
		this->Dc = stiffness*this->gainFormat;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set the gains for feedback control in joint space                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialControlBase::set_joint_gains(const float &proportional, const float &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [SERIAL-LINK CONTROL] set_joint_gains(): "
			  << "Gains cannot be negative! "
		          << "The porportional gain was " << proportional  << " "
			  << "and the derivative gain was " << derivative << "." << std::endl;
			  
		return false;
	}
	else
	{
		this->kp = proportional;
		this->kd = derivative;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Set the maximum acceleration for the joints                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialControlBase::set_max_acceleration(const float &accel)
{
	if(accel <= 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_acceleration(): "
		          << "Input argument was " << accel << ", but cannot be negative." << std::endl;
		
		return false;
	}
	else
	{
		this->maxAccel = accel;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Add a redundant task when computing Cartesian control                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialControlBase::set_redundant_task(const Eigen::VectorXf &task)
{
	if(this->n <= 6)
	{
		std::cout << "[WARNING] [SERIAL LINK CONTROL] set_redundant_task(): "
		          << "This robot has only " << this->n << " joints and therefore has no kinematic "
		          << "redundancy.Redundant task not set." << std::endl;
				  
		this->redundantTaskSet = false;
		return false;
	}
	else if(task.size() != this->n)
	{
		std::cout << "[ERROR] [SERIAL LINK CONTROL] set_redundant_task(): " 
                          << "This robot has " << this->n << " joints, but the input argument had "
		          << task.size() << " elements." << std::endl;
		
		this->redundantTaskSet = false;
		return false;
	}
	else
	{
		this->redundantTask = task;
		this->redundantTaskSet = true;
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Compute the derivative of the joint penalty function                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
float SerialControlBase::compute_joint_penalty_derivative(const unsigned int &jointNumber)
{
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based scheme
	// for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	if(jointNumber > this->n)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] compute_joint_penalty_derivative(): "
		          << "This robot only has " << this->n << " joints but your input was "
			  << jointNumber << "." << std::endl;
			  
		return 0.0;
	}
	else
	{
		std::cout << "[ERROR] [SERIAL LINK CONTROL] compute_joint_penalty derivative(): "
		          << "This function hasn't been programmed yet!" << std::endl;
		
		return 0;
//		return ( pow(this->pLim[i][1] - this->pLim[i][0], 2.0) * (2*this->q[i] - this->pLim[i][1] - this->pLim[i][0]) ) /
//		       ( 4 * pow(this->pLim[i][1] - this->q[i],2.0) * pow(this->q[i] - this->pLim[i][0],2.0) );
	
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //            Compute a penalty function based on proximity to joint position limits             //
///////////////////////////////////////////////////////////////////////////////////////////////////
float SerialControlBase::compute_joint_penalty_function(const unsigned int &jointNumber)
{
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based scheme
	// for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	// NOTE: Minimum of penalty function is 1, so subtract 1 if you want 0 penalty at midpoint
	
	if(jointNumber > this->n)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] compute_joint_penalty_function(): "
		          << "This robot only has " << this->n << " joints but your input was "
		          << jointNumber << "." << std::endl;
		          
		return 1.0;
	}
	else if(compute_joint_penalty_derivative(jointNumber)*this->qdot(jointNumber) > 0)          // Moving toward joint limit
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] compute_joint_penalty_function(): "
		          << "This function hasn't been programmed yet!" << std::endl;
		
		return 1.0;
//		return pow(this->pLim[i][1] - this->pLim[i][0],2.0) /
//		       (4*(this->pLim[i][1] - this->q[i])*(this->q[i] - this->pLim[i][0]));         // Return penalty
	}
	else return 1.0;                                                                            // Don't penalise if moving away
}
