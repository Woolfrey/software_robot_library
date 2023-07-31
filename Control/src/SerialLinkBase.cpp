#include <SerialLinkBase.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gains for Cartesian feedback control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLinkBase::set_cartesian_gains(const float &stiffness, const float &damping)
{
	if(stiffness < 0 or damping < 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gains(): "
		          << "Gains cannot be negative. Stiffness was " << stiffness << " and "
		          << "damping was " << damping << ".\n";
		
		return false;
	}
	else
	{
		this->K = stiffness*this->gainFormat;
		this->D = damping*this->gainFormat;
		
		return false;
	} 
}
              
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set the structure of the Cartesian gain matrices                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLinkBase::set_cartesian_gain_format(const Eigen::Matrix<float,6,6> &format)
{
	if((format - format.transpose()).norm() < 1e-04)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gain_format(): "
		          << "Matrix does not appear to be symmetric.\n";
		          
		return false;
	}
	else
	{
		this->gainFormat = format;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set the gains for the joint feedback control                            //
///////////////////////////////////////////////////////////////////////////////////////////////////               
bool SerialLinkBase::set_joint_gains(const float &proportional, const float &derivative)
{
	if(proportional < 0 or derivative < 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gain_format(): "
		          << "Gains cannot be negative. Proportional gain was "
		          << proportional << " and derivative was " << derivative << ".\n";
		
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
 //                     Set the maximum permissable joint acceleration                            //     
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLinkBase::set_max_joint_acceleration(const float &acceleration)
{
	if(acceleration <= 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_joint_acceleration(): "
		          << "Acceleration was " << acceleration << " but it must be positive.\n";
		
		return false;
	}
	else
	{
		this->maxJointAcceleration = acceleration;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //              Set the joint motion for controlling the extra joints in the robot               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool SerialLinkBase::set_redundant_task(const Eigen::VectorXf &task)
{
	if(task.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_redundant_task(): "
		          << "This robot has " << this->numJoints << " joints but "
		          << "the input argument had " << task.size() << " elements.\n";
		
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
}
