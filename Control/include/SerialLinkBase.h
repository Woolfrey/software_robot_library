#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <KinematicTree.h>                                                                          // Computes the kinematics and dynamics
#include <QPSolver.h>                                                                               // Control optimisation

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;

template <class DataType>
class SerialLinkBase : public KinematicTree<DataType>, 
                       public QPSolver<DataType>
{
	public:
		SerialLinkBase(const std::string &pathToURDF) : KinematicTree(pathToURDF);
		
		// This virtual functions must be defined in a derived class
		virtual Vector<DataType,Dynamic> resolve_endpoint_motion(const Vector<DataType,6> &endpointMotion) = 0; // Solve the joint motion for given endpoint motion
		
		virtual Vector<DataType,Dynamic> track_endpoint_trajectory(const Pose<DataType>     &pose,
							                   const Vector<DataType,6> &velocity,
							                   const Vector<DataType,6> &acceleration) = 0;
							  
		virtual Vector<DataType,Dynamic> track_joint_trajectory(const Vector<DataType,Dynamic> &position,
		                                                        const Vector<DataType,Dynamic> &velocity,
		                                                        const Vector<DataType,Dynamic> &acceleration) = 0;
		                                               
		// Methods
		bool set_cartesian_gains(const DataType &stiffness, const DataType &damping);
		                         
		bool set_cartesian_gain_format(const Matrix<DataType,6,6> &format);
		                      
		bool set_joint_gains(const DataType &proportional, const DataType &derivative);
		                     
		bool set_max_joint_acceleration(const DataType &acceleration);
		
		bool set_redundant_task(const Vector<DataType,Dynamic> &task);
		                                       
	protected:
	
		// General properties
		
		bool redundantTaskSet = false;
		
		DataType frequency = 100;                                                           // Control frequency
		
		DataType maxJointAccel  = 10;                                                       // Maximum joint acceleration
		
		// Properties for Cartesian control
		
		Matrix<DataType,6,6> gainFormat =(Matrix<DataType,6,6> << 1, 0, 0,   0,   0,   0,
		                                                          0, 1, 0,   0,   0,   0,
		                                                          0, 0, 1,   0,   0,   0,
		                                                          0, 0, 0, 0.1,   0,   0,
		                                                          0, 0, 0,   0, 0.1,   0,
		                                                          0, 0, 0,   0,   0, 0.1).finished();
		
		Matrix<DataType,6,6> K = 1.0*this->gainFormat;
		
		Matrix<DataType,6,6> D = 0.1*this->gainFormat;
		
		// Properties for joint control
		
		DataType kp = 1.0;                                                                  // Proportional gain for joint control
		DataType kd = 0.1;                                                                  // Derivative gain for proportional control

		// Methods
		
		virtual bool compute_control_limits(DataType &lower,
		                                    DataType &upper,
		                                    const unsigned int &jointNumber) = 0;
		
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gains for Cartesian feedback control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_cartesian_gains(const DataType &stiffness,
                                                   const DataType &damping)
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
		this->D =   damping*this->gainFormat;
		this->K = stiffness*this->gainFormat;
		
		return false;
	} 
}
              
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set the structure of the Cartesian gain matrices                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_cartesian_gain_format(const Matrix<DataType,6,6> &format)
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
template <class DataType>         
bool SerialLinkBase<DataType>::set_joint_gains(const DataType &proportional,
                                               const DataType &derivative)
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
		this->kd = derivative;
		this->kp = proportional;
		
		return true;
	}
}
                    
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Set the maximum permissable joint acceleration                            //     
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_max_joint_accel(const DataType &accel)
{
	if(acceleration <= 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_joint_acceleration(): "
		          << "Acceleration was " << acceleration << " but it must be positive.\n";
		
		return false;
	}
	else
	{
		this->maxJointAccel = accel;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //              Set the joint motion for controlling the extra joints in the robot               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool SerialLinkBase<DataType>::set_redundant_task(const Eigen::VectorXf &task)
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
		this->redundantTask    = task;
		this->redundantTaskSet = true;
		
		return true;
	}
}

#endif
