#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <KinematicTree.h>                                                                          // Computes the kinematics and dynamics
#include <QPSolver.h>                                                                               // Control optimisation

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;

template <typename DataType>
Matrix<DataType,6,Dynamic> endpoint_jacobian();

template <class DataType>
class SerialLinkBase : public KinematicTree<DataType>, 
                       public QPSolver<DataType>
{
	public:
		SerialLinkBase(const string &pathToURDF,
		               const string &endpointName);
		
		// This virtual functions must be defined in a derived class
		virtual Vector<DataType,Dynamic> resolve_endpoint_motion(const Vector<DataType,6> &endpointMotion) = 0; // Solve the joint motion for given endpoint motion
		
		virtual Vector<DataType,Dynamic> track_endpoint_trajectory(const Pose<DataType> &desiredPose,
							                   const Vector<DataType,6> &desiredVel,
							                   const Vector<DataType,6> &desiredAccel) = 0;
							  
		virtual Vector<DataType,Dynamic> track_joint_trajectory(const Vector<DataType,Dynamic> &desiredPos,
		                                                        const Vector<DataType,Dynamic> &desiredVel,
		                                                        const Vector<DataType,Dynamic> &desiredAcc) = 0;
		                                               
		// Methods
		bool set_cartesian_gains(const DataType &stiffness, const DataType &damping);
		                         
		bool set_cartesian_gain_format(const Matrix<DataType,6,6> &format);
		                      
		bool set_joint_gains(const DataType &proportional, const DataType &derivative);
		                     
		bool set_max_joint_accel(const DataType &accel);
		
		bool set_redundant_task(const Vector<DataType,Dynamic> &task);
		
		Matrix<DataType,6,Dynamic> endpoint_jacobian() { return this->jacobian(this->_endpointName); } // Need 'this->' or it won't compile
				                                       
	protected:
	
		// General properties
		
		bool redundantTaskSet = false;
		
		DataType frequency = 100;                                                           // Control frequency
		
		DataType maxJointAccel  = 10;                                                       // Maximum joint acceleration
		
		string _endpointName = "unnamed";
		
		// Properties for Cartesian control
		
		Matrix<DataType,6,6> gainFormat =
		(Matrix<DataType,Dynamic,Dynamic>(6,6) << 1, 0, 0,   0,   0,   0,
		                                          0, 1, 0,   0,   0,   0,
		                                          0, 0, 1,   0,   0,   0,
		                                          0, 0, 0, 0.1,   0,   0,
		                                          0, 0, 0,   0, 0.1,   0,
		                                          0, 0, 0,   0,   0, 0.1).finished();
		
		Matrix<DataType,6,6> K = 1.0*this->gainFormat;
		
		Matrix<DataType,6,6> D = 0.1*this->gainFormat;
		
		Vector<DataType,Dynamic> redundantTask;                                             // For kinematically redundant robots
		
		// Properties for joint control
		
		DataType kp = 1.0;                                                                  // Proportional gain for joint control
		DataType kd = 0.1;                                                                  // Derivative gain for proportional control

		// Methods
		
		virtual bool compute_control_limits(DataType &lower,
		                                    DataType &upper,
		                                    const unsigned int &jointNumber) = 0;
		
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
SerialLinkBase<DataType>::SerialLinkBase(const string &pathToURDF, const string &endpointName)
					 :
                                         KinematicTree<DataType>(pathToURDF)
{
	auto container = this->referenceFrameList.find(endpointName);
	
	if(container == this->referenceFrameList.end())
	{
		throw invalid_argument("[ERROR] [SERIAL LINK CONTROL] Constructor: "
		                       "Could not find '" + endpointName + "' as a reference frame on the robot.");
	}
	else this->_endpointName = endpointName;                                      
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gains for Cartesian feedback control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_cartesian_gains(const DataType &stiffness,
                                                   const DataType &damping)
{
	if(stiffness < 0 or damping < 0)
	{
		cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gains(): "
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
		cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gain_format(): "
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
		cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gain_format(): "
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
	if(accel <= 0)
	{
		cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_joint_acceleration(): "
		          << "Acceleration was " << accel << " but it must be positive.\n";
		
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
bool SerialLinkBase<DataType>::set_redundant_task(const Vector<DataType,Dynamic> &task)
{
	if(task.size() != this->numJoints)
	{
		cerr << "[ERROR] [SERIAL LINK CONTROL] set_redundant_task(): "
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
