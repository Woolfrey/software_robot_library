/**
 * @file   SerialLinkBase.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A base class for control of serial link mechanisms.
 */

#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <KinematicTree.h>                                                                          // Computes the kinematics and dynamics
#include <QPSolver.h>                                                                               // Control optimisation

using namespace Eigen;                                                                              // Eigen::Matrix, Eigen::Vector
using namespace std;

template <class DataType>
class SerialLinkBase : public QPSolver<DataType>
{
	public:
		/**
		 * Constructor.
		 * @param model A pointer to the KinematicTree object to be controlled.
		 * @param endpointName The name of the endpoint on the KinematicTree to be controlled.
		 */
		SerialLinkBase(KinematicTree<DataType> *model,
		               const string            &endpointName);
		
		/**
		 * Compute the required joint motion to achieve the specified endpoint motion.
		 * @param endpointMotion The desired velocity or acceleration of the endpoint.
		 * @return The required joint velocity or torque to achieved the endpoint motion.
		 */
		virtual Vector<DataType,Dynamic> resolve_endpoint_motion(const Vector<DataType,6> &endpointMotion) = 0; // Solve the joint motion for given endpoint motion
		
		/**
		 * Compute the required joint motion to track a given state for the endpoint.
		 * The function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual Vector<DataType,Dynamic> track_endpoint_trajectory(const Pose<DataType>     &desiredPose,
							                   const Vector<DataType,6> &desiredVel,
							                   const Vector<DataType,6> &desiredAccel) = 0;
		
		/**
		 * Compute the joint motion to follow a desired joint state.
		 * This function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual Vector<DataType,Dynamic> track_joint_trajectory(const Vector<DataType,Dynamic> &desiredPos,
		                                                        const Vector<DataType,Dynamic> &desiredVel,
		                                                        const Vector<DataType,Dynamic> &desiredAcc) = 0;
		                                               
		/**
		 * Set the gains for Cartesian feedback control.
		 * @param stiffness Proportional gain on the pose error.
		 * @param damping Derivative gain on the velocity error.
		 * @return Returns false if there was a problem.
		 */
		bool set_cartesian_gains(const DataType &stiffness, const DataType &damping);
		       
		/**
		 * Set the structure for the gain matrix in Cartesian feedback.
		 * This matrix is scaled by the values used in 'set_cartesian_gains()'.
		 * @param format A 6x6, positive-definite matrix.
		 * @return Returns false if there were any problems.
		 */            
		bool set_cartesian_gain_format(const Matrix<DataType,6,6> &format);
		                      
		/**
		 * Set the feedback gains for joint control.
		 * @param proportional Gain on position error.
		 * @param derivative Gain on the velocity error.
		 * @return Returns false if there was an issue.
		 */
		bool set_joint_gains(const DataType &proportional, const DataType &derivative);
		
		/**
		 * Set the maximum permissable joint acceleration.
		 * @param accel The maximum joint acceleration.
		 * @return Returns false if there was a problem.
		 */                 
		bool set_max_joint_accel(const DataType &accel);
		
		/**
		 * Set the redundant task for controlling the endpoint of a redundant robot.
		 * It must be set every time before calling a Cartesian control function.
		 * @param task The task to be executed.
		 * @return Returns false if there were any problems.
		 */
		bool set_redundant_task(const Vector<DataType,Dynamic> &task);
		
		/**
		 * @return Returns a 6xn matrix for the Jacobian to the endpoint of this serial link object.
		 */
		Matrix<DataType,6,Dynamic> endpoint_jacobian() { return this->_model->jacobian(this->_endpointName); }
		
		/**
		 * @return Returns the gradient of manipulability.
		 */
		 Vector<DataType,Dynamic> manipulability_gradient();
		                           
	protected:
		
		bool redundantTaskSet = false;                                                      ///< Reset every control loop.
		
		DataType frequency = 100;                                                           ///< Used in certain control functions.
		
		DataType maxJointAccel = 10;                                                        ///< As it says on the label.

		DataType kp = 1.0;                                                                  ///< Proportional gain for joint control
		
		DataType kd = 0.1;                                                                  ///< Derivative gain for proportional control
			
		KinematicTree<DataType>* _robot;                                                    ///< Pointer to underlying robot model.
		
		ReferenceFrame<DataType>* _endpointFrame;                                           ///< Pointer to endpoint so we don't have to look it up every time
		
		string _endpointName = "unnamed";                                                   ///< Name of the endpoint of this serial link chain.
		
		Matrix<DataType,6,6> gainFormat =
		(Matrix<DataType,Dynamic,Dynamic>(6,6) << 1, 0, 0,   0,   0,   0,
		                                          0, 1, 0,   0,   0,   0,
		                                          0, 0, 1,   0,   0,   0,
		                                          0, 0, 0, 0.1,   0,   0,
		                                          0, 0, 0,   0, 0.1,   0,
		                                          0, 0, 0,   0,   0, 0.1).finished();       ///< Structure of the Cartesian feedback gains.
		
		Matrix<DataType,6,6> K = 1.0*this->gainFormat;                                      ///< Proportional gain on pose error for endpoint.
		
		Matrix<DataType,6,6> D = 0.1*this->gainFormat;                                      ///< Gain on velocity error for endpoint.
		
		Matrix<DataType,6,Dynamic> J;                                                       ///< Jacobian matrix
		
		Vector<DataType,Dynamic> redundantTask;                                             ///< Secondary task when controlling endpoint of redundant robot.
		
		virtual bool compute_control_limits(DataType &lower,
		                                    DataType &upper,
		                                    const unsigned int &jointNumber) = 0;           ///< Instantaneous limits on joint motion when controlling robot.
		
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
SerialLinkBase<DataType>::SerialLinkBase(KinematicTree<DataType> *robot, const string &endpointName)
					 :
                                         _robot(robot)
{
	auto container = this->_robot->referenceFrameList.find(endpointName);
	
	if(container == this->_robot->referenceFrameList.end())
	{
		throw invalid_argument("[ERROR] [SERIAL LINK CONTROL] Constructor: "
		                       "Could not find '" + endpointName + "' as a reference frame on the robot.");
	}
	else
	{
		this->_endPointFrame = container->second;
		this->_endpointName = endpointName;
	}                                     
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
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the gradient of manipulability                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,Dynamic> SerialLinkBase<DataType>::manipulability_gradient()
{
	Vector<DataType,Dynamic> gradient(this->_robot.num_joints()); gradient.setZero();           // Value to be returned
	
	LDLT<Matrix<DataType,6,6>> JJt(this->J*this->J.transpose());                                // Decompose matrix for later use

	Link<DataType> *currentLink = this->_endpointFrame->link;
	
	while(currentLink != nullptr)
	{
		unsigned int jointNumber = currentLink->number();
		
		Matrix<DataType,6,Dynamic> dJ = this->_robot->partial_derivative(this->J,jointNumber);     // Partial derivative of Jacobian
		
		gradient(jointNumber) = this->_manipulability*(JJt.solve(dJ*this->J.transpose())).trace(); // Derivative w.r.t a single joint
		
		currentLink = currentLink->parent_link();                                                  // Get pointer to next link in chain
	}
	
	return gradient;
}

#endif
