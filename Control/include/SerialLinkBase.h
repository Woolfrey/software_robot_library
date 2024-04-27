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

template <class DataType>
class SerialLinkBase : public QPSolver<DataType>
{
	public:
		/**
		 * Constructor.
		 * @param model A pointer to the KinematicTree object to be controlled.
		 * @param endpointName The name of the endpoint on the KinematicTree to be controlled.
		 */
		SerialLinkBase(KinematicTree<DataType> *model, const std::string &endpointName);
		
		/**
		 * Compute the required joint motion to achieve the specified endpoint motion.
		 * @param endpointMotion The desired velocity or acceleration of the endpoint.
		 * @return The required joint velocity or torque to achieved the endpoint motion.
		 */
		virtual Eigen::Vector<DataType,Eigen::Dynamic>
		resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endpointMotion) = 0;
		
		/**
		 * Compute the required joint motion to track a given state for the endpoint.
		 * The function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual Eigen::Vector<DataType,Eigen::Dynamic>
		track_endpoint_trajectory(const Pose<DataType>   &desiredPose,
					  const Eigen::Vector<DataType,6> &desiredVelocity,
					  const Eigen::Vector<DataType,6> &desiredAcceleration) = 0;
		
		/**
		 * Compute the joint motion to follow a desired joint state.
		 * This function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual Eigen::Vector<DataType,Eigen::Dynamic>
		track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPosition,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVelocity,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcceleration) = 0;
		                                               
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
		bool set_cartesian_gain_format(const Eigen::Matrix<DataType,6,6> &format);
		                      
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
		bool set_max_joint_acceleration(const DataType &accel);
		
		/**
		 * @return Returns the gradient of manipulability.
		 */
		Eigen::Vector<DataType,Eigen::Dynamic> manipulability_gradient();
		
		/**
		 * Set a pointer to a function that computes the redundant task.
		 * Default set at construction is `manipulability_gradient`.
		 * It must be set every time before calling a Cartesian control function.
		 * @param task The task to be executed.
		 * @return Returns false if there were any problems.
		 */
		typedef Eigen::Vector<DataType,Eigen::Dynamic> (*functionArgument)(void);
		
		bool set_redundant_task(functionArgument &functionName);
		
		/**
		 * @return Returns a 6xn matrix for the Jacobian to the endpoint of this serial link object.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic> endpoint_jacobian()
		{ return this->_model->jacobian(this->_endpointName); }
		 
		/**
		 * Updates properties specific to this SerialLink object.
		 * NOTE You must update the state of the KinematicTree first.
		 * This is done since multiple serial link objects may exist on a single kinematic tree.
		 */
		void update_state()
		{
			this->_endpointPose   = this->_endpointFrame.link.pose()*this->_endpointFrame.offset();
			this->_jacobianMatrix = this->_model->jacobian(this->_endpointName);
			this->_forceEllipsoid = this->_jacobianMatrix*this->_jacobianMatrix.transpose();
			this->_manipulability = sqrt(this->_forceEllipsoid.determinant());
		}
		                           
	protected:
		
		DataType _controlBarrierScalar = 1.0;                                                     ///< Used in singularity avoidance
		
		DataType _jointPositionGain = 1.0;                                                        ///< On position tracking error
		
		DataType _jointDerivativeGain = 0.1;                                                      ///< On velocity tracking error
		
		DataType _manipulability;                                                                 ///< Proximity to a singularity
		
		DataType _minManipulability;                                                              ///< Used in singularity avoidance
		
		DataType _maxJointAcceleration = 10.0;                                                    ///< As it says.
		
		DataType _redundantTaskScalar = 1.0;                                                      ///< Used to increase or decrease effect of redundant task.
		
		Eigen::Matrix<DataType,6,6> _gainFormat =
		(Eigen::Matrix<DataType,6,6>(6,6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		                                     0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
		                                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
		                                     0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
		                                     0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
		                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.1).finished();            ///< Structure for the Cartesian gain matrices
		
		Eigen::Matrix<DataType,6,6> _cartesianDamping = 0.1*this->_gainFormat;                    ///< Derivative gain on endpoint velocity error
		
		Eigen::Matrix<DataType,6,6> _cartesianStiffness = 1.0*this->_gainFormat;                  ///< Proportional gain on endpoint pose error
		
		Eigen::Matrix<DataType,6,Eigen::Dynamic> _jacobianMatrix;                                 ///< Of the endpoint frame
		
		Eigen::Matrix<DataType,6,6> _forceEllipsoid;                                              ///< Jacobian multiplied with its tranpose: J*J.transpose()
		
		KinematicTree<DataType>* _model;                                                          ///< Pointer to the underlying robot model
		
		ReferenceFrame<DataType> *_endpointFrame;                                                 ///< The frame on the model controlled by this object
		
		unsigned int _controlFrequency = 100;                                                     ///< Used in certain control calculations.
		
		Eigen::Vector<DataType,Eigen::Dynamic> (*_redundantTask)(void);                           ///< Pointer to function that computes redundant task
		
		/**
		 * Computes the instantaneous limits on the joint control.
		 * @param jointNumber Which joint to compute the limits for.
		 * @return A Limit data structure.
		 */
		virtual Limits<DataType> compute_control_limits(const unsigned int &jointNumber) = 0;
	
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
SerialLinkBase<DataType>::SerialLinkBase(KinematicTree<DataType> *model, const std::string &endpointName)
					 : _model(model)
{
     // Save a pointer to the endpoint frame to be controlled by this object.
     // This will save time searching later.
     // NOTE: This will throw a runtime error if it doesn't exist ;)
     this->_endpointFrame = this->_model->find_frame(endpointName);

     /* THIS THROWS ERRORS; NEED TO FIX.
        The problem is with assigning a function pointer to a member function...
	if(not set_redundant_task(&this->manipulability_gradient))
	{
		throw std::runtime_error("[ERROR] [SERIAL LINK] Constructor: "
		                         "Unable to set the redundant task. How did that happen?");
	}
	*/
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
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gains(): "
		          << "Gains cannot be negative. Stiffness was " << stiffness << " and "
		          << "damping was " << damping << "." << std::endl;
		
		return false;
	}
	else
	{
		this->_cartesianDamping   =   damping*this->gainFormat;
		this->_cartesianStiffness = stiffness*this->gainFormat;
		
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
		          << "Matrix does not appear to be symmetric." << std::endl;
		          
		return false;
	}
	else
	{
		this->_cartesianGainFormat = format;
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
		          << proportional << " and derivative was " << derivative << "." << std::endl;
		
		return false;
	}
	else
	{
		this->_jointPositionGain = proportional;
		this->_jointVelocityGain = derivative;
		return true;
	}
}
                    
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Set the maximum permissable joint acceleration                            //     
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_max_joint_acceleration(const DataType &acceleration)
{
	if(acceleration <= 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_joint_acceleration(): "
		          << "Acceleration was " << acceleration << " but it must be positive." << std::endl;
		
		return false;
	}
	else
	{
		this->_maxJointAcceleration = acceleration;
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //              Set the joint motion for controlling the extra joints in the robot               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool SerialLinkBase<DataType>::set_redundant_task(functionArgument &functionName)
{
	unsigned int returnSize = functionName().size();                                              // Query the function (use * to de-reference)
	unsigned int numJoints  = this->_model->number_of_joints();                                    // As it says
	
	if(returnSize != numJoints)
	{
		std::cerr << "[ERROR] [SERIAL LINK] set_redundant_task(): "
		          << "This robot has " << numJoints << " joints but the vector returned "
		          << "from the redundant task has only " << returnSize <<  " elements." << std::endl;
		          
		return false;
	}
	else
	{	
		this->_redundantTask = functionName;                                                      // Assign the function
		
		return true;
	}
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the gradient of manipulability                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic> SerialLinkBase<DataType>::manipulability_gradient()
{
	// NOTE TO FUTURE SELF: Gradient with respect to first joint in a chain is always zero.
	//                      It is possible to reduce calcs if this condition can be efficiently
	//                      integrated in the loop below.
	
	Eigen::Vector<DataType,Eigen::Dynamic> gradient(this->_model->number_of_joints());             // Value to be returned
	
	Link<DataType> *currentLink = this->_endpointFrame->link;                                      // Starting link for computation
	
	Eigen::LDLT<Matrix<DataType,6,6>> JJT(this->_forceEllipsoid);                                  // Pre-compute the decomposition

	while(currentLink != nullptr)
	{
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> dJ
		= this->_model->partial_derivative(this->_jacobianMatrix,currentLink->number());      
	
		gradient(currentLink->number())
		= this->_manipulability*(JJT.solve(dJ*this->_jacobianMatrix.transpose())).trace();
		
		currentLink = currentLink->parent_link();                                                 // Get pointer to next link in chain
	}
	
	return gradient;
}

#endif

