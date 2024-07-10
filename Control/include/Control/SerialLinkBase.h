/**
 * @file   SerialLinkBase.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A base class for control of serial link robot arms.
 */

#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include "../Model/KinematicTree.h"                                                                 // Computes the kinematics and dynamics
#include "../Math/QPSolver.h"                                                                       // Control optimisation

#include <Eigen/Dense>                                                                              // Matrix decomposition

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
		track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
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
		bool set_cartesian_gains(Eigen::Matrix<DataType,6,6> &stiffness,
		                         Eigen::Matrix<DataType,6,6> &damping);

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
		 * Get the measure of manipulability for the current state.
		 * The robot is in a singular configuration when manipulability = 0.
		 * @return A scalar quantity that measures proximity to a singularity.
		 */
	     DataType manipulability() const { return this->_manipulability; }
		
		/**
		 * @return Returns a vector that points away from the closest singular joint configuration.
		 */
		Eigen::Vector<DataType,Eigen::Dynamic> manipulability_gradient();
		
		/**
		 * Get the position and orientation of the endpoint frame relative to the base of the robot.
		 * @return A RobotLibrary::Pose object.
		 */
		Pose<DataType> endpoint_pose() const { return this->_endpointPose; }
		
		/**
           * Get the Jacobian matrix (partial derivative of forward kinematics) for the endpoint being controlled.
		 * @return Returns a 6xn matrix for the Jacobian to the endpoint of this serial link object.
		 */
		Eigen::Matrix<DataType,6,Eigen::Dynamic> jacobian() const { return this->_jacobianMatrix; }
     
		/**
		 * Updates properties specific to this controller.
		 * NOTE: underlying KinematicTree model MUST be updated first.
		 * This is because multiple serial link objects may exist on a single kinematic tree.
		 */
		void update();
		
		/**
		 * Assign the redundant task for use in the next control calculation.
		 * NOTE: In kinematic control, this is a joint velocity vector. In dyamic control, it is a joint torque vector.
		 * @param task A vector for the joint motion to be executed using extra degrees of freedom in a redundant robot.
		 * @return True if successful, false otherwise.
		 */
		bool set_redundant_task(const Eigen::Vector<DataType,Eigen::Dynamic> &task);
		
		/**
		 * Check whether the robot is in a singular configuration or not.
		 * @return True if singular, false if not.
		 */
		bool is_singular() { return (this->_manipulability < this->_minManipulability) ? true : false; }
		                           
	protected:
		
        bool _redundantTaskSet = false;                                                             ///< When false, a redundant robot will autonomously reconfigure away from a singularity
		
		DataType _jointPositionGain = 10.0;                                                         ///< On position tracking error
		
		DataType _jointDerivativeGain = 1.0;                                                        ///< On velocity tracking error
		
		DataType _manipulability;                                                                   ///< Proximity to a singularity
		
		DataType _minManipulability = 5e-03;                                                        ///< Used in singularity avoidance
		
		DataType _maxJointAcceleration = 0.5;                                                       ///< As it says.
		
		Eigen::Matrix<DataType,6,6> _cartesianStiffness
		= (Eigen::Matrix<DataType,6,6>(6,6) << 10.0,  0.0,  0.0, 0.0, 0.0, 0.0,
		                                        0.0, 10.0,  0.0, 0.0, 0.0, 0.0,
		                                        0.0,  0.0, 10.0, 0.0, 0.0, 0.0,
		                                        0.0,  0.0,  0.0, 2.0, 0.0, 0.0,
		                                        0.0,  0.0,  0.0, 0.0, 2.0, 0.0,
		                                        0.0,  0.0,  0.0, 0.0, 0.0, 2.0).finished();         ///< Gain on the endpoint pose error
		
		Eigen::Matrix<DataType,6,6> _cartesianDamping = 0.1*_cartesianStiffness;                    ///< Gain the endpoint velocity error
		
		Eigen::Matrix<DataType,6,Eigen::Dynamic> _jacobianMatrix;                                   ///< Of the endpoint frame
		
		Eigen::Matrix<DataType,6,6> _forceEllipsoid;                                                ///< Jacobian multiplied with its tranpose: J*J.transpose()
		
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _constraintMatrix;                    ///< Used in optimisation during Cartesian control
		
		Eigen::Vector<DataType,Eigen::Dynamic> _constraintVector;                                   ///< Used in optimisation during Cartesian control
		
		Eigen::Vector<DataType,Eigen::Dynamic> _redundantTask;                                      ///< Used to control null space of redundant robots
		
		KinematicTree<DataType>* _model;                                                            ///< Pointer to the underlying robot model
		
		Pose<DataType> _endpointPose;                                                               ///< Class denoting position and orientation of endpoint frame
		
		ReferenceFrame<DataType> *_endpointFrame;                                                   ///< Pointer to frame controlled in underlying model
		
		DataType _controlFrequency = 100;                                                           ///< Used in certain control calculations.
	
		/**
		 * Computes the instantaneous limits on the joint control.
		 * @param jointNumber Which joint to compute the limits for.
		 * @return A Limit data structure.
		 */
		virtual Limits<DataType> compute_control_limits(const unsigned int &jointNumber) = 0;
	
};                                                                                                  // Semicolon needed after a class declaration

using SerialLinkBase_f = SerialLinkBase<float>;
using SerialLinkBase_d = SerialLinkBase<double>;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
SerialLinkBase<DataType>::SerialLinkBase(KinematicTree<DataType> *model, const std::string &endpointName)
					               : _model(model)
{     
     // Record pointer to the endpoint frame to be controlled so we don't need to search for it later.   
     // NOTE: This will throw a runtime error if it doesn't exist.
     
     this->_endpointFrame = this->_model->find_frame(endpointName);

     // Resize dimensions of inequality constraints for the QP solver: B*qdot < z, where:
     //
     // B = [      I    ]   z = [    qdot_max  ]
     //     [     -I    ]       [   -qdot_min  ]
     //     [  (dm/dq)' ]       [  (m - m_min) ]
     
     unsigned int n = this->_model->number_of_joints();
     
     this->_constraintMatrix.resize(2*n+1,n);
     this->_constraintMatrix.block(0,0,n,n).setIdentity();
     this->_constraintMatrix.block(n,0,n,n) = -this->_constraintMatrix.block(0,0,n,n);
//   this->_constraintMatrix.row(2*n) = this->manipulability_gradient(); <-- Needs to be set in the control loop

     this->_constraintVector.resize(2*n+1);
//   this->_constraintVector.block(0,0,n,1) =  upperBound; <-- Needs to be set in the control loop
//   this->_constraintVector.block(n,0,n,1) = -lowerBound; <-- Needs to be set in the control loop
//   this->_constraintVector(2*n) = this->_manipulability - this->_minManipulability; <-- Needs to be set in the control loop
     
     std::cout << "[INFO] [SERIAL LINK CONTROL] Controlling the '" << endpointName << "' frame on the '" 
               << this->_model->name() << "' robot.\n";
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Update properties specific to this controller                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
void SerialLinkBase<DataType>::update()
{
	this->_endpointPose = this->_endpointFrame->link->pose() * this->_endpointFrame->relativePose;  // Compute new endpoint pose
	                      
    this->_jacobianMatrix = this->_model->jacobian(this->_endpointFrame);                           // Jacobian for the endpoint
	                      
	this->_forceEllipsoid = this->_jacobianMatrix*this->_jacobianMatrix.transpose();                // Used for certain calculations
	
	DataType temp = sqrt(this->_forceEllipsoid.determinant());                                      // Proximity to a singularity
	
	this->_manipulability = ( temp < 0 or std::isnan(temp)) ? 0.0 : temp;                           // Rounding error can mean manipulability is negative or nan
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gains for Cartesian feedback control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
bool SerialLinkBase<DataType>::set_cartesian_gains(Eigen::Matrix<DataType,6,6> &stiffness,
                                                   Eigen::Matrix<DataType,6,6> &damping)
{
     if(not is_positive_definite(stiffness))
     {
          std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gains(): "
                    << "The stiffness matrix was not positive definite.\n";
          
          return false;
     }
     else if(not is_positive_definite(damping))
     {
          std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_cartesian_gains(): "
                    << "The damping matrix was not positive definite.\n";
          
          return false;
     }
     else
     {
          this->_cartesianStiffness = stiffness;
          this->_cartesianDamping   = damping;
     
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
		          << "Input was " << acceleration << " but it must be positive.\n";
		
		return false;
	}
	else
	{
		this->_maxJointAcceleration = acceleration;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Set a secondary task to be executed by a redundant robot arm                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool SerialLinkBase<DataType>::set_redundant_task(const Eigen::Vector<DataType, Eigen::Dynamic> &task)
{
     if(task.size() != this->_model->number_of_joints())
     {
          std::cerr << "[ERROR] [SERIAL LINK] set_redundant_task(): "
                    << "This robot model has " << this->_model->number_of_joints() << " joints, "
                    << "but you assigned a task with " << task.size() << " elements.\n";
          
          return false;
     }
     else
     {
          this->_redundantTask = task;
          this->_redundantTaskSet = true;
          
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

    Eigen::Vector<DataType,Eigen::Dynamic> gradient(this->_model->number_of_joints());              // Value to be returned
 
    gradient.setZero();                                                                             // Any branching chains off of this one must be zero

    Eigen::LDLT<Eigen::Matrix<DataType,6,6>> JJT(this->_forceEllipsoid);                            // Pre-compute the decomposition

    Link<DataType> *currentLink = this->_endpointFrame->link;                                       // Starting link for computation

    while(currentLink != nullptr)
    {
	    Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> dJ
	    = this->_model->partial_derivative(this->_jacobianMatrix,currentLink->number());
	    
	    gradient(currentLink->number())
	    = this->_manipulability*(JJT.solve(dJ*this->_jacobianMatrix.transpose())).trace();
	    
	    currentLink = currentLink->parent_link();                                                   // Get pointer to next link in chain
    }

    gradient(0) = 0.0;

    return gradient;
}

#endif

