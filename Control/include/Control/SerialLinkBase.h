/**
 * @file   SerialLinkBase.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A base class for control of serial link robot arms.
 */

#ifndef SERIALLINKBASE_H_
#define SERIALLINKBASE_H_

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <Model/Pose.h>
#include <Model/Joint.h>
#include <Model/KinematicTree.h>                                                                    // Computes the kinematics and dynamics
#include <Math/QPSolver.h>                                                                          // Control optimisation

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

#endif
