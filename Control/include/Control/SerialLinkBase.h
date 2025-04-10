/**
 * @file    SerialLinkBase.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   A base class providing a standardised interface for all serial link robot arm controllers.
 * 
 * @details This class is designed to provide a standardised structure for all types of serial link
 *          robot arm controllers. This enables seemless interfacing between position/velocity and
 *          dynamic control.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#ifndef SERIAL_LINK_BASE_H
#define SERIAL_LINK_BASE_H

#include <Control/DataStructures.h>
#include <Model/KinematicTree.h>                                                                    // Computes the kinematics and dynamics
#include <Math/MathFunctions.h>                                                                     // Helper functions               
#include <Math/QPSolver.h>                                                                          // Convex optimisation methods

#include <Eigen/Dense>                                                                              // Matrix decomposition
#include <memory>                                                                                   // For std::shared_ptr

namespace RobotLibrary { namespace Control {

/**
 * @brief A class that provides a standard interface for all serial link robot arm controllers.
 */
class SerialLinkBase : public QPSolver<double>
{
	public:
	
		/**
		 * @brief Constructor.
		 * @param model A pointer to the KinematicTree object to be controlled.
		 * @param endpointName The name of the endpoint on the KinematicTree to be controlled.
		 */
		SerialLinkBase(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
		               const std::string &endpointName,
		               const RobotLibrary::Control::Parameters &parameters = Parameters());
		
		/**
		 * @brief Compute the required joint motion to achieve the specified endpoint motion.
		 * @param endpointMotion The desired velocity or acceleration of the endpoint.
		 * @return The required joint velocity or torque to achieved the endpoint motion.
		 */
		virtual
		Eigen::VectorXd
		resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion) = 0;
		
		/**
         * @brief Compute the joint motion required to execute the specified endpoint twist (linear & angular velocity)
         * @param twist The desired linear & angular velocity, as a 6D vector.
         * @return The required joint velocity, or joint torque.
         */
        virtual
        Eigen::VectorXd
        resolve_endpoint_twist(const Eigen::Vector<double,6> &twist) = 0;
		
		/**
		 * @brief Compute the required joint motion to track a given state for the endpoint.
		 *        The function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual
		Eigen::VectorXd
		track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                                  const Eigen::Vector<double,6> &desiredVelocity,
                                  const Eigen::Vector<double,6> &desiredAcceleration) = 0;
		
		/**
		 * @brief Compute the joint motion to follow a desired joint state.
		 *        This function will compute the feedforward + feedback control.
		 * @param desiredPose The desired pose at the current time.
		 * @param desiredVel The desired velocity at the current time.
		 * @param desiredAcc The desired acceleration at the current time.
		 * @return The required joint velocity, or joint torques.
		 */
		virtual
		Eigen::VectorXd
		track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
		                       const Eigen::VectorXd &desiredVelocity,
		                       const Eigen::VectorXd &desiredAcceleration) = 0;
		
		/**
		 * @brief Get the measure of manipulability for the current state.
		 *        The robot is in a singular configuration when manipulability = 0.
		 * @return A scalar quantity that measures proximity to a singularity.
		 */
	    double
	    manipulability() const { return _manipulability; }
		
		/**
		 * @return Returns a vector that points away from the closest singular joint configuration.
		 */
		Eigen::VectorXd
		manipulability_gradient();
		
		/**
		 * @brief Get the position and orientation of the endpoint frame relative to the base of the robot.
		 * @return A RobotLibrary::Pose object.
		 */
		RobotLibrary::Model::Pose
		endpoint_pose() const { return _endpointPose; }
		
		/**
		 * @brief Get the linear & angular velocity of the endpoint.
		 * @return A 6D Eigen::Vector object.
		 */
		Eigen::Vector<double,6>
		endpoint_velocity() const { return _jacobianMatrix*_model->joint_velocities(); }
		
		/**
         * @brief Get the Jacobian matrix (partial derivative of forward kinematics) for the endpoint being controlled.
		 * @return Returns a 6xn matrix for the Jacobian to the endpoint of this serial link object.
		 */
		Eigen::Matrix<double,6,Eigen::Dynamic> jacobian() const { return _jacobianMatrix; }
     
		/**
		 * @brief Updates properties specific to this controller.
		 *        NOTE: underlying KinematicTree model MUST be updated first.
		 *        This is because multiple serial link objects may exist on a single kinematic tree.
		 */
		void
		update();
		
	    /**
	     * @brief Set parameters such as control gains, control frequency, etc.
	     * @param options a RobotLibrary::Control::Options data structure. Contains things like control gains, frequency, etc.
	     * @return False if there is something wrong with the arguments.
	     */
	    void
	    set_control_parameters(const Parameters &parameters);
	    
		/**
		 * @brief Assign the redundant task for use in the next control calculation.
		 *        NOTE: In kinematic control, this is a joint velocity vector. In dyamic control, it is a joint torque vector.
		 * @param task A vector for the joint motion to be executed using extra degrees of freedom in a redundant robot.
		 * @return True if successful, false otherwise.
		 */
		bool
		set_redundant_task(const Eigen::VectorXd &task);
		
		/**
		 * @brief Check whether the robot is in a singular configuration or not.
		 * @return True if singular, false if not.
		 */
		bool
		is_singular() { return (_manipulability < _minManipulability) ? true : false; }
		
		/**
		 * @brief Get a pointer to the model this controller uses.
		 */
		std::shared_ptr<RobotLibrary::Model::KinematicTree>
		model() const { return _model; }
		
	    /**
	     * @brief Get the control frequency.
	     */
	    double
	    frequency() const { return _controlFrequency; }
		                           
	protected:
		
        bool _redundantTaskSet = false;                                                             ///< When false, a redundant robot will autonomously reconfigure away from a singularity
		
		double _jointPositionGain = 100.0;                                                          ///< On position tracking error
		
		double _jointVelocityGain = 10.0;                                                           ///< On velocity tracking error
		
		double _manipulability;                                                                     ///< Proximity to a singularity
		
		double _minManipulability = 5e-03;                                                          ///< Used in singularity avoidance
		
		double _maxJointAcceleration = 0.5;                                                         ///< As it says.

		double _controlFrequency = 100.0;                                                           ///< Used in certain control calculations.
				
		Eigen::Matrix<double,6,6> _cartesianStiffness =
		(Eigen::Matrix<double,6,6>(6,6) << 10.0,  0.0,  0.0, 0.0, 0.0, 0.0,
		                                    0.0, 10.0,  0.0, 0.0, 0.0, 0.0,
		                                    0.0,  0.0, 10.0, 0.0, 0.0, 0.0,
		                                    0.0,  0.0,  0.0, 2.0, 0.0, 0.0,
		                                    0.0,  0.0,  0.0, 0.0, 2.0, 0.0,
		                                    0.0,  0.0,  0.0, 0.0, 0.0, 2.0).finished();             ///< Gain on the endpoint pose error
		
		Eigen::Matrix<double,6,6> _cartesianDamping = 0.1*_cartesianStiffness;                      ///< Gain the endpoint velocity error
		
		Eigen::Matrix<double,6,Eigen::Dynamic> _jacobianMatrix;                                     ///< Of the endpoint frame
		
		Eigen::Matrix<double,6,6> _forceEllipsoid;                                                  ///< Jacobian multiplied with its tranpose: J*J.transpose()
		
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _constraintMatrix;                      ///< Used in optimisation during Cartesian control
		
		Eigen::VectorXd _constraintVector;                                                          ///< Used in optimisation during Cartesian control
		
		Eigen::VectorXd _redundantTask;                                                             ///< Used to control null space of redundant robots
		
		std::shared_ptr<RobotLibrary::Model::KinematicTree> _model;                                 ///< Pointer to the underlying robot model
		
		RobotLibrary::Model::Pose _endpointPose;                                                    ///< Class denoting position and orientation of endpoint frame
		
		RobotLibrary::Model::ReferenceFrame *_endpointFrame;                                        ///< Pointer to frame controlled in underlying model
	
		/**
		 * @brief Computes the instantaneous limits on the joint control.
		 * @param jointNumber Which joint to compute the limits for.
		 * @return A Limit data structure.
		 */
		virtual
		RobotLibrary::Model::Limits
		compute_control_limits(const unsigned int &jointNumber) = 0;
};                                                                                                  // Semicolon needed after a class declaration

} } // namespace

#endif
