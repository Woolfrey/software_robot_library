/**
 * @file    SerialLinkImpedance.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    August 2025
 * @version 1.0
 *
 * @brief   Implements control laws for joint and Cartesian impedance control.
 * 
 * @details This class implements dynamic-free joint and Cartesian impedance control.
 *          It deliberately avoids the use of the robot inertia because of poor modeling in many
 *          robot arms. The SerialLinkDynamics should be used if a more accurate model is available.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 *
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#include <Control/SerialLinkImpedance.h>

namespace RobotLibrary { namespace Control {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
SerialLinkImpedance::SerialLinkImpedance(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                                         const std::string &endpointName,
                                         const RobotLibrary::Control::SerialLinkParameters &parameters)
: SerialLinkBase(model, endpointName, parameters)
{
    unsigned int n = _model->number_of_joints();
    
    _desiredConfiguration.resize(n);
    
    for (int i = 0; i < n; ++i)
    {
        RobotLibrary::Model::Limits limits = _model->link(i)->joint().position_limits();
        
        _desiredConfiguration(i) = (limits.lower + limits.upper) / 2.0;
    }
    
    std::cout << "[INFO] [SERIAL LINK IMPEDANCE] "
              << "Performing TORQUE control on the " + _model->name() + " robot.\n";
    
    std::cout << "[WARNING] [SERIAL LINK IMPEDANCE] "
              << "Joint limit avoidance cannot be guaranteed with this controller." << std::endl; 
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint acceleration needed to track a given trajectory            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkImpedance::track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                                               const Eigen::Vector<double,6>   &desiredVelocity,
                                               const Eigen::Vector<double,6>   &desiredAcceleration)
{
    // NOTE: This method saves the magnitude of position and orientation error internally,
    //       so it can be queried after for analysing performance
    Eigen::Vector<double,6> poseError = pose_error(desiredPose);
    
    return map_wrench_to_torque(_cartesianVelocityGain * (desiredVelocity - endpoint_velocity())
                              + _cartesianPoseGain * poseError);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Velocity feedback                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkImpedance::resolve_endpoint_twist(const Eigen::Vector<double,6> &twist)
{
    return map_wrench_to_torque(_cartesianVelocityGain * (twist - endpoint_velocity()));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Convert endpoint twist (force & moment) to joint torques                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkImpedance::map_wrench_to_torque(const Eigen::Vector<double,6> &wrench)
{
    using namespace Eigen;
    
    unsigned int n = _model->number_of_joints();                                                    // Makes things easier

    VectorXd jointTorques = VectorXd::Zero(n);
    
    if (n <= 6)
    {
        jointTorques = _jacobianMatrix.transpose() * wrench;                                        // Too easy ;)
    }
    else
    {
        Eigen::VectorXd nullSpaceTask(n);
        
        for (int i = 0; i < n; ++i)
        {
            nullSpaceTask[i] = _jointPositionGains[i]/5.0 * (_desiredConfiguration[i] - _model->joint_positions()[i])
                             - _jointVelocityGains[i]/5.0 *  _model->joint_velocities()[i];
        }

        const auto &[Q, R] = RobotLibrary::Math::schwarz_rutishauser(_jacobianMatrix.transpose(), 1e-06);
        
        jointTorques = _jacobianMatrix.transpose() * wrench
                     + (MatrixXd::Identity(n,n) - Q * Q.transpose()) * nullSpaceTask;                             
    }
    
    // Ensure the joint torque limits are obeyed
    for (int i = 0; i < n; ++i)
    {
        const auto &limits = compute_control_limits(i);
        
        jointTorques[i] = std::clamp(jointTorques[i], limits.lower + 1e-03, limits.upper - 1e-03);
    }
    
    return jointTorques;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkImpedance::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    // NOTE: NOT USED IN THIS CLASS
    
    throw std::runtime_error("[ERROR] [SERIAL LINK IMPEDANCE] resolve_endpoint_motion(): "
                             "This method is not used in this class. Did you mean to call `map_wrench_to_torque()'?");
                 
    return Eigen::VectorXd::Zero(_model->number_of_joints());                                       // Just to be safe
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint torques needed to track a given joint trajectory           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkImpedance::track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
                                            const Eigen::VectorXd &desiredVelocity,
						                    const Eigen::VectorXd &desiredAcceleration)
{
    unsigned int n = _model->number_of_joints();
    
    Eigen::VectorXd jointTorques(n);
    
    for (int i = 0; i < n; ++i)
    {
        RobotLibrary::Model::Limits positionLimits = _model->link(i)->joint().position_limits();
        double speedLimit = _model->link(i)->joint().speed_limit();
        
        double positionReference = std::clamp(desiredPosition[i], positionLimits.lower + 1e-03, positionLimits.upper - 1e-03);
        
        double velocityReference = std::clamp(desiredVelocity[i], -speedLimit + 1e-03, speedLimit - 1e-03);
        
        jointTorques[i] = _jointVelocityGains[i] * (velocityReference - _model->joint_velocities()[i])
                        + _jointPositionGains[i] * (positionReference - _model->joint_positions()[i]);
                        
        RobotLibrary::Model::Limits torqueLimits = compute_control_limits(i);
        
        jointTorques[i] = std::clamp(jointTorques[i], torqueLimits.lower + 1e-03, torqueLimits.upper - 1e-03);
    }
    
    return jointTorques;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Set the desired joint configuration for redundancy resolution            //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkImpedance::set_desired_configuration(const Eigen::VectorXd &configuration)
{
    if (configuration.size() != _model->number_of_joints())
    {
        std::cout << "[ERROR] [SERIAL LINK IMPEDANCE] set_desired_configuration(): "
                  << "Input argument had " << configuration.size() << " elements "
                  << "but expected " << _model->number_of_joints() << "." << std::endl;
                  
        return false;
    }
    else
    {
        for (int i = 0; i < _model->number_of_joints(); ++i)
        {
            RobotLibrary::Model::Limits limits = _model->link(i)->joint().position_limits();
            
            _desiredConfiguration(i) = std::clamp(configuration(i), limits.lower + 1e-03, limits.upper - 1e-03);
        }
        
        return true;
    }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Compute the control limits                                   //
///////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Limits
SerialLinkImpedance::compute_control_limits(const unsigned int &jointNumber)
{
	RobotLibrary::Model::Limits limits;
	
	limits.lower = -_model->link(jointNumber)->joint().effort_limit();
	                           
	limits.upper = _model->link(jointNumber)->joint().effort_limit();
	
	if(limits.lower > limits.upper)
	{
	    throw std::logic_error("[ERROR] [SERIAL DYNAMIC CONTROL] compute_control_limits(): "
	                           "Lower limit for the '" + _model->link(jointNumber)->joint().name() + "' joint is greater than "
	                           "upper limit (" + std::to_string(limits.lower) + " > " + std::to_string(limits.upper) + "). "
	                           "How did that happen???");
    }
	                
	return limits;
}


} } // namespace
