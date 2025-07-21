/**
 * @file    SerialKinematicControl.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   A base class providing a standardised interface for all serial link robot arm controllers.
 * 
 * @details This class is designed to provide a standardised structure for all types of serial link
 *          robot arm controllers. This enables seemless interfacing between position/velocity and
 *          dynamic control.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#include <Control/SerialLinkBase.h>

namespace RobotLibrary { namespace Control {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
SerialLinkBase::SerialLinkBase(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                               const std::string &endpointName,
                               const RobotLibrary::Control::SerialLinkParameters &parameters)
                              : QPSolver<double>(parameters.qpsolver)
{
     _model = model;                                                                                // Save model internally so we can access it later

     _endpointFrame = _model->find_frame(endpointName);                                             // Record pointer to endpoint frame. Can throw an error!

     update();                                                                                      // Compute initial state

     unsigned int n = _model->number_of_joints();
                 
     // Transfer control options
     _cartesianPoseGain     = parameters.cartesianPoseGain;
     _cartesianVelocityGain = parameters.cartesianVelocityGain;
     _controlFrequency      = parameters.controlFrequency;
     _jointPositionGain     = parameters.jointPositionGain;
     _jointVelocityGain     = parameters.jointVelocityGain;
     _minManipulability     = parameters.minManipulability;
     _maxJointAcceleration  = parameters.maxJointAcceleration;  
     
     // Set up the QP solver
     
     // Resize dimensions of inequality constraints for the QP solver: B*qdot < z, where:
     //
     // B = [      I    ]   z = [    qdot_max  ]
     //     [     -I    ]       [   -qdot_min  ]
     //     [  (dm/dq)' ]       [  (m - m_min) ]
     
     
     _constraintMatrix.resize(2*n+1,n);
     _constraintMatrix.block(0,0,n,n).setIdentity();
     _constraintMatrix.block(n,0,n,n) = -_constraintMatrix.block(0,0,n,n);

     _constraintVector.resize(2*n+1);
     
     std::cout << "[INFO] [SERIAL LINK CONTROL] Controlling the '" << endpointName << "' frame on the '" 
               << _model->name() << "' robot." << std::endl;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Update properties specific to this controller                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
SerialLinkBase::update()
{
	_endpointPose = _endpointFrame->link->pose() * _endpointFrame->relativePose;                    // Compute new endpoint pose
	                      
    _jacobianMatrix = _model->jacobian(_endpointFrame);                                             // Jacobian for the endpoint
	                      
	_forceEllipsoid = _jacobianMatrix * _jacobianMatrix.transpose();                                // Used for certain calculations
	
	double temp = sqrt(_forceEllipsoid.determinant());                                              // Proximity to a singularity
	
	_manipulability = ( temp < 0 or std::isnan(temp)) ? 0.0 : temp;                                 // Rounding error can mean manipulability is negative or nan
}     

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Set a secondary task to be executed by a redundant robot arm                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkBase::set_redundant_task(const Eigen::VectorXd &task)
{
     if (task.size() != _model->number_of_joints())
     {
          std::cerr << "[ERROR] [SERIAL LINK] set_redundant_task(): "
                    << "This robot model has " << _model->number_of_joints() << " joints, "
                    << "but you assigned a task with " << task.size() << " elements.\n";
          
          return false;
     }
     else
     {
          _redundantTask    = task;
          _redundantTaskSet = true;
          
          return true;
     }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Compute the pose error, but save performance values internally             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector<double,6>
SerialLinkBase::pose_error(const RobotLibrary::Model::Pose &desired)
{
    Eigen::Vector<double,6> error = _endpointPose.error(desired);

    _positionError = error.head(3).norm();

    Eigen::Quaterniond orientationError = desired.quaternion() * _endpointPose.quaternion().inverse();

    double angle = 2.0 * std::acos(std::clamp(orientationError.w(), -1.0, 1.0));

    _orientationError = angle;
    
    if (angle < M_PI) error.tail(3) =  orientationError.vec();
    else              error.tail(3) = -orientationError.vec();

    return error;
}
    
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the gradient of manipulability                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkBase::manipulability_gradient()
{
    // NOTE: d/dq (JJ^T) = dJ/dq * J^T + J * dJ/dq^T,
    // but when taking the trace we can ignore off-diagonal elements.
    
    using namespace Eigen;                                                                          // For clarity

    VectorXd gradient = VectorXd::Zero(_model->number_of_joints());                                 // Initialise gradient as all zero

    LDLT<Matrix<double,6,6>> JJT(_forceEllipsoid);                                                  // Decompose for faster inverse

    Matrix<double,Dynamic,6> JT = _jacobianMatrix.transpose();                                      // Cache transpose for speed

    RobotLibrary::Model::Link *currentLink = _endpointFrame->link;                                  // Start with the end-effector link

    // Move down the kinematic chain and compute gradients w.r.t each joint
    while (currentLink != nullptr)
    {
        if (currentLink->parent_link() == nullptr) break;                                           // Gradient for 1st link in chain is always zero

        int jointIndex = currentLink->number();

        Matrix<double,6,Dynamic> dJ = _model->partial_derivative(_jacobianMatrix, jointIndex);      // Compute the partial derivative of the Jacobian w.r.t. the current joint
        
        gradient(jointIndex) = _manipulability * (JJT.solve(dJ * JT)).trace();                      // Compute the manipulability gradient contribution for this joint

        currentLink = currentLink->parent_link();                                                   // Move to the parent link in the kinematic chain
    }

    return gradient;
}

} } // namespace
