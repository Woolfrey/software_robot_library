/**
 * @file   SerialLinkBase.cpp
 * @author Jon Woolfrey
 * @data   July 2023
 * @brief  Source files.
 */
 
#include "SerialLinkBase.h"

namespace RobotLibrary {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
SerialLinkBase::SerialLinkBase(KinematicTree *model,
                               const std::string &endpointName,
                               const double &controlFrequency)
					           : _model(model),
					             _controlFrequency(controlFrequency)
{     
     // Record pointer to the endpoint frame to be controlled so we don't need to search for it later.   
     // NOTE: This will throw a runtime error if it doesn't exist.
     
     _endpointFrame = _model->find_frame(endpointName);
     
     update();                                                                                      // Compute initial state
     
     QPSolver::set_barrier_reduction_rate(0.9);
     QPSolver::set_barrier_scalar(1000.0);

     // Resize dimensions of inequality constraints for the QP solver: B*qdot < z, where:
     //
     // B = [      I    ]   z = [    qdot_max  ]
     //     [     -I    ]       [   -qdot_min  ]
     //     [  (dm/dq)' ]       [  (m - m_min) ]
     
     unsigned int n = _model->number_of_joints();
     
     _constraintMatrix.resize(2*n+1,n);
     _constraintMatrix.block(0,0,n,n).setIdentity();
     _constraintMatrix.block(n,0,n,n) = -_constraintMatrix.block(0,0,n,n);
//   _constraintMatrix.row(2*n) = this->manipulability_gradient();                            <-- Needs to be set in the control loop

     _constraintVector.resize(2*n+1);
//   _constraintVector.block(0,0,n,1) =  upperBound;                                          <-- Needs to be set in the control loop
//   _constraintVector.block(n,0,n,1) = -lowerBound;                                          <-- Needs to be set in the control loop
//   _constraintVector(2*n) = _manipulability - _minManipulability;               <-- Needs to be set in the control loop

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
	                      
	_forceEllipsoid = _jacobianMatrix*_jacobianMatrix.transpose();                                  // Used for certain calculations
	
	double temp = sqrt(_forceEllipsoid.determinant());                                              // Proximity to a singularity
	
	_manipulability = ( temp < 0 or std::isnan(temp)) ? 0.0 : temp;                                 // Rounding error can mean manipulability is negative or nan
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Set the gains for Cartesian feedback control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkBase::set_cartesian_gains(Eigen::Matrix<double,6,6> &stiffness,
                                    Eigen::Matrix<double,6,6> &damping)
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
          _cartesianStiffness = stiffness;
          _cartesianDamping   = damping;
     
          return true;
     }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set the gains for the joint feedback control                            //
///////////////////////////////////////////////////////////////////////////////////////////////////      
bool
SerialLinkBase::set_joint_gains(const double &proportional,
                                const double &derivative)
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
		_jointPositionGain = proportional;
		_jointVelocityGain = derivative;
		return true;
	}
}
                    
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Set the maximum permissable joint acceleration                            //     
///////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkBase::set_max_joint_acceleration(const double &acceleration)
{
	if(acceleration <= 0)
	{
		std::cerr << "[ERROR] [SERIAL LINK CONTROL] set_max_joint_acceleration(): "
		          << "Input was " << acceleration << " but it must be positive.\n";
		
		return false;
	}
	else
	{
		_maxJointAcceleration = acceleration;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Set a secondary task to be executed by a redundant robot arm                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkBase::set_redundant_task(const Eigen::VectorXd &task)
{
     if(task.size() != _model->number_of_joints())
     {
          std::cerr << "[ERROR] [SERIAL LINK] set_redundant_task(): "
                    << "This robot model has " << _model->number_of_joints() << " joints, "
                    << "but you assigned a task with " << task.size() << " elements.\n";
          
          return false;
     }
     else
     {
          _redundantTask = task;
          _redundantTaskSet = true;
          
          return true;
     }
}
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the gradient of manipulability                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialLinkBase::manipulability_gradient()
{
    using namespace Eigen;                                                                          // For clarity

    VectorXd gradient = VectorXd::Zero(_model->number_of_joints());                                 // Initialise gradient as all zero

    LDLT<Matrix<double,6,6>> JJT(_forceEllipsoid);                                                  // Decompose for faster inverse

    Matrix<double,Dynamic,6> JT = _jacobianMatrix.transpose();                                      // Cache transpose for speed

    Link *currentLink = _endpointFrame->link;                                                       // Start with the end-effector link

    // Move down the kinematic chain and compute gradients w.r.t each joint
    while (currentLink != nullptr)
    {
        if (currentLink->parent_link() == nullptr) break;                                           // Gradient for 1st link in chain is always zero

        int jointIndex = currentLink->number();

        Matrix<double,6,Dynamic> dJ = this->_model->partial_derivative(this->_jacobianMatrix, jointIndex); // Compute the partial derivative of the Jacobian w.r.t. the current joint
        
        gradient(jointIndex) = this->_manipulability * (JJT.solve(dJ * JT)).trace();                // Compute the manipulability gradient contribution for this joint

        currentLink = currentLink->parent_link();                                                   // Move to the parent link in the kinematic chain
    }

    return gradient;
}

}
