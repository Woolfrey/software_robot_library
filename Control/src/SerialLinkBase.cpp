/**
 * @file   SerialLinkBase.cpp
 * @author Jon Woolfrey
 * @data   July 2023
 * @brief  Source files.
 */
 
#include <SerialLinkBase.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Constructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
SerialLinkBase::SerialLinkBase(KinematicTree *model, const std::string &endpointName)
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
//   this->_constraintMatrix.row(2*n) = this->manipulability_gradient();              <-- Needs to be set in the control loop

     this->_constraintVector.resize(2*n+1);
//   this->_constraintVector.block(0,0,n,1) =  upperBound;                            <-- Needs to be set in the control loop
//   this->_constraintVector.block(n,0,n,1) = -lowerBound;                            <-- Needs to be set in the control loop
//   this->_constraintVector(2*n) = this->_manipulability - this->_minManipulability; <-- Needs to be set in the control loop
     
     std::cout << "[INFO] [SERIAL LINK CONTROL] Controlling the '" << endpointName << "' frame on the '" 
               << this->_model->name() << "' robot.\n";
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Update properties specific to this controller                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
SerialLinkBase::update()
{
	this->_endpointPose = this->_endpointFrame->link->pose() * this->_endpointFrame->relativePose;  // Compute new endpoint pose
	                      
    this->_jacobianMatrix = this->_model->jacobian(this->_endpointFrame);                           // Jacobian for the endpoint
	                      
	this->_forceEllipsoid = this->_jacobianMatrix*this->_jacobianMatrix.transpose();                // Used for certain calculations
	
	double temp = sqrt(this->_forceEllipsoid.determinant());                                        // Proximity to a singularity
	
	this->_manipulability = ( temp < 0 or std::isnan(temp)) ? 0.0 : temp;                           // Rounding error can mean manipulability is negative or nan
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
          this->_cartesianStiffness = stiffness;
          this->_cartesianDamping   = damping;
     
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
		this->_jointPositionGain = proportional;
		this->_jointVelocityGain = derivative;
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
		this->_maxJointAcceleration = acceleration;
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Set a secondary task to be executed by a redundant robot arm                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool
SerialLinkBase::set_redundant_task(const Eigen::VectorXd &task)
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
Eigen::VectorXd
SerialLinkBase::manipulability_gradient()
{
    // NOTE TO FUTURE SELF: Gradient with respect to first joint in a chain is always zero.
    //                      It is possible to reduce calcs if this condition can be efficiently
    //                      integrated in the loop below.

    Eigen::VectorXd gradient(this->_model->number_of_joints());                                     // Value to be returned
 
    gradient.setZero();                                                                             // Any branching chains off of this one must be zero

    Eigen::LDLT<Eigen::Matrix<double,6,6>> JJT(this->_forceEllipsoid);                              // Pre-compute the decomposition

    Link *currentLink = this->_endpointFrame->link;                                                 // Starting link for computation

    while(currentLink != nullptr)
    {
	    Eigen::Matrix<double,6,Eigen::Dynamic> dJ
	    = this->_model->partial_derivative(this->_jacobianMatrix,currentLink->number());
	    
	    gradient(currentLink->number())
	    = this->_manipulability*(JJT.solve(dJ*this->_jacobianMatrix.transpose())).trace();
	    
	    currentLink = currentLink->parent_link();                                                   // Get pointer to next link in chain
    }

    gradient(0) = 0.0;

    return gradient;
}
