#include <KinematicTree.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Update the kinematic & dynamic properties                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool KinematicTree::update_state(const Eigen::VectorXf &jointPosition,
                                 const Eigen::VectorXf &jointVelocity,
                                 const Pose &basePose,
                                 const Eigen::Matrix<float,6,1> baseTwist)
{
	if(jointPosition.size() != this->numJoints or jointVelocity.size() != this->numJoints)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] update_state(): "
		          << "This model has " << this->numJoints << " joints, "
		          << "but this joint position argument had " << jointPosition.size() << " elements, "
		          << "and the joint velocity argument had " << jointVelocity.size() << " elements." << std::endl;
		
		return false;
	}
	else
	{
		// Variables used in this scope
		float m;
		Eigen::MatrixXf J, Jdot, Jv, Jw, Jvdot, Jwdot;
		Eigen::Matrix3f I, Idot;
		Eigen::Vector3f com;
		
		
		// Reset values
		this->Mjj.setZero();
		this->Mjb.setZero();
		this->Cjj.setZero();
		this->Cjb.setZero();
		
		for(int i = 0; i < this->numJoints; i++)
		{
			m    = this->branch[i].mass();
			I    = this->branch[i].inertia();
			com  = this->branch[i].pose()*this->branch[i].com();                       // Transform centre of mass to global frame
			Idot = this->branch[i].inertia_derivative();
			
			// Compute joint space dynamics
			J = jacobian(com,i+1);                                                    // Get the Jacobian to the ith centre of mass
			Jv = J.block(0,0,3,i+1);
			Jw = J.block(3,0,3,i+1);
			
			this->Mjj += m*Jv.transpose()*Jv
			           + Jw.transpose()*I*Jw;
			     

			Jdot = time_derivative(J);
			
			this->Cjj += m*Jv.transpose()*Jdot.block(0,0,3,i+1)
			           + Jw.transpose()*(I*Jdot.block(3,0,3,i+1) + Idot*Jw);
			           
			// Compute joint/base coupled dynamics
			
		}
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Compute the Jacobian to the given branch                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf KinematicTree::jacobian(const Eigen::Vector3f &point,
                                        const std::string     &branchName)
{
	// Search the tree from the outer-most branches for speed
	for(int i = this->numJoints-1; i >= 0; i--)
	{
		if(this->branch[i].name() == branchName) return jacobian(point, i);                 // Return Jacobian to the ith branch
	}
	
	std::cerr << "[ERROR] [KINEMATIC TREE] jacobian(): "
	          << "Could not find a branch by the name of " << branchName << "!" << std::endl;
	
	return Eigen::MatrixXf::Zero(6,this->numJoints);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Compute the Jacobian to the given branch index                          //   
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf KinematicTree::jacobian(const Eigen::Vector3f &point,
                                        const unsigned int    &jointNumber)
{
	Eigen::MatrixXf J(6,this->numJoints); J.setZero();
	
	if(jointNumber > this->numJoints)
	{
		std::cerr << "[ERROR] [KINEMATIC TREE] jacobian(): "
		          << "Cannot compute the Jacobian up to joint " << jointNumber << " "
		          << "as this model has only " << this->numJoints << " joints!" << std::endl;
	}
	else
	{
/*		unsigned int i = jointNumber;
		
		{
			J.block(0,i,3,1) = this->branch[i].jointAxis().cross(point - this->branch[i].pose().pos());
			J.block(3,i,3,1) = this->branch[i].jointAxis();
			
			i = this->branch[i].root();                                                 // Next along this branch
		}*/
	}
	
	return J;
}
