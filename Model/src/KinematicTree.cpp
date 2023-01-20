#include <KinematicTree.h>

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
		
		return J;
	}
	else
	{
		unsigned int currentBranch = jointNumber;
		
		// WORK IN PROGRESS //
		
		return J;
	}
}
