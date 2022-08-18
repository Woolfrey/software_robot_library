#include <SerialDynamicControl.h>
/*
Eigen::VectorXf SerialDynamicControl::accelerate_endpoint(const Eigen::VectorXf &accel,
                                                          const Eigen::VectorXf &redundant)
{
	// Variables used in this scope
	Eigen::MatrixXf    J = get_jacobian();
	Eigen::MatrixXf    M = get_inertia();
	Eigen::VectorXf    h = feedback_linearization();
	Eigen::VectorXf qdot = get_joint_velocities();
	Eigen::VectorXf xMin(2*this->n);
	Eigen::VectorXf xMax(2*this->n);
	
	//       6   n  n  n
	// A = [ 0   0  J  0 ] 6
	//     [ 0   0 -M  I ] n
	//     [ J' -M  0  0 ] n
	//     [ 0  -M  0  I ] n
	
	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6+3*this->n,6+3*this->n);
	A.block(0          ,6+  this->n,      6,this->n) = J;
	A.block(6          ,6+  this->n,this->n,this->n) = -M;
	A.block(6          ,6+2*this->n,this->n,this->n).setIdentity();
	A.block(6+  this->n,          0,this->n,      6) = J.transpose();
	A.block(6+  this->n,          6,this->n,this->n) = -M;
	A.block(6+2*this->n,          6,this->n,this->n) = -M;
	A.block(6+2*this->n,6+2*this->n,this->n,this->n).setIdentity();
	
	// y = = [ accel - Jdot*qdot ] 6
	//       [    C*qdot + g     ] n
	//       [         0         ] n
	//       [      redundant    ] n
	
	Eigen::VectorXf y(6+3*this->n);
	
	y.block(0          , 0,      6,1) = accel - get_time_derivative(J)*qdot;
	y.block(6          , 0,this->n,1) = h;
	y.block(6+  this->n, 0,this->n,1).setZero();
// 	y.block(6+2*this->n, 0,this->n,1) = redundant; // Assigned in the for loop below
	
	for(int i = 0; i < this->n; i++)
	{
		float lower, upper;
		get_accel_limit(lower, upper,i);                                                    // Get instantaneous limits on joint acceleration
		
		xMin(i)         = lower;                                                            // Assign lower acceleration limit
		xMax(i)         = upper;                                                            // Assign upper acceleration limit
//		xMin(i+this->n) = this->tLim[i][0];                                                 // Assign lower torque limit
//		xMax(i+this->n) = this->tLim[i][1];                                                 // Assign upper torque limit
		
		y(6+2*this->n+i) = redundant(i) - get_penalty_torque(i);                            // Add joint limit avoidance to null space
	}
	
        Eigen::VectorXf temp = least_squares(y,A,Eigen::MatrixXf::Identity(6+3*this->n,6+3*this->n),xMin,xMax,h);
//	Eigen::VectorXf temp = least_squares(y,A,Eigen::MatrixXf::Identity(6+3*this->n,6+3*this->n),xMin,xMax,this->last_solution);
//	this->last_solution = temp.tail(this->n);                                                   // Save for next time this is called
	return temp.tail(this->n);                                                                  // Last n values are the joint torques
*/
