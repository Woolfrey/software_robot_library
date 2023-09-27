#ifndef SERIALLINKKINEMATICS_H_
#define SERIALLINKKINEMATICS_H_

#include <SerialLinkBase.h>

using namespace Eigen;
using namespace std;

template <class DataType>
class SerialLinkKinematics : public SerialLinkBase<DataType>
{
	public:
		SerialLinkKinematics(const string &pathToURDF,
		                     const string &endpointName)
		: SerialLinkBase<DataType>(pathToURDF, endpointName) {}
		
		// Functions derived from the base class
		
		Vector<DataType,Dynamic> resolve_endpoint_motion(const Vector<DataType,6> &endPointMotion); // Solve the joint motion to execute a given endpoint motion
		
		Vector<DataType,Dynamic> track_endpoint_trajectory(const Pose<DataType>     &desiredPose,
							           const Vector<DataType,6> &desiredVel,
							           const Vector<DataType,6> &desiredAcc);
							  
		Vector<DataType,Dynamic> track_joint_trajectory(const Vector<DataType,Dynamic> &desiredPos,
		                                                const Vector<DataType,Dynamic> &desiredVel,
		                                                const Vector<DataType,Dynamic> &desiredAcc);
													   		
	protected:
	
		bool compute_control_limits(DataType &lower, DataType &upper, const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,Dynamic> SerialLinkKinematics<DataType>::resolve_endpoint_motion(const Vector<DataType,6>&endpointMotion)
{
	Matrix<DataType,6,Dynamic> J = this->endpoint_jacobian();                                   // Need 'this->' or it won't compile
	
	Matrix<DataType,6,6> JJt = J*J.transpose();                                                 // Makes calcs a little easier
	
	DataType manipulability = sqrt((JJt).determinant());                                        // Proximity to a singularity
	
	if(manipulability <= this->threshold)
	{
		cout << "[WARNING] [SERIAL LINK CONTROL] resolve_endpoint_motion(): "
		     << "Robot is in a singular configuration! "
		     << "(Manipulability " << manipulability << " <  threshold " << this->threshold << ").\n";
		
		return 0.9*this->jointVelocity;                                                     // Slow down
	}

	Vector<DataType,Dynamic> gradient(this->numJoints); gradient(0) = 0;                        // Gradient of manipulability
	
	Vector<DataType,Dynamic> startPoint = this->jointVelocity;                                  // Needed for the QP solver

	Vector<DataType,Dynamic> upperBound(this->numJoints),
                                 lowerBound(this->numJoints);                                       // Instantaneous limits on solution

	// Compute joint control limits and gradient of manipulability
	for(int i = 0; i < this->numJoints; i++)
	{
		// Maximum speed permissable
		if(not compute_control_limits(lowerBound(i), upperBound(i), i))
		{
			throw runtime_error("[FLAGRANT SYSTEM ERROR] resolve_endpoint_motion(): "
				            "Unable to compute the joint limits.");
		}
		
		// Ensure start point is inside bounds or QP solver will fail
		     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 0.001;
		else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 0.001;
		
		// Get the gradient of manipulability
		if(i > 0)
		{
			Matrix<DataType,6,Dynamic> dJ = partial_derivative(J,i);                    // Partial derivative w.r.t. ith joint
			
			gradient(i) = manipulability*(JJt.ldlt().solve(dJ*J.transpose()));          // Gradient of manipulability
		}
	}
	
	// Set up the inequality constraints for the QP solver B*qdot > z
	
	// B = [     I     ]   >   z = [  upperBound ]
	//     [    -I     ]           [ -lowerBound ]
	//     [ (dmu/dq)' ]           [ -gamma*mu   ]
	
	unsigned int n = this->numJoints;                                                           // I'm too lazy to type 'this->numJoints' every time
	
	Matrix<DataType,Dynamic,Dynamic> B(2*n+1,n);
	B.block(0,0,n,n).setIdentity();
	B.block(n,0,n,n) = -B.block(0,0,n,n);
	B.row(2*n+1) = gradient.transpose();
	
	Vector<DataType,Dynamic> z(2*n+1);
	
	z.block(0,0,n,1) =  upperBound;
	z.block(n,0,n,1) = -lowerBound;
	z(2*n+1) = -this->barrierScalar*manipulability;
	
	// Solve optimisation problem based on number of joints
	
	if(this->numJoints <= 6)                                                                    // No redundancy
	{
		// min || xdot - J*qdot ||^2
		// subject to:   qdot_min < qdot < qdot_max
		//               (dmu/dq)'*qdot > -gamma*mu 
		
		return QPSolver<DataType>::solve(J.transpose()*J,-endpointMotion,B,z,startPoint);   // Too easy lol				
	}
	else
	{
		if(not this->redundantTaskSet) this->redundantTask = gradient;                      // Optimise manipulability by default
		
		this->redundantTaskSet = false;                                                     // Reset for next loop
	
		return QPSolver<DataType>::redundant_least_squares(this->redundantTask,
		                                                   this->jointInertiaMatrix,
		                                                   J, endpointMotion, B, z, startPoint);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,Dynamic> SerialLinkKinematics<DataType>::track_endpoint_trajectory(const Pose<DataType>     &desiredPose,
                                                                                   const Vector<DataType,6> &desiredVel,
                                                                                   const Vector<DataType,6> &desiredAcc)
{
	return resolve_endpoint_motion(desiredVel + this->K*this->endpoint_pose().error(desiredPose)); // Feedforward + feedback
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint velocities needs to track a given trajectory               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,Dynamic> SerialLinkKinematics<DataType>::track_joint_trajectory(const Vector<DataType,Dynamic> &desiredPos,
                                                                                const Vector<DataType,Dynamic> &desiredVel,
                                                                                const Vector<DataType,Dynamic> &desiredAcc)
{
	if(desiredPos.size() != this->numJoints	or desiredVel.size() != this->numJoints)
	{
		cerr << "[ERROR] [SERIAL LINK CONTROL] track_joint_trajectory(): "
		     << "This robot has " << this->numJoints << " joints but "
		     << "the position argument had " << desiredPos.size() << " elements and "
		     << "the velocity argument had " << desiredVel.size() << " elements.\n";
		
		return 0.9*this->jointVelocity;
	}
	else
	{
		Vector<DataType,Dynamic> vel = desiredVel + this->kp*(desiredPos - this->jointPosition);    // Feedforward + feedback
	
		// Ensure kinematic feasibility	
		for(int i = 0; i < this->numJoints; i++)
		{
			DataType lowerBound, upperBound;
			
			if(not compute_joint_limits(lowerBound,upperBound,i))
			{
				cerr << "[ERROR] [SERIAL LINK CONTROL] track_joint_trajectory(): "
				     << "Could not compute joint limits for the '"
				     << this->activeLink[i].joint().name() << "' joint.\n";
			}
			else
			{
				     if(vel(i) <= lowerBound) vel(i) = lowerBound + 0.001;
				else if(vel(i) >= upperBound) vel(i) = upperBound - 0.001;
			}
		}
		
		return vel;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool SerialLinkKinematics<DataType>::compute_control_limits(DataType &lower,
                                                            DataType &upper,
                                                            const unsigned int &jointNumber)
{
	DataType dq = this->jointPosition[jointNumber] - this->positionLimit[jointNumber][0];       // Distance to lower limit
	
	lower = max( -this->hertz*dq,                                                               
	        max( -this->velocityLimit[jointNumber],
	             -sqrt(2*this->accelLimit[jointNumber]*dq) ));
	                  
	dq = this->positionLimit[jointNumber][1] - this->jointPosition[jointNumber];
	
	upper = min( this->hertz*dq,
	        min( this->velocityLimit[jointNumber],
	                  sqrt(2*this->accelLimit[jointNumber]*dq) ));
	                  
	if(lower >= upper)
	{
		cerr << "[ERROR] [SERIAL LINK CONTROL] compute_joint_limits(): "
		     << "Lower bound is greater than upper bound for the '"
		     << this->joint[jointNumber].name() << "' joint. "
		     << "(" << lower << " >= " << upper << "). How did that happen?\n";
	
		return false;
	}
	else 	return true;        
}

#endif
