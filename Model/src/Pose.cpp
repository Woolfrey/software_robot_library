#include <Pose.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose::Pose(const Eigen::Vector3f &position,
           const Eigen::Quaternionf &quaternion):
           pos(position),
           quat(quaternion)
{
	this->quat.normalize();                                                                     // Ensure unit norm for good measure
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,6,1> Pose::get_pose_error(const Pose &desired)
{
	Eigen::Matrix<float,6,1> error;                                                             // Value to be returned
	
//	if(acos(desired.quat.dot(this->quat)) > M_PI) this->quat *= -1;                             // Ensure angle < 180 degrees

	error.head(3) = desired.pos - this->pos;
	
	error.tail(3) = this->quat.w()*desired.quat.vec()
                      - desired.quat.w()*this->quat.vec()
                      - desired.quat.vec().cross(this->quat.vec());
	
	return error;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Get the pose as a 4x4 homogeneous transformation matrix                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,4,4> Pose::get_matrix()
{
	Eigen::Matrix<float,4,4> T; T.setIdentity();                                                    // Value to be returned
	
	T.block(0,0,3,3) = this->quat.toRotationMatrix();                                               // Convert to SO(3) and insert
	T.block(0,3,3,1) = this->pos;                                                                   // Insert translation component
	
	return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose Pose::operator* (const Pose &other)
{
	return Pose(this->pos + this->quat.toRotationMatrix()*other.pos,
	            this->quat*other.quat);
}
