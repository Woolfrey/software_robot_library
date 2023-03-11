#include <Pose.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose::Pose(const Eigen::Vector3f    &position,
           const Eigen::Quaternionf &quaternion)
           :
           _pos(position),
           _quat(quaternion)
{
	this->_quat.normalize();                                                                    // Ensure unit norm for good measure
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,6,1> Pose::error(const Pose &desired)
{
	Eigen::Matrix<float,6,1> error;                                                             // Value to be returned

	float angle = acos(desired.quat().dot(this->_quat));

	if(angle > M_PI)
	{
		std::cout << "[WARNING] [POSE] error(): "
		          << "Angle between quaternions is " << angle*180/M_PI << " degrees!" << std::endl;
	}

	error.head(3) = desired.pos() - this->_pos;
	
	error.tail(3) = this->_quat.w()*desired.quat().vec()
                      - desired.quat().w()*this->_quat.vec()
                      - desired.quat().vec().cross(this->_quat.vec());
	
	return error;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Get the pose as a 4x4 homogeneous transformation matrix                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,4,4> Pose::as_matrix()
{
	Eigen::Matrix<float,4,4> T; T.setIdentity();                                                // Value to be returned
	
	T.block(0,0,3,3) = this->_quat.toRotationMatrix();                                          // Convert to SO(3) and insert
	T.block(0,3,3,1) = this->_pos;                                                              // Insert translation component
	
	return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose Pose::inverse()
{
	// NOTE TO FUTURE SELF: I don't think this is correct!
	// Need to rotate the position vector.
	
	return Pose(-this->_pos, this->_quat.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose Pose::operator* (const Pose &other)
{
	return Pose(this->_pos + this->_quat.toRotationMatrix()*other.pos(),
	            this->_quat*other.quat());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f Pose::operator* (const Eigen::Vector3f &other)
{
	return this->_pos + this->_quat.toRotationMatrix()*other;
}
