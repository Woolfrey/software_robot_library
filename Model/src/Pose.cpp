#include <Pose.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose::Pose(const Eigen::Vector3f    &position,
           const Eigen::Quaternionf &quaternion)
           :
           _position(position),
           _quaternion(quaternion.normalized())
{

}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,6,1> Pose::error(const Pose &desired)
{
	Eigen::Matrix<float,6,1> error;                                                             // Value to be returned

	error.head(3) = desired.position() - this->_position;                                       // Position error
	
	float angle = this->_quaternion.angularDistance(desired.quaternion());                      // Distance between vectors (i.e. dot product)
	
	if(angle < M_PI)
	{
		error.tail(3) = this->_quaternion.w() * desired.quaternion().vec()
		              - desired.quaternion().w() * this->_quaternion.vec()
		              - desired.quaternion().vec().cross(this->_quaternion.vec());
	}
	else // Spin the other direction
	{
		error.tail(3) = desired.quaternion().w() * this->_quaternion.vec()
		              - this->_quaternion.w() * desired.quaternion().vec()
		              + desired.quaternion().vec().cross(this->_quaternion.vec());
	}
	
	return error;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Get the pose as a 4x4 homogeneous transformation matrix                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,4,4> Pose::as_matrix()
{
	Eigen::Matrix<float,4,4> T; T.setIdentity();                                                // Value to be returned
	
	T.block(0,0,3,3) = this->_quaternion.toRotationMatrix();                                    // Convert to SO(3) and insert
	T.block(0,3,3,1) = this->_position;                                                         // Insert translation component
	
	return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose Pose::inverse()
{
	return Pose(-this->_quaternion.toRotationMatrix()*this->_position, this->_quaternion.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose Pose::operator* (const Pose &other)
{
	return Pose(this->_position + this->_quaternion.toRotationMatrix()*other.position(),
	            this->_quaternion*other.quaternion());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f Pose::operator* (const Eigen::Vector3f &other)
{
	return this->_position + this->_quaternion.toRotationMatrix()*other;
}
