/**
 * @file   SkewSymmetric.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class uses for representing the skew-symmetric matrix mapping of a 3D vector.
 */
 
#ifndef SKEW_SYMMETRIC_H_
#define SKEW_SYMMETRIC_H
 
template <class DataType>
class SkewSymmetric
{
     public:
          /**
           * Constructor.
           * @param vec A 3D Eigen::Vector to be made as a skew-symmetric matrix.
           */
          SkewSymmetric(const Eigen::Vector<DataType,3> vec) : _vec(vec) {}
          
          Eigen::Matrix<DataType,3,3> as_matrix()
          {
               Eigen::Matrix<DataType,3,3> S;
               S <<             0 , -this->_vec(2),  this->_vec(1),
                     this->_vec(2),             0 , -this->_vec(0),
                    -this->_vec(1),  this->_vec(0),             0 ;
                    
               return S;
          }

          /**
           * Multiply this skew-symmetric matrix with another tensor.
           * This speeds up calcs by skipping the 0's along the diagonal.
           * @param other The other tensor to multiply with (3xn)
           * @return A 3xn matrix resulting from the product.
           */
          Eigen::Matrix<DataType,3,Eigen::Dynamic> operator*(const Eigen::Matrix<DataType,3,Eigen::Dynamic> &other)
          {
               Eigen::Matrix<DataType,3,Eigen::Dynamic> result;
               result.resize(Eigen::NoChange,other.cols());
               
               for(int j = 0; j < other.cols(); j++)
               {
                    result(0,j) = this->_vec(1)*other(2,j) - this->_vec(2)*other(1,j);
                    result(1,j) = this->_vec(2)*other(0,j) - this->_vec(0)*other(2,j);
                    result(2,j) = this->_vec(0)*other(1,j) - this->_vec(1)*other(0,j);
               }
               
               return result;
          }
          
     private:
     
          Eigen::Vector<DataType,3> _vec;
};                                                                                                  // Semicolon needed after class declaration

#endif
