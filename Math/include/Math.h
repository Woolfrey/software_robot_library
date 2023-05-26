#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>
       
// Decompose a matrix A = Q*R                       
bool qr_decomposition(const Eigen::MatrixXf &A,
                            Eigen::MatrixXf &Q,
                            Eigen::MatrixXf &R);
                            
// Find the null space of an upper-triangular matrix U
Eigen::MatrixXf triangular_null_space(const Eigen::MatrixXf &U);

#endif                                    
