#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>
       
// Decompose a matrix A = Q*R                       
bool qr_decomposition(const Eigen::MatrixXf &A,
                            Eigen::MatrixXf &Q,
                            Eigen::MatrixXf &R);
#endif                                    
