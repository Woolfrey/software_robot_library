#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Dense>

bool get_qr_decomposition(const Eigen::MatrixXf &A,                                                 // Get the QR decomposition of a matrix
                                Eigen::MatrixXf &Q,
                                Eigen::MatrixXf &R);
                            
Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);                                              // Get the inverse of a matrix

Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A,                                               // Get the weighted inverse of a matrix
                            const Eigen::MatrixXf &W);


Eigen::VectorXf solve_linear_system(const Eigen::VectorXf &y,                                       // Solve y = A*x
                                    const Eigen::MatrixXf &A,
                                    const Eigen::VectorXf &x0);
                                    
Eigen::VectorXf solve_triangular_system(const Eigen::VectorXf &y,                                   // Solve y = U*x where U is upper triangular
                                        const Eigen::MatrixXf &U,
                                        const Eigen::VectorXf &x0);

#endif                                    
