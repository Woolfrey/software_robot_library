#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Dense>

bool get_lu_decomposition(const Eigen::MatrixXf &A,                                                 // Split a matrix A in to L and U
                                Eigen::MatrixXf &L,
                                Eigen::MatrixXf &U);
                                
bool get_qr_decomposition(const Eigen::MatrixXf &A,                                                 // Split a matrix in to Q and R
                                Eigen::MatrixXf &Q,
                                Eigen::MatrixXf &R);
                                                                                              
Eigen::MatrixXf get_cholesky_decomposition(const Eigen::MatrixXf &A);                               // A = L*L' for a positive-definite matrix

Eigen::MatrixXf get_cholesky_inverse(const Eigen::MatrixXf &A);                                     // Get the inverse of a positive-definite matrix

Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);                                              // Get inverse via LU decomposition

Eigen::MatrixXf get_triangular_inverse(const Eigen::MatrixXf &T);                                   // Get the inverse of a triangular matrix

Eigen::VectorXf backward_substitution(const Eigen::VectorXf &y,                                     // Solve a system for upper-triangular U
                                      const Eigen::MatrixXf &U,
                                      const Eigen::VectorXf &x0);
                                  
Eigen::VectorXf forward_substitution(const Eigen::VectorXf &y,                                      // Solve a sytem for lower-triangular L
                                     const Eigen::MatrixXf &L,
                                     const Eigen::VectorXf &x0);

Eigen::VectorXf solve_cholesky_system(const Eigen::VectorXf &y,                                     // Solve y = A*x where A is positive-definite
                                      const Eigen::MatrixXf &A);

Eigen::VectorXf solve_linear_system(const Eigen::VectorXf &y,                                       // Solve y = A*x for arbitrary A
                                    const Eigen::MatrixXf &A,
                                    const Eigen::VectorXf &x0);

#endif                                    
