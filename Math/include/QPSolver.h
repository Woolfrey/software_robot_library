#ifndef QPSOLVER_H_
#define QPSOLVER_H_

#include <Math.h>                                                                                    // Required for QR Decomposition
#include <vector>                                                                                    // std::vector

class QPSolver
{
	public:
		QPSolver() {}
		
		// These functions can be called without creating a QPSolver object:
		static Eigen::VectorXf solve(const Eigen::MatrixXf &H,                              // Solve a generic QP problem
		                             const Eigen::VectorXf &f,
		                             const Eigen::VectorXf &x0);		
		                             
		static Eigen::VectorXf least_squares(const Eigen::VectorXf &y,                      // Solve an unconstrained least squares problem
		                                     const Eigen::MatrixXf &A,
		                                     const Eigen::MatrixXf &W,
		                                     const Eigen::VectorXf &x0);
		                                     
		static Eigen::VectorXf least_squares(const Eigen::VectorXf &xd,                     // Solve a least squares with equality constraints
		                                     const Eigen::MatrixXf &W,
		                                     const Eigen::VectorXf &y,
		                                     const Eigen::MatrixXf &A,
		                                     const Eigen::VectorXf &x0);
		                                     
		// These functions require an object to be created since they use the
		// interior point solver:
		Eigen::VectorXf solve(const Eigen::MatrixXf &H,                                     // Solve a QP problem with inequality constraints
		                      const Eigen::VectorXf &f,
		                      const Eigen::MatrixXf &B,
		                      const Eigen::VectorXf &c,
		                      const Eigen::VectorXf &x0);
		                                                   
		Eigen::VectorXf least_squares(const Eigen::VectorXf &y,                             // Solve a constrained least squares problem
		                              const Eigen::MatrixXf &A,
		                              const Eigen::MatrixXf &W,
		                              const Eigen::MatrixXf &xMin,
		                              const Eigen::MatrixXf &xMax,
		                              const Eigen::VectorXf &x0);
		                              
		Eigen::VectorXf least_squares(const Eigen::VectorXf &xd,                            // Solve a constrained least squares problem
		                              const Eigen::MatrixXf &W,
		                              const Eigen::VectorXf &y,
		                              const Eigen::VectorXf &A,
		                              const Eigen::VectorXf &xMin,
		                              const Eigen::VectorXf &xMax,
		                              const Eigen::VectorXf &x0);
		                         
	private:
		// These are variables used by the interior point method:
		float alpha0    = 1.0;                                                              // Modify size of Newton Step
		float alphaMod  = 0.1;                                                              // Modify step size when constraint violated
		float beta0     = 0.1;                                                             // Rate of decreasing barrier function
		float betaMod   = 0.2;                                                              // Increase of rate of barrier function
		float timeout   = 5e-03;                                                            // Timeout for interior point method
		float tol       = 1e-03;                                                            // Tolerance on step size
		float u0        = 1.0;                                                              // Scalar on barrier function
		float uMod      = 2.0;                                                              // Value to increase barrier function
		int   steps     = 20;                                                               // No. of steps to run interior point method
		                         
};                                                                                                  // Semicolon needed after class declaration

#endif
