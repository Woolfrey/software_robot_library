#include <iostream>                                                                                 // std::cerr
#include <QPSolver.h>                                                                               // Declaration of functions
#include <vector>                                                                                   // std::vector


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a generic QP problem min 0.5*x'*H*x - x'*f                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::solve(const Eigen::MatrixXf &H,
                                const Eigen::VectorXf &f,
                                const Eigen::VectorXf &x0)
{
	// Check the inputs are sound
	int n = x0.size();
	if(H.rows() != n or H.cols() != n or f.size() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Dimensions of input arguments do not match. "
		          << "H matrix was " << H.rows() << "x" << H.cols() << ", "
		          << "f vector was " << f.size() << "x1, and "
		          << "x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else	return solve_linear_system(f,H,x0);                                                 // Too easy, lol
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve a constrained QP problem: min 0.5*x'*H*x - x'*f subject to: B*x >= c           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::solve(const Eigen::MatrixXf &H,                                          // Solve a QP problem with inequality constraints
                                const Eigen::VectorXf &f,
                                const Eigen::MatrixXf &B,
                                const Eigen::VectorXf &c,
                                const Eigen::VectorXf &x0)
{
	// Check the inputs are sound
	int n = x0.size();
	if(H.rows() != n or H.cols() != n or f.size() != n or B.rows() != c.size())
	{
		std::cout << "[ERROR] [QPSOLVER] solve(): "
		          << "Dimensions of input arguments do not match. "
		          << "H matrix was " << H.rows() << "x" << H.cols() << ", "
		          << "f vector was " << f.size() << "x1, "
		          << "B matrix was " << B.rows() << "x" << B.cols() << ", "
		          << "c vector was " << c.size() << "x1, and "
		          << "x0 vector was " << n << "x1." << std::endl;
		          
		return x0;
	}
	else
	{
		// Solve the following optimization problem with Guass-Newton method:
		//
		//    min f(x) = 0.5*x'*H*x - x'*f - u*sum(log(d_i))
		//
		// where d_i = b_i*x - c_i is the distance to the constraint
		//
		// Then the gradient and Hessian are:
		//
		//    g(x) = H*x - f - u*sum((1/d_i)*b_i')
		//
		//    I(x) = H + u*sum((1/(d_i^2))*b_i'*b_i)
		
		// Local variables
		Eigen::VectorXf  x = x0;
		Eigen::VectorXf  x_prev = x;
		Eigen::VectorXf g(n);
		Eigen::MatrixXf I;
		Eigen::VectorXf dx;
		
		float u = 0.1;
		float alpha = 1.0;
		float beta = 0.01;
		
		int count;
		
		for(int i = 0; i < 10; i++)
		{
			// (Re)set values for new loop
			bool violation = false;
			I = H;
			g = Eigen::VectorXf::Zero(n);
			
			// Calculate distance to each constraint
			for(int j = 0; j < B.rows(); j++)
			{
				float d = B.row(j).dot(x) - c(j);
				
				if(d <= 0)
				{
					violation = true;
					d = 1E-4;
					u *= 10;
				}
				
				g -= u/d*B.row(j).transpose();
				I += u/(d*d)*B.row(j).transpose()*B.row(j);
			}
			
			if(violation)
			{
				x = x_prev;
				alpha *= 0.1;
				beta += 0.1*(1-beta);
			}
			
			g += H*x - f;
			
			dx = solve_linear_system(-g,I,Eigen::VectorXf::Zero(n));
			
			count = i;
			
			if(dx.norm() < 1E-2) break;
			
			x += alpha*dx;
			u *= beta;
		}
		
		for(int j = 0; j < B.rows(); j++)
		{
			float d = B.row(j).dot(x) - c(j);
			
			if(d <= 0)
			{
				x = x_prev;
				break;
			}
		}
		
		return x;
	}
}
					
			

/*		
		float u     = this->u0;                                                             // Scalar for barrier function
		float alpha = this->alpha0;                                                         // Scalar for Newton Step
		float beta  = this->beta0;                                                          // Shrinks barrier function scalar each loop
		
		int numConstraints = B.rows();
		
		Eigen::MatrixXf I(n,n);                                                             // Hessian matrix for barrier function
		
		Eigen::VectorXf dx;                                                                 // Newton step
		Eigen::VectorXf g(n);		                                                    // Gradient vector
		Eigen::VectorXf x      = x0;                                                        // Set initial state vector
		Eigen::VectorXf x_prev = x;                                                        // Used to save last solution
		
		// Run interior point method
		for(int i = 0; i < this->steps; i++)
		{
			//(Re)set values for new loop
			bool violation = false;
			I = H;
			g.setZero();
			
			// Check the constraints
			for(int j = 0; j < numConstraints; j++)
			{
				float d = B.row(j).dot(x) - c(j);

				if(d <= 0)
				{
					violation = true;                                           // Flag constraint violation for later
					u        *= this->uMod;                                     // Increase barrier function scalar
					d         = 1E-3;                                           // Set a very small, non-zero value
				}
				
				g -= (u/d)*B.row(j).transpose();                                    // Add up gradient vector
				I += (u/(d*d))*B.row(j).transpose()*B.row(j);                       // Add up Hessian matrix
			}
			
			if(violation)
			{
				x      = x_prev;                                                    // Go back to last solution
				alpha *= this->alphaMod;                                            // Decrease the step size
				beta  += this->betaMod*(1 - beta);                                  // Slow decrease of barrier function
			}
					
			g += H*x - f;                                                               // Add final part of gradient

//			dx = -I.inverse()*g;                                                        // Too slow!
			dx = solve_linear_system(-g,I,Eigen::VectorXf::Zero(n));                    // Using LU decomposition
//			dx = solve_cholesky_system(-g,I);                                           // Doesn't work for singular matrices!
						
			if(dx.norm() < this->tol) break;                                            // Step size is small, end algorithm
			
			x_prev   = x;                                                               // Save last value for next loop
			x       += alpha*dx;                                                        // Increment the solution
			u       *= beta;                                                            // Decrement barrier function
		}
		
		// Do one last check on the constraint
		for(int j = 0; j < numConstraints; j++)
		{
			float d = B.row(j).dot(x) - c(j);                                           // Distance to constraint
			
			if(d <= 0)
			{
				x = x_prev;
				break;
			}
		}
		
		return x;
	}
} */

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve an unconstrained least squares problem: min 0.5(y-*Ax)'*W*(y-A*x)              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A,
                                        const Eigen::MatrixXf &W,
                                        const Eigen::VectorXf &x0)
{
	// Check the inputs are sound
	int m = A.rows();
	int n = A.cols();
	if(y.size() != m or x0.size() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions of inputs arguments do not match! "
		          << "The y vector was " << y.size() << "x1, "
		          << "the A matrix was " << m << "x" << n << ", "
		          << "the W matrix was " << W.rows() << "x" << W.cols() << ", and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else if(W.rows() != m or W.cols() != m)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
		          << "but expected " << m << "x" << m << "." << std::endl;
		return x0;
	}
	else
	{
		Eigen::MatrixXf AtW = A.transpose()*W;
		return solve(AtW*A,AtW*y,x0);
	}
}
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //    Solve a constrained least squares problem 0.5*(y-A*x)'*W*(y-A*x) s.t. xMin <= x <= xMax    //
///////////////////////////////////////////////////////////////////////////////////////////////////                           
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A,
                                        const Eigen::MatrixXf &W,
                                        const Eigen::VectorXf &xMin,
                                        const Eigen::VectorXf &xMax,
                                        const Eigen::VectorXf &x0)
{
	int m = A.rows();
	int n = A.cols();
	
	// Check that the inputs are sound
	if(y.size() != m or x0.size() != n or xMin.size() != n or xMax.size() != n or x0.size() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions of inputs arguments do not match! "
		          << "The y vector was " << y.size() << "x1, "
		          << "the A matrix was " << m << "x" << n << ", "
		          << "the W matrix was " << W.rows() << "x" << W.cols() << ", "
		          << "the xMin vector was " << xMin.size() << "x1, "
		          << "the xMax vector was " << xMax.size() << "x1, and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else if(W.rows() != m or W.cols() != m)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
		          << "but expected " << m << "x" << m << "." << std::endl;
		return x0;
	}
	else
	{
		// Set up constraint matrices in standard form Bx >= c
		Eigen::MatrixXf B(2*n,n);
		B.block(0,0,n,n) = -1*Eigen::MatrixXf::Identity(n,n);
		B.block(n,0,n,n) =    Eigen::MatrixXf::Identity(n,n);
		
		Eigen::VectorXf c(2*n);
		c.block(0,0,n,1) = -1*xMax;
		c.block(n,0,n,1) =    xMin;
		
		Eigen::MatrixXf AtW = A.transpose()*W;
		
		return solve(AtW*A, AtW*y, B, c, x0);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //       Solve a least squares problem of the form min 0.5*(xd-x)'*W*(xd-x)  s.t. A*x = y        //
///////////////////////////////////////////////////////////////////////////////////////////////////                                  
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &xd,
                                        const Eigen::MatrixXf &W,
                                        const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A,
                                        const Eigen::VectorXf &x0)
{
	// Check that the inputs are sound
	int m = A.rows();
	int n = A.cols();
	
	if(xd.size() != n or y.size() != m or x0.size() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions of input arguments do not match. "
		          << "The xd vector was " << xd.size() << "x1, "
		          << "the W matrix was " << W.rows() << "x" << W.cols() << ", "
		          << "the y vector was " << y.size() << "x1, "
		          << "the A matrix was " << m << "x" << n << ", and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else if(W.rows() != n or W.cols() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
		          << "but expected " << n << "x" << n << "." << std::endl;
		
		return x0;
	}
	else
	{
		// Lagrangian L = 0.5*x'*W*x - x'*W*xd + (A*x - y)'*lambda,
		// Solution exists for:
		//
		// [ dL/dlambda ]    =  [ 0   A ][ lambda ]  = [ 0 ]
		// [ dL/dx      ]       [ A'  W ][   x    ]    [ 0 ]
		//
		// but we can speed up calcs and skip solving lambda if we are clever.
		
		Eigen::MatrixXf H = Eigen::MatrixXf::Zero(m+n,m+n);
//		H.block(0,0,m,m) = Eigen::MatrixXf::Zero(m,n);
		H.block(0,m,m,n) = A;
		H.block(m,0,n,m) = A.transpose();
		H.block(m,m,n,n) = W;
		
		Eigen::MatrixXf Q, R;
		if(get_qr_decomposition(H,Q,R))
		{
			Eigen::MatrixXf R22 = R.block(m,m,n,n);
			Eigen::MatrixXf Q12 = Q.block(0,m,m,n);
			Eigen::MatrixXf Q22 = Q.block(m,m,n,n);
			
			return backward_substitution(Q12.transpose()*y + Q22.transpose()*W*xd, R22, x0);
		}
		else    return x0;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //     Solve a problem of the form min 0.5*(xd-x)'*W*(xd-x)  s.t. A*x = y, xMin <= x <= xMax     //
///////////////////////////////////////////////////////////////////////////////////////////////////  
                              
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &xd,
                                        const Eigen::MatrixXf &W,
                                        const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A,
                                        const Eigen::VectorXf &xMin,
                                        const Eigen::VectorXf &xMax,
                                        const Eigen::VectorXf &x0)
{
	// Check that the inputs are sound
	int m = A.rows();
	int n = A.cols();
	if(xd.size() != n or y.size() != m or x0.size() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions of input arguments do not match. "
		          << "The xd vector was " << xd.size() << "x1, "
		          << "the W matrix was " << W.rows() << "x" << W.cols() << ", "
		          << "the y vector was " << y.size() << "x1, "
		          << "the A matrix was " << m << "x" << n << ", "
		          << "the xMin vector was " << xMin.size() << "x1, "
		          << "the xMax vector was " << xMax.size() << "x1, and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else if(W.rows() != n or W.cols() != n)
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
		          << "but expected " << n << "x" << n << "." << std::endl;
		
		return x0;
	}
	else
	{
		// NOTE: MAYBE WE CAN DO A 2-STAGE QP TO IMPROVE RESULTS?
		// min || y - A*x || s.t. xMin <= x <= xMax to ensure x is feasible
		// then override y = A*x with solution for x, then solve:
		// min || xd - x || s.t. Ax = y, xMin <= x <= xMax
		
		// Set up QP problem in standard form
		
		// H = [ W A'
		//       A 0 ]
		
		Eigen::MatrixXf H = Eigen::MatrixXf::Zero(m+n,m+n);
		H.block(0,0,n,n) = W;
		H.block(0,n,n,m) = A.transpose();
		H.block(n,0,m,n) = A;
//		H.block(n,n,m,m) = Eigen::MatrixXf::Zero(m,n);
		
		// f = [  W*xd  ]
		//     [   y    ]
		Eigen::VectorXf f(m+n);
		f.block(0,0,n,1) = W*xd;
		f.block(n,0,m,1) = y;
		
		// B = [ -I 0 ]
		//     [  I 0 ]
		Eigen::MatrixXf B =    Eigen::MatrixXf::Zero(2*n,n+m);
		B.block(0,0,n,n)  = -1*Eigen::MatrixXf::Identity(n,n);
		B.block(n,0,n,n)  =    Eigen::MatrixXf::Identity(n,n);
		
		// c = [ -xMax ]
		//     [  xMin ]
		Eigen::VectorXf c(2*n);
		c.block(0,0,n,1) = -1*xMax;
		c.block(n,0,n,1) =    xMin;
		
		// start = [ x0 ]
		//         [ 0  ]
		// NOTE: TRY A'*lambda = f - H*x
		Eigen::VectorXf start = Eigen::VectorXf::Zero(m+n);
		start.block(0,0,n,1)  = x0;
		
		Eigen::VectorXf temp = solve(H,f,B,c,start);                                         // Solution is [ x' lambda']'

		return temp.block(0,0,n,1);                                                         // Return only the x vector
	}
}                  
