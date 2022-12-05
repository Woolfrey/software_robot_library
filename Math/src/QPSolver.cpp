#include <iostream>                                                                                 // std::cerr
#include <math.h>
#include <QPSolver.h>                                                                               // Declaration of functions
#include <vector>                                                                                   // std::vector


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve a generic QP problem min 0.5*x'*H*x + x'*f                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::solve(const Eigen::MatrixXf &H,
                                const Eigen::VectorXf &f)
{
	if(H.rows() != H.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Expected a square matrix for the Hessian H, but it was "
		          << H.rows() << "x" << H.cols() << "." << std::endl;
		
		return Eigen::VectorXf::Zero(f.size());
	}
	else if(H.rows() != f.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Dimensions for the decision variable do not match! "
		          << "The Hessian matrix H was " << H.rows() << "x" << H.cols() << " "
		          << "and the f vector was " << f.size() << "x1." << std::endl;
		
		return Eigen::VectorXf::Zero(f.size());
	}
	else	return H.partialPivLu().solve(-f);                                                  // Too easy lol
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve a constrained QP problem: min 0.5*x'*H*x + x'*f subject to: B*x >= z           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::solve(const Eigen::MatrixXf &H,
                                const Eigen::VectorXf &f,
                                const Eigen::MatrixXf &B,
                                const Eigen::VectorXf &z,
                                const Eigen::VectorXf &x0)
{
	// Check the inputs are sound
	int n = x0.size();
	if(H.rows()!= H.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Hessian matrix H is not square! "
		          << "Your input was " << H.rows() << "x" << H.cols() << "." << std::endl;
		return x0;
	}
	else if(f.size() != H.cols() or H.cols() != B.cols() or H.cols() != x0.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Decision variable dimensions do not match! "
		          << "The Hessian matrix H was " << H.rows() << "x" << H.cols() << ", "
		          << "the f vector had " << f.size() << " elements, "
		          << "the B matrix had " << B.cols() << " columns, "
		          << "and the start point x0 had " << x0.size() << " elements." << std::endl;
		return x0;
	}
	else if(B.rows() != z.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] solve(): "
		          << "Constraint dimensions do not match! "
		          << "The constraint matrix B had " << B.rows() << " rows, "
		          << "and the constraint vector z had " << z.size() << " elements." << std::endl;
		return x0;
	}
	else
	{
		// Solve the following optimization problem with Guass-Newton method:
		//
		//    min f(x) = 0.5*x'*H*x + x'*f - u*sum(log(d_i))
		//
		// where d_i = b_i*x - c_i is the distance to the constraint
		//
		// Then the gradient and Hessian are:
		//
		//    g(x) = H*x + f - u*sum((1/d_i)*b_i')
		//
		//    I(x) = H + u*sum((1/(d_i^2))*b_i'*b_i)
		
		// Local variables
	
		Eigen::MatrixXf I;                                                                  // Hessian matrix
		Eigen::VectorXf g(n);                                                               // Gradient vector
		Eigen::VectorXf dx = Eigen::VectorXf::Zero(n);                                      // Newton step = -I^-1*g
		Eigen::VectorXf x = x0;                                                             // Assign initial state variable
		
		float alpha;                                                                        // Scalar for Newton step
		float beta  = this->beta0;                                                          // Shrinks barrier function
		float u     = this->u0;                                                             // Scalar for barrier function
	
		int numConstraints = B.rows();
		
		std::vector<float> d; d.resize(numConstraints);
		
		// Do some pre-processing
		std::vector<Eigen::VectorXf> bt(numConstraints);
		std::vector<Eigen::MatrixXf> btb(numConstraints);
		for(int j = 0; j < numConstraints; j++)
		{
			bt[j]  = B.row(j).transpose();                                              // Row vectors of B transposed
			btb[j] = bt[j]*bt[j].transpose();                                           // Outer product of row vectors
		}

		// Run the interior point method
		for(int i = 0; i < this->steps; i++)
		{
			// (Re)set values for new loop
			g.setZero();                                                                // Gradient vector
			I = H;                                                                      // Hessian for log-barrier function
			
			// Compute distance to each constraint
			for(int j = 0; j < numConstraints; j++)
			{
				d[j] = bt[j].dot(x) - z(j);                                         // Distance to jth constraint
				
				if(d[j] <= 0)
				{
					if(i == 0)
					{
						std::cerr << "[ERROR] [QPSOLVER] solve(): "
						          << "Start point x0 is outside the constraints!" << std::endl;
						return x0;
					}
			
					d[j] = 1e-03;                                               // Set a small, non-zero value
					u *= 100;
				}
						
				g += -(u/d[j])*bt[j];                                               // Add up gradient vector
				I +=  (u/(d[j]*d[j]))*btb[j];                                       // Add up Hessian
				
			}
			
			g += H*x + f;                                                               // Finish summation of gradient vector

			dx = I.partialPivLu().solve(-g);                                            // LU decomposition seems most stable
			
			// Shrink step size until within the constraint
			alpha = this->alpha0;
			for(int j = 0; j < numConstraints; j++)
			{
				while( d[j] + alpha*bt[j].dot(dx) < 0) alpha *= this->alphaMod;
			}

			if(alpha*dx.norm() < this->tol) break;
			
			// Update values for next loop
			x += alpha*dx;                                                              // Increment state
			u *= beta;                                                                  // Decrease barrier function
		}
		
		return x;
	}
}	

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Solve an unconstrained least squares problem: min 0.5(y-A*x)'*W*(y-A*x)              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A,
                                        const Eigen::MatrixXf &W)
{
	if(A.rows() < A.cols())                                                                     // Redundant system, use other function
	{
		int n = A.cols();
		return least_squares(Eigen::VectorXf::Zero(n),Eigen::MatrixXf::Identity(n,n),A,y);
	}
	if(W.rows() != W.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Expected a square weighting matrix, "
		          << "but it was " << W.rows() << "x" << W.cols() << std::endl;
		          
		return Eigen::VectorXf::Zero(A.cols());
	}
	else if(y.size() != W.rows() and A.rows() != y.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions do not match! "
		          << "The y vector had " << y.size() << " elements, "
		          << "the A matrix had " << A.rows() << " rows, "
		          << "and the W matrix was " << W.rows() << "x" << W.cols() << std::endl;
		          
		return Eigen::VectorXf::Zero(A.cols());
	}
	else	return (A.transpose()*W*A).partialPivLu().solve(A.transpose()*W*y);                 // x = (A'*W*A)^-1*A'*W*y
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
	if(W.rows() != W.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Expected the weighting matrix W to be square, "
		          << "but it was " << W.rows() << "x" << W.cols() << std::endl;
		          
		return x0;
	}
	else if(y.size() != W.rows() and A.rows() != y.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions do not match! "
		          << "The y vector had " << y.size() << " elements, "
		          << "the A matrix had " << A.rows() << " rows, "
		          << "and the W matrix was " << W.rows() << "x" << W.cols() << std::endl;
		          
		return x0;
	}
	else if(A.cols() != xMin.size() or xMin.size() != xMax.size() or xMax.size() != x0.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions for the decision variable to no match! "
		          << "The A matrix had " << A.cols() << " columns, "
		          << "the xMin vector had " << xMin.size() << " elements, "
		          << "the xMax vector had " << xMax.size() << " elements, "
		          << "and the start point x0 had " << x0.size() << " elements." << std::endl;
		
		return x0;
	}
	else
	{
		int n = x0.size();
		
		// Set up constraint matrices in standard form Bx >= c where:
		// B*x = [ -I ] >= [ -xMax ]
		//       [  I ]    [  xMin ]
		Eigen::MatrixXf B(2*n,n);
		B.block(n,0,n,n).setIdentity();
		B.block(0,0,n,n) = -B.block(n,0,n,n);

		Eigen::VectorXf z(2*n);
		z.head(n) = -xMax;
		z.tail(n) =  xMin;
		
		Eigen::MatrixXf AtW = A.transpose()*W;                                              // Makes calcs a little simpler

		return solve(AtW*A,-AtW*y, B, z, x0);                                               // Convert to standard form and solve
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //       Solve a least squares problem of the form min 0.5*(xd-x)'*W*(xd-x)  s.t. A*x = y        //
///////////////////////////////////////////////////////////////////////////////////////////////////                                  
Eigen::VectorXf QPSolver::least_squares(const Eigen::VectorXf &xd,
                                        const Eigen::MatrixXf &W,
                                        const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &A)
{
	if(W.rows() != W.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Expected a square weighting matrix W, but it was " 
		          << W.rows() << "x" << W.cols() << "." << std::endl;
		          
		return Eigen::VectorXf::Zero(xd.size());
	}
	else if(xd.size() != W.rows() or W.cols() != A.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions for the decision variable do not match! "
                          << "The desired vector xd had " << xd.size() << " elements, "
                          << "the weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
                          << "and the constraint matrix A had " << A.cols() << " columns." << std::endl;
                
                return Eigen::VectorXf::Zero(xd.size());
        }
        else if(y.size() != A.rows())
        {
        	std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
        	          << "Dimensions for the equality constraint do not match! "
        	          << "The y vector had " << y.size() << " elements, "
        	          << "and the A matrix had " << A.rows() << " rows." << std::endl;
        	
        	return Eigen::VectorXf::Zero(xd.size());
        }
        else
        {   	
		// Lagrangian L = 0.5*x'*W*x - x'*W*xd + (A*x - y)'*lambda,
		// Solution exists for:
		//
		// [ dL/dlambda ]  =  [ 0   A ][ lambda ] - [   y  ] = [ 0 ]
		// [   dL/dx    ]     [ A'  W ][   x    ]   [ W*xd ]   [ 0 ]
		//
		// but we can speed up calcs and skip solving lambda if we are clever.
		
		int m = A.rows();
		int n = A.cols();
		
		Eigen::MatrixXf H = Eigen::MatrixXf::Zero(m+n,m+n);
//		H.block(0,0,m,m) = Eigen::MatrixXf::Zero(m,n);
		H.block(0,m,m,n) = A;
		H.block(m,0,n,m) = A.transpose();
		H.block(m,m,n,n) = W;
		
		Eigen::MatrixXf Q, R;
		if(get_qr_decomposition(H,Q,R))
		{
			return backward_substitution(Q.block(0,m,m,n).transpose()*y                 //   Q12'*y
			                           + Q.block(m,m,n,n).transpose()*W*xd,             // + Q22*W*xd 
			                             R.block(m,m,n,n),                              //   R22
			                             xd);
		}
		else    return Eigen::VectorXf::Zero(xd.size());
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
	if(W.rows() != W.cols())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Expected a weighting matrix W, but it was " 
		          << W.rows() << "x" << W.cols() << "." << std::endl;
		          
		return x0;
	}
	else if(xd.size() != W.rows() or W.cols() != A.cols() or A.cols() != x0.size() 
	     or xMin.size() != x0.size() or xMax.size() != x0.size())
	{
		std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
		          << "Dimensions for the decision variable do not match! "
                          << "The desired vector xd had " << xd.size() << " elements, "
                          << "the weighting matrix W was " << W.rows() << "x" << W.cols() << ", "
                          << "the constraint matrix A had " << A.cols() << " columns, "
                          << "the xMin vector had " << xMin.size() << " elements, "
                          << "the xMax vector had " << xMax.size() << " elements, "
                          << "and the start point x0 had " << x0.size() << " columns." << std::endl;
                
                return x0;
        }
        else if(y.size() != A.rows())
        {
        	std::cerr << "[ERROR] [QPSOLVER] least_squares(): "
        	          << "Dimensions for the equality constraint do not match! "
        	          << "The y vector had " << y.size() << " elements, "
        	          << "and the A matrix had " << A.rows() << " rows." << std::endl;
        	
        	return x0;
        }
	else
	{
		// Convert to standard form 0.5*x'*H*x + x'*f subject to B*x >= z
		// where "x" is now [lambda' x' ]'
		
		unsigned int m = A.rows();
		unsigned int n = A.cols();
		
		// H = [ 0  A ]
		//     [ A' W ]
		Eigen::MatrixXf H(m+n,m+n);
		H.block(0,0,m,m).setZero();
		H.block(0,m,m,n) = A;
		H.block(m,0,n,m) = A.transpose();
		H.block(m,m,n,n) = W;
		
		// B = [ 0 -I ]
		//     [ 0  I ]
		Eigen::MatrixXf B(2*n,m+n);
		B.block(0,0,2*n,m).setZero();
		B.block(n,m,  n,n).setIdentity();
		B.block(0,m,  n,n) = -B.block(n,m,n,n);

		// z = [ -xMax ]
		//     [  xMin ]
		Eigen::VectorXf z(2*n);
		z.head(n) = -xMax;
		z.tail(n) =  xMin;

		// f = [   -y  ]
		//     [ -W*xd ]
		Eigen::VectorXf f(m+n);
		f.head(m) = -y;
		f.tail(n) = -W*xd;
		
		Eigen::VectorXf startPoint(m+n);
		startPoint.head(m) = (A*W.partialPivLu().inverse()*A.transpose()).partialPivLu().solve(A*xd - y);
		startPoint.tail(n) = x0;
		
		return (solve(H,f,B,z,startPoint)).tail(n);                                         // Convert to standard form and solve
	}
}                  
