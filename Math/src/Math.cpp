#include <iostream>                                                                                 // std::cerr
#include <Math.h>                                                                                   // Declaration of function names


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Decompose a positive-definite matrix A in to L*L'                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf get_cholesky_decomp(const Eigen::MatrixXf &A)
{
	int n = A.rows();
	Eigen::MatrixXf L = Eigen::MatrixXf::Zero(n,n);
	if(A.rows() != A.cols())
	{
		std::cerr << "[ERROR] get_cholesky_decomp(): "
		          << "Expected a square matrix, but "
		          << "the input was " << A.rows() << "x" << A.cols() << "." << std::endl;
	}
	else
	{
                // Cholesky–Banachiewicz algorithm (row-wise)
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j <= i; j++)
			{
				float sum = 0.0;
				for(int k = 0; k <= j; k++) sum += L(i,k) * L(j,k);
				
				if(i == j) L(i,j) = sqrt(A(i,i) - sum);
				else       L(i,j) = (A(i,j) - sum)/L(j,j);
			}
		}
	}
	
	return L;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the inverse of a positive-definite matrix                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf get_cholesky_inverse(const Eigen::MatrixXf &A)
{
	int n = A.rows();
	
	if(A.cols() != n)
	{
		std::cerr << "[ERROR] get_cholesky_inverse(): "
		          << "Expected a square matrix but "
		          << "the input was " << A.rows() << "x" << A.cols() << "." << std::endl;
		
		return Eigen::MatrixXf::Zero(n,n);
	}
	else
	{
		Eigen::MatrixXf L = get_cholesky_decomp(A);                                         // As it says on the label
		Eigen::MatrixXf I = L;                                                              // This will be in the inverse
		
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j <= i; j++)
			{
				if(i == j)
				{
					I(i,i) = 1/(L(i,i));
				}
				else
				{
					float sum = 0.0;
					
					for(int k = j; k < i; k++) sum += L(i,k)*I(k,j);

					I(i,j) = -sum/L(i,i);
				}
			}
		}
		
		return I.transpose()*I;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //           Decompose a matrix A in to orthogonal matrix Q and triangular matrix R              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool get_qr_decomposition(const Eigen::MatrixXf &A,
                                Eigen::MatrixXf &Q,
                                Eigen::MatrixXf &R)
{
	int m = A.rows();
	int n = A.cols();
	
	if(m < n)
	{
		std::cerr << "[ERROR] get_qr_decomp() "
		          << "Matrix A has " << A.rows() << " rows which is less than its "
		          << A.cols() << " columns. Cannot solve the QR decomposition." << std::endl;
		
		return false;
	}
	else
	{
		// Schwarz-Rutishauser Algorithm.
		// A full decomposition of an mxn matrix A (m > n) is:
		//
		// [ Q_r Q_n ][ R ]  = A
		//            [ 0 ]
		//
		// where: - Q_r is mxn,
		//        - Q_n is mx(m-n)
		//        - R   is nxn
		//
		// The null space of A is obtained with N = Q_n*Q_n'.
		//
		// This algorithm returns Q_r, and R
		
		Q = A;
		R = Eigen::MatrixXf::Zero(n,n);
		
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < j; i++)
			{
				R(i,j)   = Q.col(i).dot(Q.col(j));                                  // Project the columns
				Q.col(j) = Q.col(j) - R(i,j)*Q.col(i);
			}
			
			R(j,j) = Q.col(j).norm();
			
			if(abs(R(j,j)) > 1E-6) Q.col(j) /= R(j,j);
			else                   Q.col(j).setZero();
		}
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Get the inverse of a matrix                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A)                                               // Get the inverse of a matrix
{                  
	Eigen::JacobiSVD<Eigen::MatrixXf> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);        // Get the SVD decomposition
	Eigen::MatrixXf V = SVD.matrixV();                                                          // V matrix
	Eigen::MatrixXf U = SVD.matrixU();                                                          // U matrix
	Eigen::VectorXf s = SVD.singularValues();                                                   // Get the singular values
	Eigen::MatrixXf invA(A.cols(), A.rows()); invA.setZero();                                   // Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-05) invA(i,k) += (V(i,j)*U(k,j))/s(j);                  // Fast inverse
//				else              invA(i,k) += 0;                                     // Ignore singular directions         
			}
		}
	}
	return invA;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the weighted pseudoinverse of a matrix                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A,
                            const Eigen::MatrixXf &W)
{
	if(W.rows() != W.cols())
	{
		std::cerr << "[ERROR] [SERIALKINCONTROL] get_weighted_inverse() : "
		          << "Weighting matrix must be square. Your input had " << W.rows() << " rows and "
		          << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
                         
		return get_inverse(A);                                                              // Ignore the weighting matrix
	}
	else if(A.cols() == W.rows())                                                               // Overdetermined system
	{
		Eigen::MatrixXf invWAt = get_inverse(W)*A.transpose();
		
		return invWAt*get_inverse(A*invWAt);                                                // W^-1*A'*(A*W^-1*A')^-1
	}
	else if(W.cols() == A.rows())                                                               // Underdetermined system
	{
		Eigen::MatrixXf AtW = A.transpose()*W;
		
		return get_inverse(AtW*A)*AtW;                                                      // (A'*W*A)^-1*A'*W
	}
	else
	{
		std::cerr << "[WARNING] [SERIALKINCONTROL] get_weighted_inverse() : "
		          << "Input matrices do not have compatible dimensions. Matrix A has " << A.rows()
		          << " rows and " << A.cols() << " columns, and matrix W has " << W.rows()
		          << " rows and " << W.cols() << " columns. Ignoring the weighting matrix..." << std::endl;
                         
		return get_inverse(A);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve a linear system of equations given by y = Ax                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf solve_linear_system(const Eigen::VectorXf &y,
                                    const Eigen::MatrixXf &A,
				     const Eigen::VectorXf &x0)
{
	int m = A.rows();
	int n = A.cols();
	
	// Check that the inputs are sound
	if(y.size()  != m or x0.size() != n)
	{
		std::cerr << "[ERROR] solve_linear_system(): "
		          << "Dimensions of inputs do not match! "
		          << "The y vector was " << y.size() << "x1 ,"
		          << "the A matrix was " << m << "x" << n << ", and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else
	{
		Eigen::VectorXf x = x0;                                                             // Value to be returned
		Eigen::MatrixXf Q, R;                                                               // Used in QR decomposition
		
		if(m == n)                                                                          // Fully determined
		{
			if(get_qr_decomposition(A,Q,R)) x = solve_triangular_system(Q.transpose()*y, R, x0);
		}		
		else if(m < n)                                                                      // Overdetermined, find minimum  ||x||^2                                                                        
		{
			// M = [ 0  A ]
			//     [ A' W ]
			Eigen::MatrixXf At = A.transpose();
			Eigen::MatrixXf M  = Eigen::MatrixXf::Zero(m+n,m+n);
//			M.block(0,0,m,m)   = Eigen::MatrixXf::Zero(m,m);
			M.block(0,m,m,n)   = A;
			M.block(m,0,n,m)   = A.transpose();
			M.block(m,m,n,n)   = Eigen::MatrixXf::Identity(n,n);
			
			if(get_qr_decomposition(M,Q,R))
			{
					Eigen::MatrixXf R22 = R.block(m,m,n,n);
					Eigen::MatrixXf Q12 = Q.block(0,m,m,n);
					
					x = solve_triangular_system(Q12.transpose()*y, R22, x0);
			}
		}
		else // m > n                                                                       // Underdetermined, minimize ||y-A*x||^2
		{
			Eigen::MatrixXf At = A.transpose();
			if(get_qr_decomposition(At*A,Q,R)) x = solve_triangular_system(Q.transpose()*At*y, R, x0);
		}
		
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //        Solve a linear system of equations given by y = Ax where A is positive-definite        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf solve_cholesky_system(const Eigen::VectorXf &y, const Eigen::MatrixXf &A)
{
	int n = A.rows();
	if(A.cols() != n)
	{
		std::cerr << "[ERROR] solve_cholesky_system(): "
		          << "Expected a square A matrix but it was "
		          << A.rows() << "x" << A.cols() << "." << std::endl;
		          
		return Eigen::VectorXf::Zero(n);
	}
	else if(y.size() != n)
	{
		std::cerr << "[ERROR] solve_cholesky_system(): "
		          << "Expected a " << n << "x1 vector for y, but "
		          << "it was " << y.size() << "x1." << std::endl;
		
		return Eigen::VectorXf::Zero(n);
	}
	else
	{
		Eigen::MatrixXf L = get_cholesky_decomp(A);
		
		return back_substitution(forward_substitution(y,L), L.transpose()); // lol too easy
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = U*x, where U is an upper-triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf back_substitution(const Eigen::VectorXf &y, const Eigen::MatrixXf &U)
{
	int n = U.cols();
	Eigen::VectorXf x(n);
	
	if(y.size() != n or U.rows() != n)
	{
		std::cerr << "[ERROR] back_substitution(): "
		          << "Dimensions of inputs do not match. "
		          << "The y vector had " << y.size() << " elements, and "
		          << "the U matrix had " << U.rows() << "x" << U.cols() << " elements." << std::endl;
		          
		x.setZero();
		return x;
	}
	else
	{
		for(int i = n-1; i >= 0; i--)
		{
			float sum = 0.0;
			for(int j = i+1; j < n; j++) sum += U(i,j)*x(j);
			
			if(U(i,i) < 1E-6) x(i) = 0.0;
			else              x(i) = (y(i)-sum)/U(i,i);
		}
		
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = L*x, where L is an lower-triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf forward_substitution(const Eigen::VectorXf &y, const Eigen::MatrixXf &L)
{
	int n = L.cols();
	
	if(y.size() != n or L.rows() != n)
	{
		std::cerr << "[ERROR] forward_substitution(): "
		          << "Dimensions of inputs do not match. "
		          << "The y vector had " << y.size() << " elements, and "
		          << "the L matrix had " << L.rows() << "x" << L.cols() << " elements." << std::endl;
		
		return Eigen::VectorXf::Zero(n);
	}
	else
	{
		Eigen::VectorXf x(n);
		
		for(int i = 0; i < n; i++)
		{
			float sum = 0.0;
			for(int j = 0; j < i; j++) sum += L(i,j)*x(j);
			
			if(L(i,i) < 1E-6) x(i) = 0.0;
			else              x(i) = (y(i) - sum)/L(i,i);
		}
		
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = U*x, where U is an upper-triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf solve_triangular_system(const Eigen::VectorXf &y,
                                        const Eigen::MatrixXf &U,
                                        const Eigen::VectorXf &x0)
{
	int n = U.cols();
	
	// Check that the inputs are sound
	if(y.size() != n or U.rows() != n or x0.size() != n)
	{
		std::cerr << "[ERROR] solve_triangular_system(): "
		          << "Dimensions of inputs do not match. "
		          << "The y vector had " << y.size() << " elements, "
		          << "the U matrix had " << U.rows() << "x" << U.cols() << " elements, and "
		          << "the x0 vector had " << x0.size() << " elements." << std::endl;
		
		return x0;
	}
	else
	{
		Eigen::VectorXf x(n);                                                               // Value to be returned
		
		for(int i = n-1; i >=0; i--)                                                        // Solve backwards from the last element
		{
			float sum = 0.0;
			for(int j = i+1; j < n; j++) sum += U(i,j)*x(j);                            // Compute backwards recursions
			
			if(U(i,i) < 1E-5) x(i) = x0(i);                                             // Near singular, so use default value
			else              x(i) = ( y(i) - sum )/U(i,i);                             // Compute ith value
		}
		
		return x;
	}
}
