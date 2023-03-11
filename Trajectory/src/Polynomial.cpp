#include <Polynomial.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Constructor for a trajectory representing translation                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Polynomial::Polynomial(const Eigen::VectorXf &startPoint,
                       const Eigen::VectorXf &endPoint,
                       const Eigen::VectorXf &startVelocity,
                       const Eigen::VectorXf &endVelocity,
                       const float           &startTime,
                       const float           &endTime,
                       const unsigned int    &order)
                       :
                       TrajectoryBase(startTime, endTime, startPoint.size()),                       // Construct base object
                       _order(order)                                                                // Order of polynomial
{
	if(startPoint.size()    != endPoint.size()
	or endPoint.size()      != startVelocity.size()
	or startVelocity.size() != endVelocity.size())
	{
		auto startPointSize = std::to_string(startPoint.size());
		auto endPointSize   = std::to_string(endPoint.size());
		auto startVelSize   = std::to_string(startVelocity.size());
		auto endVelSize     = std::to_string(endVelocity.size());
		
		std::string message = "[ERROR] [POLYNOMIAL TRAJECTORY] Constructor: "
		                      "Dimensons of input arguments do not match. "
		                      "Start point had " + startPointSize + " elements, "
		                      "end point had " + endPointSize + " elements, "
		                      "start velocity had " + startVelSize + " elements, and "
		                      "end velocity had " + endVelSize + " elements.";
		
		throw std::runtime_error(message);
	}
	else
	{
		if(not compute_coefficients(startPoint, endPoint, startVelocity, endVelocity))      // As it says on the label
		{
			std::string message = "[ERROR] [POLYNOMIAL TRAJECTORY] Constructor: "
			                      "Something went wrong during the construction of this object.";
			                      
			throw std::runtime_error(message);
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the polynomial coefficients                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::compute_coefficients(const Eigen::VectorXf &startPoint,
                                      const Eigen::VectorXf &endPoint,
                                      const Eigen::VectorXf &startVelocity,
                                      const Eigen::VectorXf &endVelocity)
{
	// Decent explanation here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (Fourth Edition)
        // Springer,  p. 258 - 276
        
        float dt = this->_startTime - this->_endTime;
        
        this->coeff.resize(this->dimensions);
        
        if(this->_order == 3) // Cubic polynomial : x(t) = a*dt^3 + b*dt^2 + c*dt^1 + d*dt^0
        {
		for(int i = 0; i < this->dimensions; i++)
		{
			this->coeff[i].resize(4);
			
			float dx = endPoint(i) - startPoint(i);
			
			this->coeff[i][3] = startPoint(i);                                                  // d
			this->coeff[i][2] = startVelocity(i);                                               // c
			this->coeff[i][1] = 3*dx/(dt*dt) - (endVelocity(i) + 2*this->coeff[i][2])/dt;       // b
			this->coeff[i][0] = (endVelocity(i) - this->coeff[i][2])/(dt*dt) - 2*dx/(dt*dt*dt); // a
		}	
		
		return true;
        }
        else if(this->_order == 5) // Quintic polynomial: x(t) + a*dt^5 + b*dt^4 + c*dt^3 + d*dt^2 + e*dt^1 + f*dt^0
        {
		//    a*dt^5 +    b*dt^4 +   c*dt^3 = x_f - x_0 - dt*xdot_0                        (final position constraint)
		//  5*a*dt^4 +  4*b*dt^3 + 2*c*dt^2 = xdot_f - xdot_0                              (final velocity constraint)
		// 20*a*dt^3 + 12*b*dt^2 + 6*c*dt   = 0                                            (final acceleration constraint)

		// [    dt^5     dt^4    dt^3 ][a] = [ x_f - x_0 - dt*xdot_0 ]
		// [  5*dt^4   4*dt^3  3*dt^2 ][b] = [    xdot_f - xdot_0    ]
		// [ 20*dt^3  12*dt^2  6*dt   ][c] = [          0            ]

		// [    dt^2     dt       1   ][a] = [ (x_f - x_0)/dt^3 - xdot_0/dt^2 ]            y1
		// [  5*dt^2   4*dt       3   ][b] = [     (xdot_f - xdot_0)/dt^2     ]            y2
		// [ 20*dt^2  12*dt       6   ][c] = [               0                ]

		// Then solve the above through Gaussian elimination, starting with c.
		
		for(int i = 0; i < this->dimensions; i++)
		{
			this->coeff[i].resize(6);
		
			float y1 = (endPoint[i] - startPoint[i])/(dt*dt*dt) - startVelocity[i]/(dt*dt);
			float y2 = (endVelocity[i] - startVelocity[i])/(dt*dt);
			
			this->coeff[i][5] = startPoint[i];                                         // f = x_0
			this->coeff[i][4] = startVelocity[i];                                      // e = xdot_0
			this->coeff[i][3] = 0.0;                                                   // d = 0.5*xddot_0 (assume 0)
			this->coeff[i][2] = 10*y1 - 4*y2;                                          // c
			this->coeff[i][1] = (7*y2 - 15*y1)/dt;                                     // b
			this->coeff[i][0] = (6*y1 - 3*y2)/(dt*dt);                                 // a
		}
		
		return true;
        }
        else
        {
        	std::cout << "[ERROR] [POLYNOMIAL] compute_coefficients(): "
        	          << "Polynomial order was " << this->_order << " but currently "
        	          << "only 3 or 5 is available.";
        	          
	        return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::get_state(Eigen::VectorXf &pos,
			   Eigen::VectorXf &vel,
			   Eigen::VectorXf &acc,
			   const float     &time)
{
	if(pos.size() != vel.size()
	or vel.size() != acc.size())
	{
		std::cerr << "[ERROR] [POLYNOMIAL] get_state() : " 
	                  << "Input vectors were not of equal length! "
		          << "The position argument had " << pos.size() << " elements, "
		          << "the velocity argument had " << vel.size() << " elements, and "
		          << "the acceleration argument had " << acc.size() << " elements.";
		
		return false;
	}
	else if(pos.size() != this->dimensions)
	{
		std::cerr << "[ERROR] [POLYNOMIAL TRAJECTORY] get_state() : "
		          << "This object has " << this->dimensions << " dimensions, but "
		          << "the input vectors had " << pos.size() << " elements.";
		          
		return false;
	}
	else
	{
		float dt;
		
		if(time < this->_startTime)     dt = 0.0;                                           // Not yet begun, stay at beginning
		else if (time < this->_endTime) dt = time - this->_startTime;                       // Somewhere inbetween...
		else                            dt = this->_startTime - this->_endTime;             // Finished, remain at end

		for(int i = 0; i < this->dimensions; i++)
		{
			for(int j = 1; j <= this->_order; j++)
			{
				pos(i) +=                                     this->coeff[i][this->_order-j]*pow(dt,this->_order-j);
				vel(i) +=                    (this->_order-j)*this->coeff[i][this->_order-j]*pow(dt,this->_order-j-1);
				acc(i) += (this->_order-j-1)*(this->_order-j)*this->coeff[i][this->_order-j]*pow(dt,this->_order-j-2);
			}
		}
		
		return true;
	}
}
