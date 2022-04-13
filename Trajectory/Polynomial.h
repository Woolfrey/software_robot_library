    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                            A base class for polynomial trajectories                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <Eigen/Geometry>                                                                          // Eigen::VectorXf, Eigen::AngleAxisf
#include <math.h>                                                                                  // M_PI
#include <vector>                                                                                  // std::vector

class Polynomial
{
	public:
	
		Polynomial();

		// Constructor with only positions, delegates zero velocity
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order) :
		Polynomial(startPoint,
                          endPoint,
                          Eigen::VectorXf::Zero(startPoint.size()),
                          Eigen::VectorXf::Zero(endPoint.size()),
                          startTime,
                          endTime,
                          order) {}
		
		// Constructor with positions and velocities
		Polynomial(const Eigen::VectorXf &startPoint,
			   const Eigen::VectorXf &endPoint,
			   const Eigen::VectorXf &startVelocity,
			   const Eigen::VectorXf &endVelocity,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order);
		
		// Constructor with orientation, delegates zero velocity
		Polynomial(const Eigen::AngleAxisf &startPoint,
			   const Eigen::AngleAxisf &endPoint,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order) :
		Polynomial(startPoint,
               	   endPoint,
               	   Eigen::Vector3f::Zero(),
               	   Eigen::Vector3f::Zero(),
               	   startTime,
               	   endTime,
               	   order) {}
		
		// Constructor with orientation and velocity
		Polynomial(const Eigen::AngleAxisf &startPoint,
			   const Eigen::AngleAxisf &endPoint,
			   const Eigen::Vector3f &startVelocity,
			   const Eigen::Vector3f &endVelocity,
			   const float &startTime,
			   const float &endTime,
			   const unsigned int &order);
		
		// Get the desired state for translations
		bool get_state(Eigen::VectorXf &pos,
			       Eigen::VectorXf &vel,
			       Eigen::VectorXf &acc,
			       const float &time);
		
		// Get the desired state for rotations
		bool get_state(Eigen::AngleAxisf &rot,
			       Eigen::Vector3f &vel,
			       Eigen::Vector3f &acc,
			       const float &time);
		
	private:
	
		bool isValid = false;                                                              // Won't do calcs if this is false
		Eigen::AngleAxisf R0;                                                              // Start rotation: R(t) = R0*dR(t)
		float t0, tf;                                                                      // Start time and end time			
		unsigned int m;                                                                    // Number of dimensions
		unsigned int n;                                                                    // Order of polynomial
		std::vector<std::vector<float>> coeff;                                            
		
		bool times_are_sound(const float &startTime, const float &endTime);
		
		bool compute_coefficients(const std::vector<float> &startPoint,
					   const std::vector<float> &endPoint,
					   const std::vector<float> &startVelocity,
					   const std::vector<float> &endVelocity);
		
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Constructor for trajectory over real numbers                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
Polynomial::Polynomial(const Eigen::VectorXf &startPoint,
		       const Eigen::VectorXf &endPoint,
		       const Eigen::VectorXf &startVelocity,
		       const Eigen::VectorXf &endVelocity,
		       const float &startTime,
		       const float &endTime,
		       const unsigned int &order):
		       m(startPoint.size()),
		       n(order),
		       t0(startTime),
		       tf(endTime)
{
	// If input vectors are not all the same length...
	if(endPoint.size()      != this->m
	or startVelocity.size() != this->m
	or endVelocity.size()   != this->m)
	{
		std::cerr << "[ERROR] [POLYNOMIAL] Constructor: "
			  << "Input vectors are not of equal length! "
			  << "Start point had " << startPoint.size() << " elements, "
			  << "end point had " << endPoint.size() << " elements, "
			  << "start velocity had " << startVelocity.size() << " elements, and "
			  << "end velocity had " << endVelocity.size() << " elements." << std::endl;
	}
	else if(times_are_sound(startTime, endTime))
	{
		// Compute the coefficients
		std::vector<float> p1, p2, v1, v2;
		for(int i = 0; i < this->m; i++)
		{
			p1.push_back( startPoint[i]    );
			p2.push_back( endPoint[i]      );
			v1.push_back( startVelocity[i] );
			v2.push_back( endVelocity[i]   );
		}
		this->isValid = compute_coefficients(p1, p2, v1, v2);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Constructor for trajectory over rotations                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
Polynomial::Polynomial(const Eigen::AngleAxisf &startPoint,
			const Eigen::AngleAxisf &endPoint,
			const Eigen::Vector3f &startVelocity,
			const Eigen::Vector3f &endVelocity,
			const float &startTime,
			const float &endTime,
			const unsigned int &order):
			m(3),
			n(order),
			t0(startTime),
			tf(endTime),
			R0(startPoint)                                                             // Need this to return the desired state
{
	if(times_are_sound(startTime, endTime))
	{
		// Get the difference in rotation
		Eigen::AngleAxisf dR = Eigen::AngleAxisf(startPoint.inverse()*endPoint);           // R0*dR = Rf ---> dR = R0^-1*Rf
		double angle = dR.angle();                                                         // Get the angle between the two
		if(angle > M_PI) angle = 2*M_PI - angle;                                           // If > 180 degrees, take shorter path
		Eigen::Vector3f axis = dR.axis();                                                  // Get the axis of rotation
		
		std::vector<float> p1, p2, v1, v2;
		for(int i = 0; i < 3; i++)
		{
			p1.push_back( 0.0              );                                          // Start at zero: R(0) = R0*dR(0) = R0
			p2.push_back( angle*axis[i]    );                                          // Finish at end: R(tf) = R0*dR(tf) = Rf
			v1.push_back( startVelocity[i] );                                                      
			v2.push_back( endVelocity[i]   );
		}
		this->isValid = compute_coefficients(p1, p2, v1, v2);
	}	
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Check that the start and end time are logical                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::times_are_sound(const float &startTime, const float &endTime)
{
	if(startTime == endTime)
	{
		std::cerr << "[ERROR] [POLYNOMIAL] Constructor: "
                         << "Cannot move in zero time! "
                         << "Start time was " << startTime << " seconds and "
                         << "end time was " << endTime << " seconds." << std::endl;
			  
		return false;
	}
	else if(startTime > endTime)
	{
		std::cout << "[WARNING] [POLYNOMIAL] Constructor: "
                         << "Start time of " << startTime << " seconds was greater than "
                         << "end time of " << endTime << " seconds. "
                         << "Swapping their values to avoid problems..." << std::endl;
				  
		float temp = this->t0;
		this->t0 = this->tf;
		this->tf = temp;
		
		return true;
	}
	else return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the polynomial coefficients                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::compute_coefficients(const std::vector<float> &startPoint,
				       const std::vector<float> &endPoint,
				       const std::vector<float> &startVelocity,
				       const std::vector<float> &endVelocity)
{
	// Decent explanation here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (Fourth Edition)
        // Springer,  p. 258 - 276
        
	float dt = this->tf - this->t0;                                                            // Time difference
	this->coeff.resize(this->m);                                                              

	if(this->n == 3) // Cubic Polynomial : x(t) = a*dt^3 + b*dt^2 + c*dt + d
	{		
		for(int i = 0; i < this->m; i++)
		{
			this->coeff[i].resize(4);
			
			float dx = endPoint[i] - startPoint[i];
			
			this->coeff[i][3] = startPoint[i];                                                  // d
			this->coeff[i][2] = startVelocity[i];                                               // c
			this->coeff[i][1] = 3*dx/(dt*dt) - (endVelocity[i] + 2*this->coeff[i][2])/dt;       // b
			this->coeff[i][0] = (endVelocity[i] - this->coeff[i][2])/(dt*dt) - 2*dx/(dt*dt*dt); // a
		}	
		
		return true;
	}
	else if(this->n == 5) // Quintic Polynomial : x(t) = a*dt^5 + b*dt^4 + c*dt^3 + d*dt^2 + e*dt + f
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
		
		for(int i = 0; i < this->m; i++)
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
			  << "Polynomial order was " << this->n << " "
			  << "but can currently only do 3 or 5." << std::endl;
				  
		return false;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::get_state(Eigen::VectorXf &pos,
			    Eigen::VectorXf &vel,
			    Eigen::VectorXf &acc,
			    const float &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [POLYNOMIAL] get_state(): "
			  << "Something went wrong during the construction of this object. "
			  << "Cannot obtain the state." << std::endl;
				  
		return false;
	}
	else if(pos.size() != this->m
	     or vel.size() != this->m
	     or acc.size() != this->m)
	{
		std::cerr << "[ERROR] [POLYNOMIAL] get_state(): "
			  << "Input vectors are not the correct length. "
			  << "This object has " << this->m << " dimensions but "
			  << "pos had " << pos.size() << " elements, "
			  << "vel had " << vel.size() << " elements, and "
			  << "acc had " << acc.size() << " elements." << std::endl;
		
		return false;
	}
	else
	{
		float dt;
		if(time < this->t0)      dt = 0.0;                                                 // Not yet begun, remain at start
		else if(time < this->tf) dt = time - this->t0;                                     // Somewhere inbetween...
		else                     dt = this->tf - this->t0;                                 // Finished, remain at end
		
		for(int i = 0; i < this->m; i++)
		{
			for(int j = 1; j <= this->n; j++)
			{
				pos[i] +=                           this->coeff[i][this->n-j]*pow(dt,this->n-j);
				vel[i] +=               (this->n-j)*this->coeff[i][this->n-j]*pow(dt,this->n-j-1);
				acc[i] += (this->n-j-1)*(this->n-j)*this->coeff[i][this->n-j]*pow(dt,this->n-j-2);
			}
		}
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Get the desired orientation                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Polynomial::get_state(Eigen::AngleAxisf &rot,
			    Eigen::Vector3f &vel,
			    Eigen::Vector3f &acc,
			    const float &time)
{
	Eigen::VectorXf p(3), v(3), a(3);
	if(get_state(p, v, a, time))                                                               // Get the desired state
	{
		rot = this->R0*Eigen::AngleAxisf(p.norm(), p.normalized());                        // Separate out angle and axis
		return true;
	}
	else return false;
}
#endif
