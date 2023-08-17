    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                            A base class for polynomial trajectories                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <TrajectoryBase.h>
#include <string>                                                                                   // std::cerr, std::cout
#include <vector>                                                                                   // std::vector

using namespace Eigen;                                                                              // Eigen::Vector
using namespace std;

template <class DataType>
class Polynomial : public TrajectoryBase<DataType>
{
	public:
		Polynomial() {}                                                                     // Empty constructor

		// Constructor for translations; delegate zero velocity
		Polynomial(const Vector<DataType,Dynamic> &startPoint,
			   const Vector<DataType,Dynamic> &endPoint,
			   const DataType                 &startTime,
			   const DataType                 &endTime,
			   const unsigned int             &order)
		:
		Polynomial(startPoint,
                           endPoint,
                           Vector<DataType,Dynamic>::Zero(startPoint.size()),
                           Vector<DataType,Dynamic>::Zero(endPoint.size()),
                           startTime,
                           endTime,
                           order) {}
		
		// Constructor with positions and endpoint velocities
		Polynomial(const Vector<DataType,Dynamic> &startPoint,
			   const Vector<DataType,Dynamic> &endPoint,
			   const Vector<DataType,Dynamic> &startVelocity,
			   const Vector<DataType,Dynamic> &endVelocity,
			   const DataType                 &startTime,
			   const DataType                 &endTime,
			   const unsigned int             &order);
		
		// Get the desired state
		bool get_state(Vector<DataType,Dynamic> &pos,
			       Vector<DataType,Dynamic> &vel,
			       Vector<DataType,Dynamic> &acc,
			       const DataType           &time);
		
	private:
		unsigned int _order;
		
		std::vector<std::vector<DataType>> coeff;                                           // Polynomial coefficients
		
		bool compute_coefficients(const Vector<DataType,Dynamic> &startPoint,
					  const Vector<DataType,Dynamic> &endPoint,
					  const Vector<DataType,Dynamic> &startVelocity,
					  const Vector<DataType,Dynamic> &endVelocity);
		
};                                                                                                  // Semicolon needed after class declaration


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Cosntructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Polynomial<DataType>::Polynomial(const Vector<DataType,Dynamic> &startPoint,
                                 const Vector<DataType,Dynamic> &endPoint,
                                 const Vector<DataType,Dynamic> &startVelocity,
                                 const Vector<DataType,Dynamic> &endVelocity,
                                 const DataType                 &startTime,
                                 const DataType                 &endTime,
                                 const unsigned int             &order)
                                 :
                                 TrajectoryBase<DataType>(startTime, endTime, startPoint.size()),   // Construct base object
                                 _order(order)                                                      // Order of polynomial
{
	if(startPoint.size()    != endPoint.size()
	or endPoint.size()      != startVelocity.size()
	or startVelocity.size() != endVelocity.size())
	{	
		throw logic_error("[ERROR] [POLYNOMIAL] Constructor: "
	                          "Dimensons of input arguments do not match. "
		                  "Start point had " + to_string(startPoint.size()) + " elements, "
		                  "end point had " + to_string(endPoint.size()) + " elements, "
		                  "start velocity had " + to_string(startVelocity.size()) + " elements, and "
		                  "end velocity had " + to_string(endVelocity.size()) + " elements.");
	}
	else
	{
		if(not compute_coefficients(startPoint, endPoint, startVelocity, endVelocity))      // As it says on the label
		{
			throw runtime_error("[ERROR] [POLYNOMIAL] Constructor: "
			                    "Could not compute the polynomial coefficients.");
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the polynomial coefficients                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool Polynomial<DataType>::compute_coefficients(const Vector<DataType,Dynamic> &startPoint,
                                                const Vector<DataType,Dynamic> &endPoint,
                                                const Vector<DataType,Dynamic> &startVelocity,
                                                const Vector<DataType,Dynamic> &endVelocity)
{
	// Decent explanation here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (Fourth Edition)
        // Springer,  p. 258 - 276
        
        DataType dt = this->_startTime - this->_endTime;
        
        this->coeff.resize(this->_dimensions);
        
        if(this->_order == 3) // Cubic polynomial : x(t) = a*dt^0 + b*dt^1 + c*dt^2 + d*dt^3
        {
		for(int i = 0; i < this->dimensions; i++)
		{
			this->coeff[i].resize(4);
			
			DataType dx = endPoint(i) - startPoint(i);
	
			this->coeff[i][0] = startPoint(i);                                                  // a
			this->coeff[i][1] = startVelocity(i);                                               // b
			this->coeff[i][2] = 3*dx/(dt*dt) - (endVelocity(i) + 2*this->coeff[i][1])/dt;       // c
			this->coeff[i][3] = (endVelocity(i) + this->coeff[i][1])/(dt*dt) - 2*dx/(dt*dt*dt); // d
		}	
		
		return true;
        }
        else if(this->_order == 5) // Quintic polynomial: x(t) = a*dt^0 + b*dt^1 + c*dt^2 + d*dt^3 + e*dt^4 + f*dt^5
        {
        	//   d*dt^3 +    e*dt^4 +    f*dt^5 = x_f - x_0 - dt*xdot_0                         Final position constraint
        	// 3*d*dt^2 +  4*e*dt^3 +  5*f*dt^4 = xdot_f - xdot_0                               Final velocity constraint
        	// 6*d*dt   + 12*e*dt^2 + 20*f*dt^3 = 0                                             Final acceleration constraint

		// [   dt^3     dt^4     dt^5 ][d] = [ x_f - x_0 - dt*xdot_0 ]                      // y1
		// [ 3*dt^2   4*dt^3   5*dt^4 ][e] = [    xdot_f - xdot_0    ]                      // y2
		// [ 6*dt    12*dt^2  20*dt^3 ][f] = [          0            ]

		// [  1      dt      dt^2 ][d] = [ (x_f - x_0)/dt^3 - xdot_0/dt^2 ]                 y1
		// [  3    4*dt    5*dt^2 ][e] = [     (xdot_f - xdot_0)/dt^2     ]                 y2
		// [  6   12*dt   20*dt^2 ][f] = [               0                ]
		
		// Then solve the above through Gaussian elimination, starting with the bottom row
		
		for(int i = 0; i < this->dimensions; i++)
		{
			this->coeff[i].resize(6);
		
			DataType y1 = (endPoint[i] - startPoint[i])/(dt*dt*dt) - startVelocity[i]/(dt*dt);
			DataType y2 = (endVelocity[i] - startVelocity[i])/(dt*dt);
			
			this->coeff[i][0] = startPoint[i];                                          // a = x_0
			this->coeff[i][1] = startVelocity[i];                                       // b = xdot_0
			this->coeff[i][2] = 0.0;                                                    // c = 0.5*xddot_0 (assume 0)
			this->coeff[i][3] = (6*y1 - 3*y2)/(dt*dt);                                  // d
			this->coeff[i][4] = (7*y2 - 15*y1)/dt;                                      // e
			this->coeff[i][5] = 10*y1 - 4*y2;
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
template <class DataType> inline
bool Polynomial<DataType>::get_state(Vector<DataType,Dynamic> &pos,
			             Vector<DataType,Dynamic> &vel,
			             Vector<DataType,Dynamic> &acc,
			             const DataType           &time)
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
		DataType dt;
		
		if(time <= this->_startTime)    dt = 0.0;                                           // Not yet begun, stay at beginning
		else if (time < this->_endTime) dt = time - this->_startTime;                       // Somewhere inbetween...
		else                            dt = this->_startTime - this->_endTime;             // Finished, remain at end

		for(int i = 0; i < this->dimensions; i++)
		{
			for(int j = 0; j < this->_order; j++)
			{
				pos(i) +=         this->coeff[i][j]*pow(dt,j);
				vel(i) +=       j*this->coeff[i][j]*pow(dt,j-1);
				acc(i) += (j-1)*j*this->coeff[i][j]*pow(dt,j-2);
			}
		}
		
		return true;
	}
}

#endif
