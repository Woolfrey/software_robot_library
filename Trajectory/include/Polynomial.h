/**
 * @file   Polynomial.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A trajectory which is a polynomial function of time
 */

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <TrajectoryBase.h>
#include <string>                                                                                   // std::cerr, std::cout
#include <vector>                                                                                   // std::vector

template <class DataType>
class Polynomial : public TrajectoryBase<DataType>
{
	public:
		/**
		 * A constructor where the start and end velocities are assumed zero.
		 * @param startPoint The starting position for the trajectory.
		 * @param endPoint The ending position for the trajectory.
		 * @param startTime The time when the trajectory begins.
		 * @param endTime The time when the trajectory ends.
		 * @param order The order of the polynomial (should be an odd number)
		 */
		Polynomial(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
			   const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
			   const DataType                               &startTime,
			   const DataType                               &endTime,
			   const unsigned int                           &order)
		:
		Polynomial(startPoint,
                           endPoint,
                           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(startPoint.size()),
                           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(endPoint.size()),
                           startTime,
                           endTime,
                           order) {}
		
		/** A constructor that also specifies start and end velocities.
		 * @param startPoint The starting position for the trajectory.
		 * @param endPoint The ending position for the trajectory.
		 * @param startVelocity The initial speed for the trajectory.
		 * @param endVelocity The final speed for the trajectory.
		 * @param startTime The time when the trajectory begins.
		 * @param endTime The time when the trajectory ends.
		 * @param order The order of the polynomial (should be an odd number)
		 */
		Polynomial(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
			   const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
			   const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
			   const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
			   const DataType                               &startTime,
			   const DataType                               &endTime,
			   const unsigned int                           &order);
		
		/**
		 * Compute the position on the trajectory for the given time.
		 * @param time The point at which to calculate the position.
		 * @return The position as an Eigen::Vector object
		 */
		Eigen::Vector<DataType,Eigen::Dynamic> query_position(const DataType &time)
		{
			return query_state(time).position;                                          // Too easy lol ( •_•) ( •_•)>⌐■-■ (⌐■_■)
		}
		
		/**
		 * Query the current state (position, velocity, acceleration) for the given time.
		 * @param time The time at which to query the state.
		 * @return Returns a State data structure.
		 */
		State<DataType> query_state(const DataType &time);
		
	private:
		unsigned int _order;                                                                ///< The order of the polynomial
		
		std::vector<std::vector<DataType>> coeff;                                           ///< An array of polynomial coefficients
		
		/**
		 * Computes the polynomial coefficients given the initial and final state.
		 * @param startPoint The starting position for the trajectory.
		 * @param endPoint The ending position for the trajectory.
		 * @param startVelocity The initial speed for the trajectory.
		 * @param endVelocity The final speed for the trajectory.
		 */
		bool compute_coefficients(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
					  const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
					  const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
					  const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity);
		
};                                                                                                  // Semicolon needed after class declaration


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Cosntructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Polynomial<DataType>::Polynomial(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
                                 const DataType                               &startTime,
                                 const DataType                               &endTime,
                                 const unsigned int                           &order)
                                 :
                                 TrajectoryBase<DataType>(startTime, endTime, startPoint.size()),   // Construct base object
                                 _order(order)                                                      // Order of polynomial
{
	if(startPoint.size()    != endPoint.size()
	or endPoint.size()      != startVelocity.size()
	or startVelocity.size() != endVelocity.size())
	{	
		throw std::logic_error("[ERROR] [POLYNOMIAL] Constructor: "
	                               "Dimensons of input arguments do not match. "
		                       "Start point had " + std::to_string(startPoint.size()) + " elements, "
		                       "end point had " + std::to_string(endPoint.size()) + " elements, "
		                       "start velocity had " + std::to_string(startVelocity.size()) + " elements, and "
		                       "end velocity had " + std::to_string(endVelocity.size()) + " elements.");
	}
	else
	{
		if(not compute_coefficients(startPoint, endPoint, startVelocity, endVelocity))      // As it says on the label
		{
			throw std::runtime_error("[ERROR] [POLYNOMIAL] Constructor: "
			                         "Could not compute the polynomial coefficients.");
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Compute the polynomial coefficients                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
bool Polynomial<DataType>::compute_coefficients(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
                                                const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
                                                const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
                                                const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity)
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
        	std::cerr << "[ERROR] [POLYNOMIAL] compute_coefficients(): "
        	          << "Polynomial order was " << this->_order << " but currently "
        	          << "only 3 or 5 is available." << std::endl;
        	          
	        return false;
	}

}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
State<DataType> Polynomial<DataType>::query_state(const DataType &time)
{
	// Value to be returned
	State<DataType> state = {Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions),
	                         Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions),
	                         Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions)};
	
	// Determine where we are on the trajectory               
	DataType dt;
	
	if(time <= this->_startTime)    dt = 0.0;                                           // Not yet begun, stay at beginning
	else if (time < this->_endTime) dt = time - this->_startTime;                       // Somewhere inbetween...
	else                            dt = this->_startTime - this->_endTime;             // Finished, remain at end

	// Interpolate along the trajectory for the given time
	for(int i = 0; i < this->dimensions; i++)
	{
		for(int j = 0; j < this->_order; j++)
		{
			state.position(i)     +=         this->coeff[i][j]*pow(dt,j);
			state.velocity(i)     +=       j*this->coeff[i][j]*pow(dt,j-1);
			state.acceleration(i) += (j-1)*j*this->coeff[i][j]*pow(dt,j-2);
		}
	}
	
	return state;
}

#endif
