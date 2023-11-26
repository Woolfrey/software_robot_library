/**
 * @file   Polynomial.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A trajectory which is a polynomial function of time.
 */

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <Eigen/Dense>                                                                              // Eigen::PartialPivLU
#include <TrajectoryBase.h>
#include <string>                                                                                   // std::cerr, std::cout
#include <vector>                                                                                   // std::vector

template <class DataType>
class Polynomial : public TrajectoryBase<DataType>
{
	public:
		/**
		 * A constructor that specifies start and end positions.
		 * Velocity and acceleration are assumed to be zero.
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
		Polynomial(startPoint, endPoint,
                           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(startPoint.size()),
                           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(endPoint.size()),
                           startTime, endTime, order) {}
		
		/**
		 * A constructor that specifies positions and velocities at the endpoints.
		 * Acceleration is assumed zero.
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
			   const unsigned int                           &order)
		:
		Polynomial(startPoint, endPoint, startVelocity, endVelocity,
		           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(startPoint.size()),
		           Eigen::Vector<DataType,Eigen::Dynamic>::Zero(endPoint.size()),
		           startTime, endTime, order) {}
		
		/**
		 * A constructor that specifies position, velocity and acceleration at the start and end.
		 * @param startPoint The starting position for the trajectory.
		 * @param endPoint The ending position for the trajectory.
		 * @param startVelocity The initial speed for the trajectory.
		 * @param endVelocity The final speed for the trajectory.
		 * @param startAcceleration The initial acceleration for the trajectory.
		 * @param endAcceleration The final acceleration for the trajectory.
		 * @param startTime The time when the trajectory begins.
		 * @param endTime The time when the trajectory ends.
		 * @param order The order of the polynomial (should be an odd number)
		 */
		 Polynomial(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
			    const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
			    const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
			    const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
			    const Eigen::Vector<DataType,Eigen::Dynamic> &startAcceleration,
			    const Eigen::Vector<DataType,Eigen::Dynamic> &endAcceleration,
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
			return query_state(time).position;                                          // Too easy lol ( •_•)  ( •_•)>⌐■-■  (⌐■_■)
		}
		
		/**
		 * Query the current state (position, velocity, acceleration) for the given time.
		 * @param time The time at which to query the state.
		 * @return Returns a State data structure.
		 */
		State<DataType> query_state(const DataType &time);
		
	private:
		unsigned int _order;                                                                ///< The order of the polynomial
		
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _coefficients;                ///< Array of coefficients for every dimension
};                                                                                                  // Semicolon needed after class declaration


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           Cosntructor                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
Polynomial<DataType>::Polynomial(const Eigen::Vector<DataType,Eigen::Dynamic> &startPoint,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &endPoint,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &startVelocity,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &endVelocity,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &startAcceleration,
                                 const Eigen::Vector<DataType,Eigen::Dynamic> &endAcceleration,
                                 const DataType                               &startTime,
                                 const DataType                               &endTime,
                                 const unsigned int                           &order)
                                 :
                                 TrajectoryBase<DataType>(startTime, endTime, startPoint.size()),   // Construct base object
                                 _order(order)                                                      // Order of polynomial
{
	using namespace Eigen;                                                                      // Eigen::Matrix, Eigen::Vector, Eigen::Dynamic
	
	this->_coefficients.resize(this->_dimensions,this->_order+1);                               // NOTE: A nth order polynomial has n+1 coefficients
	
	if(startPoint.size()        != endPoint.size()
	or endPoint.size()          != startVelocity.size()
	or startVelocity.size()     != endVelocity.size()
	or endVelocity.size()       != startAcceleration.size()
	or startAcceleration.size() != endAcceleration.size())
	{	
		throw std::logic_error("[ERROR] [POLYNOMIAL] Constructor: "
	                               "Dimensons of input arguments do not match. "
		                       "The start point had " + std::to_string(startPoint.size()) + " elements, "
		                       "the end point had " + std::to_string(endPoint.size()) + " elements, "
		                       "the start velocity had " + std::to_string(startVelocity.size()) + " elements, "
		                       "the end velocity had " + std::to_string(endVelocity.size()) + " elements. "
		                       "the start acceleration had " + std::to_string(startAcceleration.size()) + " elements, and "
		                       "the end acceleration had " + std::to_string(endAcceleration.size()) + " elements.");
	}
	else if(order%2 == 0)
	{
		throw std::invalid_argument("[ERROR] [POLYNOMIAL] Constructor: "
		                            "Order was " + std::to_string(order) + " but it must be an odd number.");
	}
	
	// Decent explanation of polynomial trajectories here:
        // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
        // theory, methods, and algorithms (Fourth Edition)
        // Springer,  p. 258 - 276
        
        unsigned int n = (this->_order+1)/2;                                                        // Makes indexing for start and end points easier...
        
        Matrix<DataType,Dynamic,Dynamic> timeMatrix(this->_order+1,this->_order+1);                 // Needed to solve polynomial coefficients
       
        timeMatrix.setZero();

        for(int i = 0; i < n; i++)
        {
        	for(int j = i; j < timeMatrix.cols(); j++)
        	{
        		DataType derivativeCoeff;
        		
        		     if(i == 0) derivativeCoeff = 1.0;
        		else if(i == 1) derivativeCoeff = j;
        		else
        		{
        			derivativeCoeff = 1.0;
        			for(int k = 0; k < i; k++) derivativeCoeff *= (j-k);
        		}
        		
        		timeMatrix(i,j)   = derivativeCoeff*pow(this->_startTime,j-i);
        		timeMatrix(i+n,j) = derivativeCoeff*pow(this->_endTime,j-i);
        	}
        }
        
        PartialPivLU<Matrix<DataType,Dynamic,Dynamic>> timeMatrixDecomp(timeMatrix);                // Pre-compute to speed up calcs
        
        Vector<DataType,Dynamic> supportPoints(this->_order+1); supportPoints.setZero();
        
        for(int i = 0; i < this->_dimensions; i++)
        {
	  	supportPoints(0)   = startPoint(i);
	  	supportPoints(n)   = endPoint(i);
	  	
	  	if(i > 0)
	  	{
	  		supportPoints(1)   = startVelocity(i);
	  		supportPoints(1+n) = endVelocity(i);
	  	}
	  	
	  	if(i > 1)
	  	{
	  		supportPoints(2)   = startAcceleration(i);
	  		supportPoints(2+n) = endAcceleration(i);
	  	}
	  
 		this->_coefficients.row(i) = (timeMatrixDecomp.solve(supportPoints)).transpose();
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
	DataType t;
	     if(time <  this->_startTime) t = this->_startTime;                                     // Not yet started
	else if(time >= this->_endTime)   t = this->_endTime;                                       // Finished
	else                              t = time;                                                 // Somewhere in between
	
	// Interpolate along the trajectory for the given time
	for(int i = 0; i < this->_dimensions; i++)
	{
		for(int j = i; j < this->_order+1; j++)
		{
			          state.position(i)     +=         this->_coefficients(i,j)*pow(t,j-0);
			if(j > 0) state.velocity(i)     +=       j*this->_coefficients(i,j)*pow(t,j-1);
			if(j > 1) state.acceleration(i) += (j-1)*j*this->_coefficients(i,j)*pow(t,j-2);
		}
	}
	
	return state;
}

#endif
