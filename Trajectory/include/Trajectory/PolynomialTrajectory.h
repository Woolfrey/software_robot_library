/**
 * @file   PolynomialTrajectory.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A trajectory which is a polynomial function of time.
 */

#ifndef POLYNOMIALTRAJECTORY_H_
#define POLYNOMIALTRAJECTORY_H_

#include <Eigen/Core>                                                                               // Eigen::Vector class
#include <Math/Polynomial.h>
#include <Trajectory/TrajectoryBase.h>
#include <string>                                                                                   // std::cerr, std::cout
#include <vector>                                                                                   // std::vector

template <class DataType>
class PolynomialTrajectory : public Polynomial<DataType>,
                             public TrajectoryBase<DataType>
{
    public:
        /**
         * Constructor.
         * @param start The starting position, velocity, and acceleration for the trajectory.
         * @param end The final position, velocity, and acceleration for the trajectory.
         * @param startTime The time for which the trajectory commences.
         * @param endTime The time for which the trajectory finishes.
         * @param order The number of terms in the polynomial.
         */
        PolynomialTrajectory(const State<DataType> &startPoint,
                             const State<DataType> &endPoint,
                             const DataType &startTime,
                             const DataType &endTime,
                             const unsigned int &order);
        /**
         * Query the position, velocity, and acceleration for the given time.
         * @param time The point in time at which to evaluate the trajectory.
         * @return A State data structure containing position, velocity, and acceleration.
         */
        inline
        State<DataType>
        query_state(const DataType &time);
        
        /** Query the position for the given time.
         * @param time The point in time at which to evaluate the position.
         * @return The position as an Eigen::Vector object.
         */
        inline
        Eigen::Vector<DataType,Eigen::Dynamic>
        query_position(const DataType &time)
        {
            return query_state(time).position;                                                      // Too easy, lol ( •_•)>⌐■-■  (⌐■_■)
        }
        
     private:
     
        std::vector<Polynomial<DataType>> _polynomial;                                              // The underlying polynomial function for each dimension
        
};                                                                                                  // Semicolon required after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Constructor.                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
PolynomialTrajectory<DataType>::PolynomialTrajectory(const State<DataType> &startPoint,
                                                     const State<DataType> &endPoint,
                                                     const DataType        &startTime,
                                                     const DataType        &endTime,
                                                     const unsigned int    &order)
                                                     : TrajectoryBase<DataType>(startPoint, endPoint, startTime, endTime)
{
    // Decent explanation of polynomial trajectories here:
    // Angeles, J. (Ed.). (2014). Fundamentals of robotic mechanical systems:
    // theory, methods, and algorithms (Fourth Edition)
    // Springer,  p. 258 - 276
     
    // NOTE: Checks on the start time, dimensions are done in TrajectoryBase
    //       so we don't need to do any here.
    
    for(int i = 0; i < this->_dimensions; i++)
    {
        std::array<DataType,3> start = { startPoint.position[i],
                                         startPoint.velocity[i],
                                         startPoint.acceleration[i] };
                                     
        std::array<DataType,3> end = { endPoint.position[i],
                                       endPoint.velocity[i],
                                       endPoint.acceleration[i] };
                              
        this->_polynomial.push_back(Polynomial<DataType>(start, end, startTime, endTime, order));
    }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType>
inline
State<DataType>
PolynomialTrajectory<DataType>::query_state(const DataType &time)
{
    // Determine where we are on the trajectory
         if(time <= this->_startTime) return this->_startPoint;
    else if(time >= this->_endTime)   return this->_endPoint;
    else
    {   
        // Value to be returned
        State<DataType> state = {Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions),
                                 Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions),
                                 Eigen::Vector<DataType,Eigen::Dynamic>::Zero(this->_dimensions)};
        
        for(int i = 0; i < this->_dimensions; i++)
        {
            const auto &[pos, vel, acc] = this->_polynomial[i].evaluate_point(time);
            
            state.position[i]     = pos;
            state.velocity[i]     = vel;
            state.acceleration[i] = acc;                         
        }
        
        return state;
    }
}

#endif
