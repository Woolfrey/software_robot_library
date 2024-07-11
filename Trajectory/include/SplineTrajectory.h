/**
 * @file   SplineTrajectory.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class defining trajectories with waypoints joined over splines.
 */
 
#ifndef SPLINETRAJECTORY_H_
#define SPLINETRAJECTORY_H_

#include "MathFunctions.h"
#include "Spline.h"
#include "TrajectoryBase.h"

class SplineTrajectory : public TrajectoryBase
{
    public:
        
        /**
         * An empty constructor.
         */
        SplineTrajectory() {}
        
        /**
         * Constructor.
         * @param waypoints The states (position, velocity, acceleration) for given points in time.
         * @param times The time at which to pass through each waypoint.
         * @param polynomialOrder The type of polynomial interpolation to use.
         */
        SplineTrajectory(const std::vector<State>  &waypoints,
                         const std::vector<double> &times,
                         const unsigned int        &polynomialOrder);
        
        /**
         * A basic constructor for a cubic spline where the waypoints and start velocity are given.
         * The final velocity is assumed to be zero.
         * @param positions An array of positions to pass through.
         * @param times The time at which to pass through each position.
         * @param startVelocity The initial velocity of the trajectory.
         */
        SplineTrajectory(const std::vector<Eigen::VectorXd> &positions,
                         const std::vector<double> &times,
                         const Eigen::VectorXd &startVelocity);
                                                
        /**
         * Query the state for the given input time.
         * This overrides the virtual function in the TrajectoryBase class.
         * @param time The point at which to evaluate the trajectory.
         * @return The position, velocity, acceleration as a State data structure.
         */
        State
        query_state(const double &time);
    
    private:
        
        std::vector<Spline> _spline;                                                                ///< Underlying spline object over real numbers
        
};                                                                                                  // Semicolon needed after a class declarationS

#endif
